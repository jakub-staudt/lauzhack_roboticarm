"""The arm agent: the part of the real-arm drive that should run NEXT TO THE ARM (on the RAMION server), not on a laptop.

Why: planning a move and sending its ~70 joint steps (each with heartbeats and state polls) needs hundreds of round trips. Over
the relayed Tailscale link (40 to 600 ms) that made the arm crawl, and one lag spike lets the lease (1.5 s) expire mid-move. Here
those round trips go to the controller on localhost (about 1 ms). The laptop sends ONE small command ("go to x, y, 3 cm") and
Stop, and polls the state.

* Talks to the controller's dashboard on this machine, http://100.85.56.14:8780 (override with --controller). Same API as before.
* Reuses `RealArmClient` unchanged, so every safety rule stays: hover >= 3 cm, joint-limit/gripper/e-stop/other-owner checks,
  control claimed only after planning, soft stop, release after the move.
* Listens on one address only (give it the Tailscale IP or 127.0.0.1) and requires a shared token on every request.
* It never changes torque or the controller's speed, never latches the emergency stop, and exits cleanly on SIGTERM.
* Its own dead-man: a move needs the laptop to keep asking /state; if nobody has asked for `--deadman-seconds` during a move,
  the agent holds the arm (soft stop).
"""

from __future__ import annotations

import argparse
import hmac
import json
import signal
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

MAX_BODY = 4096


class AgentState:
    def __init__(self, client, token: str, deadman_s: float) -> None:
        self.client, self.token, self.deadman_s = client, token, deadman_s
        self.last_contact = time.monotonic()
        self.lock = threading.Lock()

    def touch(self) -> None:
        self.last_contact = time.monotonic()

    def watchdog(self, stop: threading.Event) -> None:
        fired = False
        while not stop.wait(0.25):
            try:
                busy = self.client.mover.busy()
            except Exception:
                busy = False
            silent = time.monotonic() - self.last_contact > self.deadman_s
            if busy and silent and not fired:
                fired = True
                try:
                    self.client.stop()
                except Exception:
                    pass
            if not silent:
                fired = False


def make_handler(agent: AgentState):
    class Handler(BaseHTTPRequestHandler):
        protocol_version = "HTTP/1.1"

        def log_message(self, fmt, *args):  # one short line per request, no bodies
            print(f"{time.strftime('%H:%M:%S')} {self.address_string()} {fmt % args}", flush=True)

        def _send(self, code: int, body: dict) -> None:
            data = json.dumps(body).encode()
            self.send_response(code)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(data)))
            self.end_headers()
            self.wfile.write(data)

        def _authorised(self) -> bool:
            return hmac.compare_digest(self.headers.get("X-Agent-Token", ""), agent.token)

        def _body(self) -> dict:
            n = int(self.headers.get("Content-Length") or 0)
            if n > MAX_BODY:
                raise ValueError("body too large")
            return json.loads(self.rfile.read(n) or b"{}")

        def do_GET(self) -> None:
            if not self._authorised():
                return self._send(401, {"error": "bad token"})
            if self.path == "/health":
                return self._send(200, {"ok": True})
            if self.path == "/state":
                agent.touch()
                try:
                    return self._send(200, agent.client.state())
                except Exception as exc:
                    return self._send(502, {"error": f"controller not reachable: {exc}"})
            self._send(404, {"error": "not found"})

        def do_POST(self) -> None:
            if not self._authorised():
                return self._send(401, {"error": "bad token"})
            try:
                body = self._body()
            except ValueError as exc:
                return self._send(400, {"error": str(exc)})
            agent.touch()
            if self.path == "/stop":                      # always accepted, never queued behind a move
                try:
                    return self._send(200, agent.client.stop())
                except Exception as exc:
                    return self._send(502, {"error": str(exc)})
            if self.path == "/go":
                try:
                    x, y, h = float(body["x"]), float(body["y"]), float(body["height_cm"])
                except (KeyError, TypeError, ValueError):
                    return self._send(400, {"error": "x, y and height_cm are required numbers"})
                with agent.lock:
                    try:
                        return self._send(200, agent.client.go_to_base(x, y, h))
                    except RuntimeError as exc:            # MoveError: a plain reason the move was refused
                        return self._send(409, {"error": str(exc)})
            self._send(404, {"error": "not found"})

    return Handler


def load_config(path: str) -> tuple[list[tuple[float, float]], float]:
    cfg = json.loads(Path(path).read_text())
    return [(float(p[0]), float(p[1])) for p in cfg["points_xy"]], float(cfg["z_table"])


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--config", required=True, help="JSON {points_xy: [[x, y], ...], z_table}: the calibrated area (made by tools/make_agent_config.py)")
    ap.add_argument("--token-file", required=True, help="file holding the shared secret (the laptop sends it in X-Agent-Token)")
    ap.add_argument("--host", default="127.0.0.1", help="the ONE address to listen on (the Tailscale IP, or 127.0.0.1)")
    ap.add_argument("--port", type=int, default=8790)
    ap.add_argument("--controller", default="http://100.85.56.14:8780", help="the dashboard on this machine (it listens on the Tailscale address, not on 127.0.0.1)")
    ap.add_argument("--deadman-seconds", type=float, default=4.0)
    ap.add_argument("--pause-s", type=float, default=0.35)
    ap.add_argument("--max-step-deg", type=float, default=3.0)
    args = ap.parse_args()

    from robotic_arm.clickmove import pooled_http          # the arm's kinematics: soarm_lab must be importable
    from soarm_lab.kinematics import RobotKinematics
    from soarm_lab.robot import load_robot

    from lauzkach_eye_robot_control.realarm import RealArmClient, RealSettings

    points, z_table = load_config(args.config)
    token = Path(args.token_file).read_text().strip()
    if len(token) < 16:
        raise SystemExit("the token must be at least 16 characters")
    client = RealArmClient(pooled_http(args.controller), RobotKinematics(load_robot()), points, z_table,
                           RealSettings(pause_s=args.pause_s, max_step_deg=args.max_step_deg), label="Camera drive (server agent)")
    agent = AgentState(client, token, args.deadman_seconds)
    stop = threading.Event()
    threading.Thread(target=agent.watchdog, args=(stop,), daemon=True, name="deadman").start()
    server = ThreadingHTTPServer((args.host, args.port), make_handler(agent))

    def shutdown(*_):
        try:
            client.stop()
        except Exception:
            pass
        stop.set()
        threading.Thread(target=server.shutdown, daemon=True).start()

    signal.signal(signal.SIGTERM, shutdown)
    signal.signal(signal.SIGINT, shutdown)
    print(f"arm agent on http://{args.host}:{args.port}, controller {args.controller}", flush=True)
    server.serve_forever()


if __name__ == "__main__":
    main()
