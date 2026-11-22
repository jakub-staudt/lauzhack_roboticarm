"""Local web UI server: serves ./static and proxies a small allowlist of controller routes.

The browser cannot call the controller directly: its JSON POSTs need a CORS preflight that it never grants. This server is
same-origin with the page and forwards only the routes the UI needs. It does not modify or restart anything on the server; it is a
client of the public HTTP API, like the RAMION web apps.

  ../robotic_arm/.venv/bin/python server.py           # controller via the server's dashboard (enables click-to-move)
  python3 server.py                                   # UI only, no click-to-move
  python3 server.py --controller http://127.0.0.1:8780 --port 8800     # e.g. through an SSH tunnel

Deliberately NOT forwarded: torque, joint target, speed, model/*, cameras (re-assignment), view.
Joint jog is forwarded only for wrist_roll in steps of at most 20 degrees (to back off its end stop).
Click-to-move (/api/mvp/*) is handled here and talks to the controller itself with the token the page passes in.
"""

from __future__ import annotations

import argparse
import http.client
import json
import mimetypes
import socket
import sys
import threading
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.error import HTTPError, URLError
from urllib.parse import parse_qs, urlsplit
from urllib.request import Request, urlopen

STATIC = Path(__file__).parent / "static"

try:  # click-to-move needs the robotic_arm package (MuJoCo kinematics); the rest of the UI works without it
    from robotic_arm.clickmove import ImagePlane, MoveError, Mover, pooled_http
    from soarm_lab.kinematics import RobotKinematics
    from soarm_lab.robot import load_robot
    MVP_ERROR = None
except Exception as exc:  # noqa: BLE001
    MoveError = RuntimeError  # type: ignore[assignment,misc]
    MVP_ERROR = f"{type(exc).__name__}: {exc}"

# The server's dashboard (port 8780) exposes the same /api and /cameras routes as the controller (8765), keeps connections alive and talks to
# the controller on the server itself, which is what the RAMION web apps use. Going through it roughly halves the latency of each call.
DEFAULT_CONTROLLER = "http://100.85.56.14:8780"

GET_ALLOWED = {"/api/state", "/cameras/overhead.jpg", "/cameras/overhead.mjpg", "/cameras/wrist.jpg", "/cameras/wrist.mjpg"}
POST_ALLOWED = {
    "/api/control/claim", "/api/control/heartbeat", "/api/control/release",
    "/api/cartesian/start", "/api/cartesian/renew", "/api/cartesian/stop",
    "/api/emergency-stop", "/api/emergency-stop/reset", "/api/stop",
    "/api/jog",  # restricted below: wrist_roll only, small steps
}


class Handler(BaseHTTPRequestHandler):
    controller = DEFAULT_CONTROLLER
    mover = None  # set in main() when the click-to-move package is available
    protocol_version = "HTTP/1.1"
    _upstream = threading.local()  # one kept-alive connection per request thread

    # ---------------------------------------------------------------- helpers
    def _send(self, status: int, body: bytes, content_type: str = "application/json") -> None:
        self.send_response(status)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        try:
            self.wfile.write(body)
        except (BrokenPipeError, ConnectionResetError):
            self.close_connection = True

    def _error(self, status: int, message: str) -> None:
        self._send(status, json.dumps({"error": message}).encode())

    def _same_origin(self) -> bool:
        origin = self.headers.get("Origin")
        return origin is None or urlsplit(origin).netloc == self.headers.get("Host")

    # ---------------------------------------------------------------- GET
    def do_GET(self) -> None:
        path = urlsplit(self.path).path
        if path == "/api/mvp/status":
            if self.mover is None:
                return self._send(HTTPStatus.OK, json.dumps({"available": False, "error": MVP_ERROR}).encode())
            return self._send(HTTPStatus.OK, json.dumps({"available": True, "move": self.mover.status(),
                                                         "plane": self.mover.plane.summary()}).encode())
        if path in GET_ALLOWED:
            if path.endswith(".mjpg"):
                opts = parse_qs(urlsplit(self.path).query, keep_blank_values=True)
                # The dashboard streams at 2 fps unless asked: ?fps=<2|10|15|20>&profile=preview (what the RAMION game requests).
                if (set(opts) - {"fps", "profile", "t"} or opts.get("fps", ["2"])[0] not in {"2", "10", "15", "20"}
                        or opts.get("profile", ["preview"])[0] != "preview" or not opts.get("t", ["0"])[0].isdigit()):
                    return self._error(HTTPStatus.BAD_REQUEST, "unsupported stream options")
                upstream = "&".join(f"{k}={opts[k][0]}" for k in ("fps", "profile") if k in opts)  # "t" is only a cache-buster
                return self._stream(path + ("?" + upstream if upstream else ""))
            return self._forward("GET", path)
        if path == "/":
            path = "/index.html"
        target = (STATIC / path.lstrip("/")).resolve()
        if STATIC.resolve() not in target.parents or not target.is_file():
            return self._error(HTTPStatus.NOT_FOUND, f"not found: {path}")
        ctype = mimetypes.guess_type(target.name)[0] or "application/octet-stream"
        self._send(HTTPStatus.OK, target.read_bytes(), ctype)

    # ---------------------------------------------------------------- POST
    def do_POST(self) -> None:
        path = urlsplit(self.path).path
        if path.startswith("/api/mvp/"):
            return self._mvp_post(path)
        if path not in POST_ALLOWED:
            return self._error(HTTPStatus.NOT_FOUND, f"route not available through this UI: {path}")
        if not self._same_origin():
            return self._error(HTTPStatus.FORBIDDEN, "cross-origin request refused")
        if self.headers.get_content_type() != "application/json":
            return self._error(HTTPStatus.UNSUPPORTED_MEDIA_TYPE, "expected application/json")
        length = int(self.headers.get("Content-Length") or 0)
        if not 0 <= length <= 65536:
            return self._error(HTTPStatus.REQUEST_ENTITY_TOO_LARGE, "body too large")
        raw = self.rfile.read(length)
        if path == "/api/jog":
            try:
                body = json.loads(raw)
                delta = body.get("delta")
                ok = (body.get("joint") == "wrist_roll" and isinstance(delta, (int, float))
                      and not isinstance(delta, bool) and abs(delta) <= 20)
            except (ValueError, AttributeError):
                ok = False
            if not ok:
                return self._error(HTTPStatus.FORBIDDEN, "only wrist_roll nudges of at most 20 degrees are allowed here")
        self._forward("POST", path, raw)

    def _mvp_post(self, path: str) -> None:
        if self.mover is None:
            return self._error(HTTPStatus.SERVICE_UNAVAILABLE, f"click-to-move unavailable: {MVP_ERROR}")
        if not self._same_origin():
            return self._error(HTTPStatus.FORBIDDEN, "cross-origin request refused")
        if self.headers.get_content_type() != "application/json":
            return self._error(HTTPStatus.UNSUPPORTED_MEDIA_TYPE, "expected application/json")
        try:
            length = int(self.headers.get("Content-Length") or 0)
            body = json.loads(self.rfile.read(length)) if 0 < length <= 65536 else {}
            if not isinstance(body, dict):
                raise ValueError("expected a JSON object")
            num = lambda k: float(body[k])  # noqa: E731
            if path == "/api/mvp/calpoint":
                result = {"plane": self.mover.record_spot(num("px"), num("py"))}
            elif path == "/api/mvp/calundo":
                self.mover.plane.undo()
                result = {"plane": self.mover.plane.summary()}
            elif path == "/api/mvp/calclear":
                self.mover.plane.clear()
                result = {"plane": self.mover.plane.summary()}
            elif path == "/api/mvp/click":
                token = body.get("token")
                if not isinstance(token, str):
                    raise MoveError("Take control first.")
                result = {"move": self.mover.start_click(token, num("px"), num("py"))}
            elif path == "/api/mvp/nudge":
                token = body.get("token")
                if not isinstance(token, str):
                    raise MoveError("Take control first.")
                result = {"move": self.mover.start_nudge(token, num("dz"))}
            elif path == "/api/mvp/cancel":
                self.mover.cancel()
                result = {"move": self.mover.status()}
            else:
                return self._error(HTTPStatus.NOT_FOUND, f"not found: {path}")
        except MoveError as exc:
            return self._error(HTTPStatus.CONFLICT, str(exc))
        except (KeyError, ValueError, TypeError) as exc:
            return self._error(HTTPStatus.BAD_REQUEST, f"bad request: {exc}")
        self._send(HTTPStatus.OK, json.dumps(result).encode())

    def _forward(self, method: str, path: str, body: bytes | None = None) -> None:
        parts = urlsplit(self.controller)
        headers = {"Content-Type": "application/json"} if body is not None else {}
        for attempt in (0, 1):  # a kept-alive connection may have been closed by the server: retry once on a fresh one
            conn = getattr(self._upstream, "conn", None)
            if conn is None:
                conn = self._upstream.conn = http.client.HTTPConnection(parts.hostname, parts.port or 80, timeout=4.0)
            try:
                conn.request(method, path, body=body, headers=headers)
                response = conn.getresponse()
                payload = response.read()
            except (http.client.HTTPException, OSError) as exc:
                conn.close()
                self._upstream.conn = None
                if attempt == 1:
                    return self._error(HTTPStatus.BAD_GATEWAY, f"controller unavailable: {type(exc).__name__}")
                continue
            if response.status >= 400:  # the controller's refusal, passed through with its message
                sys.stderr.write(f"controller refused {method} {path}: {response.status} {payload[:200].decode(errors='replace')}\n")
            return self._send(response.status, payload or b"{}", response.headers.get_content_type())

    def _stream(self, path: str) -> None:
        try:
            upstream = urlopen(Request(self.controller + path), timeout=5.0)
        except (HTTPError, URLError, TimeoutError, OSError) as exc:
            return self._error(HTTPStatus.BAD_GATEWAY, f"camera unavailable: {type(exc).__name__}")
        self.send_response(HTTPStatus.OK)
        self.send_header("Content-Type", upstream.headers.get("Content-Type", "multipart/x-mixed-replace"))
        self.send_header("Cache-Control", "no-store")
        self.send_header("Connection", "close")
        self.end_headers()
        self.close_connection = True
        try:
            while chunk := upstream.read(16384):
                self.wfile.write(chunk)
        except (BrokenPipeError, ConnectionResetError, socket.timeout, OSError):
            pass
        finally:
            upstream.close()

    def log_message(self, format: str, *args) -> None:  # keep the terminal readable; errors still print
        if args and str(args[1]).startswith(("4", "5")):
            sys.stderr.write("%s - %s\n" % (self.address_string(), format % args))


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--controller", default=DEFAULT_CONTROLLER)
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=8800)
    args = ap.parse_args()
    Handler.controller = args.controller.rstrip("/")
    if MVP_ERROR is None:
        Handler.mover = Mover(pooled_http(Handler.controller), RobotKinematics(load_robot()),
                              ImagePlane(Path(__file__).parent / "mvp_plane.json"))
    else:
        print(f"click-to-move disabled ({MVP_ERROR}); start with ../robotic_arm/.venv/bin/python to enable it")
    server = ThreadingHTTPServer((args.host, args.port), Handler)
    server.daemon_threads = True
    print(f"UI on http://{args.host}:{args.port}  ->  controller {Handler.controller}")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
