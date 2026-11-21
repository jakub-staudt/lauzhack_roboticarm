"""Web UI for the Gazebo gaze station: tap a block, tap a destination, and the simulated SO-101 moves it.

    robotic_arm/.venv/bin/python -m simulator.web.server            # http://127.0.0.1:8810
    robotic_arm/.venv/bin/python -m simulator.web.server --host 0.0.0.0   # also reachable from a phone on your network

By default it simulates the station with MuJoCo and needs only `pip install -r simulator/requirements.txt`. With
`--backend gazebo` it drives a running Gazebo instead (`python3 -m simulator.gazebo.run`). The server shows the simulated
overhead camera, turns taps into table positions, plans the pick-and-place with the MuJoCo kinematics and sends joint
targets to the simulator.
"""

from __future__ import annotations

import argparse
import json
import mimetypes
import os
import struct
import sys
import threading
import time
import zlib

if sys.platform.startswith("linux"):  # headless-friendly rendering; set MUJOCO_GL yourself to override
    os.environ.setdefault("MUJOCO_GL", "egl")
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import urlsplit

import numpy as np

from simulator import frame
from simulator.planner import Planner, Unreachable

STATIC = Path(__file__).resolve().parent / "static"
REACH_MAP = Path(__file__).resolve().parent / "reach_map.json"
COLORS = ("red", "blue", "yellow")
SNAP_M = 0.035        # a tap this close to a cube selects it
CLEARANCE_M = 0.03    # a destination this close to another cube is refused


def png_bytes(rgb: np.ndarray) -> bytes:
    h, w, _ = rgb.shape
    raw = np.concatenate([np.zeros((h, 1), np.uint8), rgb.reshape(h, w * 3)], axis=1).tobytes()

    def chunk(kind: bytes, body: bytes) -> bytes:
        return struct.pack(">I", len(body)) + kind + body + struct.pack(">I", zlib.crc32(kind + body))

    return (b"\x89PNG\r\n\x1a\n" + chunk(b"IHDR", struct.pack(">IIBBBBB", w, h, 8, 2, 0, 0, 0))
            + chunk(b"IDAT", zlib.compress(raw, 1)) + chunk(b"IEND", b""))


class Station:
    """Gazebo + planner + the single running task."""

    def __init__(self, backend: str = "mujoco") -> None:
        self.planner = Planner()
        if backend == "gazebo":  # optional: needs Gazebo (and on Linux the ROS Jazzy bindings); the default needs neither
            from simulator.gazebo.client import GazeboStation
            self.sim = GazeboStation()
        else:
            from simulator.mujoco_backend import MujocoStation
            self.sim = MujocoStation()
        self.sim.wait_ready(timeout=60)
        self.lock = threading.Lock()
        self.cancel = threading.Event()
        self.task = {"state": "idle", "label": "", "step": 0, "total": 0, "message": "Ready."}
        self.initial = {c: [float(v) for v in self.planner.model.body(f"{c}_block").pos] for c in COLORS}
        self.frame_png = b""
        self.frame_lock = threading.Lock()

    # ------------------------------------------------------------------ state
    def blocks(self) -> list[dict]:
        out = []
        for color in COLORS:
            try:
                x, y, z = self.sim.block(color)
            except KeyError:
                x, y, z = self.initial[color]
            out.append({"color": color, "x": x, "y": y, "z": z, "label": frame.describe(x, y), "held": z > 0.775})
        return out

    def state(self) -> dict:
        return {"task": dict(self.task), "blocks": self.blocks(),
                "joints": {k: round(v, 3) for k, v in self.sim.joints().items()}}

    def png(self) -> bytes:
        return png_bytes(self.sim.image("overhead"))

    # ------------------------------------------------------------------ tasks
    def _set(self, **kw) -> None:
        self.task.update(kw)

    def _start(self, name: str, build) -> dict:
        if not self.lock.acquire(blocking=False):
            raise Busy("The arm is busy. Press Stop, or wait for it to finish.")
        self.cancel.clear()
        self._set(state="planning", label=name, step=0, total=0, message="Planning…")

        def worker() -> None:
            try:
                plan = build()
                self._set(state="running", total=len(plan), message=name)
                deadline = time.monotonic()
                for i, (label, q, dwell) in enumerate(plan):
                    if self.cancel.is_set():
                        break
                    self.sim.command(q)
                    self._set(step=i + 1, label=label)
                    deadline += dwell  # absolute schedule, so per-waypoint overheads do not add up
                    if self.cancel.wait(max(0.0, deadline - time.monotonic())):
                        break
                if self.cancel.is_set():
                    hold = {k: v for k, v in self.sim.joints().items() if k != "gripper"}
                    self.sim.command(hold)  # freeze the arm where it is; the gripper keeps its grip
                    self._set(state="stopped", message="Stopped.")
                else:
                    self._set(state="done", label="", message=f"{name}: done.")
            except Unreachable as exc:
                self._set(state="error", label="", message=str(exc))
            except Exception as exc:  # noqa: BLE001 - report anything to the UI instead of dying silently
                self._set(state="error", label="", message=f"{type(exc).__name__}: {exc}")
            finally:
                self.lock.release()

        threading.Thread(target=worker, daemon=True).start()
        return self.task

    def pick_and_place(self, src_px, dst_px) -> dict:
        sx, sy = frame.pixel_to_world(*src_px)
        dx, dy = frame.pixel_to_world(*dst_px)
        blocks = self.blocks()
        near = min(blocks, key=lambda b: (b["x"] - sx) ** 2 + (b["y"] - sy) ** 2)
        if (near["x"] - sx) ** 2 + (near["y"] - sy) ** 2 > SNAP_M**2:
            raise BadRequest("There is no block at the first spot. Tap a block.")
        if near["held"]:
            raise BadRequest("That block is not on the table.")
        hit = frame.world_to_block(dx, dy, margin_cm=1.0)
        if hit is None or hit[0] != "left":
            raise BadRequest("Tap the destination inside the block.")
        for other in blocks:
            if other is not near and (other["x"] - dx) ** 2 + (other["y"] - dy) ** 2 < CLEARANCE_M**2:
                raise BadRequest(f"The {other['color']} block is in the way at the destination.")
        src, dst = (near["x"], near["y"]), (dx, dy)
        name = f"Move {near['color']} {frame.describe(*src)} → {frame.describe(*dst)}"
        return self._start(name, lambda: self.planner.pick_and_place(self.sim.joints(), src, dst))

    def go_to(self, px) -> dict:
        x, y = frame.pixel_to_world(*px)
        return self._start(f"Move tip to {frame.describe(x, y)}", lambda: self.planner.go_to(self.sim.joints(), (x, y)))

    def go_home(self) -> dict:
        def build():
            plan: list = []
            home = {**self.planner.home, "gripper": self.planner.to_app(self.sim.joints())["gripper"]}
            self.planner._joint_move(plan, "home", self.planner.to_app(self.sim.joints()), home)
            return plan
        return self._start("Home", build)

    def stop(self) -> dict:
        self.cancel.set()
        return self.task

    def reset_blocks(self) -> dict:
        if self.lock.locked():
            raise Busy("Stop the arm before resetting the blocks.")
        self.sim.reset_blocks(self.initial)
        self._set(state="idle", message="Blocks reset.")
        return self.task


class BadRequest(Exception):
    pass


class Busy(Exception):
    pass


class Handler(BaseHTTPRequestHandler):
    station: Station
    protocol_version = "HTTP/1.1"

    def _send(self, status: int, body: bytes, ctype: str = "application/json") -> None:
        self.send_response(status)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        try:
            self.wfile.write(body)
        except (BrokenPipeError, ConnectionResetError):
            self.close_connection = True

    def _json(self, obj, status: int = 200) -> None:
        self._send(status, json.dumps(obj).encode())

    def do_GET(self) -> None:
        path = urlsplit(self.path).path
        if path == "/frame.png":
            return self._send(200, self.station.png(), "image/png")
        if path == "/api/state":
            return self._json(self.station.state())
        if path == "/api/config":
            reach = json.loads(REACH_MAP.read_text()) if REACH_MAP.exists() else None
            return self._json({"cam_pos": frame.CAM_POS, "cam_focal": frame.CAM_FOCAL, "cam_size": frame.CAM_SIZE,
                               "table_z": frame.TABLE_Z, "near_x": frame.NEAR_X, "origin_y": frame.ORIGIN_Y,
                               "side_cm": frame.SIDE_CM, "reach": reach})
        target = (STATIC / ("index.html" if path == "/" else path.lstrip("/"))).resolve()
        if STATIC.resolve() not in target.parents or not target.is_file():
            return self._json({"error": "not found"}, 404)
        self._send(200, target.read_bytes(), mimetypes.guess_type(target.name)[0] or "application/octet-stream")

    def do_POST(self) -> None:
        path = urlsplit(self.path).path
        try:
            length = int(self.headers.get("Content-Length") or 0)
            body = json.loads(self.rfile.read(length)) if 0 < length <= 4096 else {}
            pt = lambda k: (float(body[k][0]), float(body[k][1]))  # noqa: E731
            if path == "/api/pick_place":
                return self._json(self.station.pick_and_place(pt("src"), pt("dst")))
            if path == "/api/go_to":
                return self._json(self.station.go_to(pt("at")))
            if path == "/api/home":
                return self._json(self.station.go_home())
            if path == "/api/stop":
                return self._json(self.station.stop())
            if path == "/api/reset_blocks":
                return self._json(self.station.reset_blocks())
            return self._json({"error": "not found"}, 404)
        except BadRequest as exc:
            self._json({"error": str(exc)}, HTTPStatus.BAD_REQUEST)
        except Busy as exc:
            self._json({"error": str(exc)}, HTTPStatus.CONFLICT)
        except (KeyError, ValueError, TypeError, IndexError):
            self._json({"error": "malformed request"}, HTTPStatus.BAD_REQUEST)

    def log_message(self, format: str, *args) -> None:  # noqa: A002 - only errors are interesting
        if args and str(args[1]).startswith(("4", "5")):
            super().log_message(format, *args)


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=8810)
    ap.add_argument("--backend", choices=("mujoco", "gazebo"), default="mujoco",
                    help="mujoco (default): runs anywhere with `pip install mujoco`; gazebo: needs a running Gazebo")
    args = ap.parse_args()
    Handler.station = Station(args.backend)
    server = ThreadingHTTPServer((args.host, args.port), Handler)
    server.daemon_threads = True
    print(f"Gaze station UI ({args.backend}) on http://{args.host}:{args.port}")
    try:
        if args.backend == "mujoco":
            # OpenGL rendering must stay on the main thread on macOS, so the simulation owns it and HTTP runs beside it.
            threading.Thread(target=server.serve_forever, daemon=True).start()
            Handler.station.sim.run()
        else:
            server.serve_forever()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
