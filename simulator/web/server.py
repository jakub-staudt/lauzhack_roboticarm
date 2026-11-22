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
import math
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
from soarm_lab.kinematics import RobotKinematics
from soarm_lab.station import PREFIX

STATIC = Path(__file__).resolve().parent / "static"
REACH_MAP = Path(__file__).resolve().parent / "reach_map.json"
COLORS = ("red", "blue", "yellow")
START_BLOCK_CM = {"red": (15.0, 3.0), "blue": (4.0, 3.0), "yellow": (9.0, 6.0)}   # in the aligned rectangle: the near half, where a top-down grasp reaches
SNAP_M = 0.04         # a tap this close to a block (about 49 mm across) selects it
CLEARANCE_M = 0.06    # a destination this close (centre to centre) to another 49 mm block is refused


def png_bytes(rgb: np.ndarray) -> bytes:
    h, w, _ = rgb.shape
    raw = np.concatenate([np.zeros((h, 1), np.uint8), rgb.reshape(h, w * 3)], axis=1).tobytes()

    def chunk(kind: bytes, body: bytes) -> bytes:
        return struct.pack(">I", len(body)) + kind + body + struct.pack(">I", zlib.crc32(kind + body))

    return (b"\x89PNG\r\n\x1a\n" + chunk(b"IHDR", struct.pack(">IIBBBBB", w, h, 8, 2, 0, 0, 0))
            + chunk(b"IDAT", zlib.compress(raw, 1)) + chunk(b"IEND", b""))


class Station:
    """Gazebo + planner + the single running task."""

    def __init__(self, backend: str = "mujoco", park_base: tuple[float, float] | None = None, park_height_cm: float = 3.0) -> None:
        self.park_base, self.park_height_cm = park_base, park_height_cm   # the resting spot, in the ARM's frame [m]: out of the camera's way
        self.planner = Planner()
        if backend == "gazebo":  # optional: needs Gazebo (and on Linux the ROS Jazzy bindings); the default needs neither
            from simulator.gazebo.client import GazeboStation
            self.sim = GazeboStation()
        else:
            from simulator.mujoco_backend import MujocoStation
            self.sim = MujocoStation()
        self.sim.wait_ready(timeout=60)
        # a second kinematics object for reading the tip: the planner's one is busy while a plan is being built
        self._tip_kin = RobotKinematics(self.planner.robot, self.planner.model, prefix=PREFIX)
        self._tip_lock = threading.Lock()
        self.lock = threading.Lock()
        self.cancel = threading.Event()
        self.task = {"state": "idle", "label": "", "step": 0, "total": 0, "message": "Ready."}
        self.initial = {c: [float(v) for v in self.planner.model.body(f"{c}_block").pos] for c in COLORS}
        self.frame_png = b""
        self.frame_lock = threading.Lock()
        self.aligned: dict | None = None
        self.twin = None

    def start_twin(self, url: str, rate_hz: float = 8.0) -> None:
        """Mirror the REAL arm (read-only polling of the controller dashboard): the simulated arm follows its measured joints."""
        from simulator.twin import Twin, http_fetch

        def apply(measured: dict[str, float]) -> None:
            self.sim.command(self.planner.to_rad(measured))

        self.twin = Twin(http_fetch(url), apply, rate_hz, True)     # always together: there is no separate rehearsal mode while it runs
        self.cancel.set()                                           # a move still running would fight the mirror
        self.twin.start()

    def twin_status(self) -> dict:
        if self.twin is None:
            return {"available": False}
        out = {"available": True, **self.twin.status()}
        real = out.get("real", {}).get("joints")
        if real:
            sim = self.planner.to_app(self.sim.joints())
            out["max_diff_deg"] = round(max(abs(sim[j] - real[j]) for j in real if j != "gripper"), 1)
        return out

    def align_to_robot_points(self, path: str) -> dict:
        """Put the working rectangle where the REAL arm's recorded corners are: its (0, 0) at the real corner_00, stretched a little
        to the real size but still orthogonal. Moves the drawn guide lines, the three blocks (same block coordinates) and the frame."""
        from simulator import alignment
        base = self.planner.base
        fit = alignment.fit(alignment.load_points(path))
        old = {c: frame.world_to_block(*self.initial[c][:2], margin_cm=50.0) for c in COLORS}          # block coordinates before the move
        origin = (float(base[0]) + fit["origin"][0], float(base[1]) + fit["origin"][1])
        frame.configure_left(origin, fit["theta"], fit["su_m"], fit["sv_m"])
        model = getattr(self.sim, "model", None)
        if model is not None:                                                                         # MuJoCo: move the ink lines
            th, su, sv = fit["theta"], fit["su_m"], fit["sv_m"]
            ox, oy = origin
            e_u, e_v = (math.sin(th), -math.cos(th)), (math.cos(th), math.sin(th))
            quat = [math.cos(th / 2), 0.0, 0.0, math.sin(th / 2)]
            def edge(name, start, direction, length, along_u):
                g = model.geom(f"gaze/{name}")
                mid = (ox + start[0] + direction[0] * length / 2, oy + start[1] + direction[1] * length / 2)
                g.pos[:2] = mid
                g.size[:2] = (0.0008, length / 2 + 0.0008) if along_u else (length / 2 + 0.0008, 0.0008)
                g.quat[:] = quat
            edge("grid_near", (0, 0), e_u, su, True)                                                  # v = 0, runs along u
            edge("grid_far", (sv * e_v[0], sv * e_v[1]), e_u, su, True)                              # v = 1
            edge("grid_left", (0, 0), e_v, sv, False)                                                 # u = 0, runs along v
            edge("grid_center", (su * e_u[0], su * e_u[1]), e_v, sv, False)                           # u = 1
            model.geom("gaze/grid_right").size[:] = (0.0, 0.0, 0.0)                                   # the right block is not used
            for color in old:                                                                         # start the blocks where a top-down grasp reaches
                cx, cy = START_BLOCK_CM[color]
                x, y = frame.block_to_world("left", cx, cy)
                self.initial[color][:2] = [x, y]
            self.sim.reset_blocks(self.initial)
        self.aligned = {**fit, "origin_world": origin}
        return self.aligned

    # ------------------------------------------------------------------ state
    def blocks(self) -> list[dict]:
        out = []
        for color in COLORS:
            try:
                x, y, z = self.sim.block(color)
            except KeyError:
                x, y, z = self.initial[color]
            out.append({"color": color, "x": x, "y": y, "z": z, "label": frame.describe(x, y), "held": z > 0.7745 + 0.012})
        return out

    def tip_base(self) -> dict:
        """The gripper tip in the arm's base frame [m]: x forward, y to the left, z up (table about 0)."""
        with self._tip_lock:
            x, y, z = self._tip_kin.tcp(self.planner.to_app(self.sim.joints()))[:3, 3]
        return {"x": round(float(x), 4), "y": round(float(y), 4), "z": round(float(z), 4)}

    def state(self) -> dict:
        return {"task": dict(self.task), "blocks": self.blocks(), "tip_base": self.tip_base(), "twin": self.twin_status(),
                "park": None if self.park_base is None else {"x": self.park_base[0], "y": self.park_base[1], "height_cm": self.park_height_cm},
                "joints": {k: round(v, 3) for k, v in self.sim.joints().items()}}

    def png(self) -> bytes:
        return png_bytes(self.sim.image("overhead"))

    # ------------------------------------------------------------------ tasks
    def _set(self, **kw) -> None:
        self.task.update(kw)

    def _start(self, name: str, build) -> dict:
        if self.twin is not None and self.twin.enabled:
            raise Busy("The simulator is a digital twin: it mirrors the real arm, so it cannot move on its own. Move the real arm instead.")
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
        park = None if self.park_base is None else (float(self.planner.base[0]) + self.park_base[0], float(self.planner.base[1]) + self.park_base[1])
        return self._start(name, lambda: self.planner.pick_and_place(self.sim.joints(), src, dst, park=park, park_height=self.park_height_cm / 100.0))

    def go_to(self, px) -> dict:
        x, y = frame.pixel_to_world(*px)
        return self._start(f"Move tip to {frame.describe(x, y)}", lambda: self.planner.go_to(self.sim.joints(), (x, y)))

    def go_to_base(self, x: float, y: float, height_cm: float) -> dict:
        """Move the tip over a point given in the ARM'S OWN frame (metres: x forward, y left), `height_cm` above the table.
        Used by the camera page: its points come from the robot's recorded calibration, not from the drawn blocks."""
        if not (0.05 <= x <= 0.45 and -0.30 <= y <= 0.30) or not 0.5 <= height_cm <= 10:
            raise BadRequest("That point is outside the arm's working area.")
        world = (float(self.planner.base[0]) + x, float(self.planner.base[1]) + y)
        return self._start(f"Move tip over base ({x * 100:.1f}, {y * 100:.1f}) cm",
                           lambda: self.planner.go_to(self.sim.joints(), world, height=height_cm / 100.0))

    def park(self) -> dict:
        """Go to the resting spot (3 cm above the bottom-left corner), out of the camera's way."""
        if self.park_base is None:
            raise BadRequest("No park spot is configured (start the server with --park-points).")
        return self.go_to_base(self.park_base[0], self.park_base[1], self.park_height_cm)

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
        if path == "/api/twin":
            return self._json(self.station.twin_status())
        if path == "/api/config":
            reach = json.loads(REACH_MAP.read_text()) if REACH_MAP.exists() and not self.station.aligned else None   # the map is for the ideal square
            return self._json({"cam_pos": frame.CAM_POS, "cam_focal": frame.CAM_FOCAL, "cam_size": frame.CAM_SIZE,
                               "table_z": frame.TABLE_Z, "near_x": frame.NEAR_X, "origin_y": frame.ORIGIN_Y,
                               "side_cm": frame.SIDE_CM, "reach": reach, "left": frame.LEFT, "aligned": self.station.aligned is not None})
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
            if path == "/api/go_to_base":
                return self._json(self.station.go_to_base(float(body["x"]), float(body["y"]), float(body.get("height_cm", 3.0))))
            if path == "/api/park":
                return self._json(self.station.park())
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
    ap.add_argument("--park-points", default=str(Path(__file__).resolve().parents[2] / "robotic_arm" / "calibration_poses.json"),
                    help="calibration points recorded with the real arm; the arm parks 3 cm above its corner_00 (bottom-left) after each move")
    ap.add_argument("--no-align", action="store_true", help="keep the ideal 20 x 20 cm squares instead of fitting the rectangle to the real arm's corners")
    ap.add_argument("--twin", metavar="URL", help="digital twin: make the simulated arm follow the REAL arm by reading the controller dashboard's "
                    "/api/state (read-only), e.g. http://100.85.56.14:8780")
    ap.add_argument("--park-height-cm", type=float, default=3.0)
    ap.add_argument("--backend", choices=("mujoco", "gazebo"), default="mujoco",
                    help="mujoco (default): runs anywhere with `pip install mujoco`; gazebo: needs a running Gazebo")
    args = ap.parse_args()
    park_base = None
    try:
        for point in json.loads(Path(args.park_points).read_text()):
            if point.get("name") == "corner_00":
                park_base = (float(point["xyz_m"][0]), float(point["xyz_m"][1]))
    except (OSError, ValueError, KeyError):
        pass
    print(f"park spot: {'none (home pose is used)' if park_base is None else f'x {park_base[0] * 100:.1f}, y {park_base[1] * 100:.1f} cm in the arm frame, {args.park_height_cm:g} cm up'}")
    Handler.station = Station(args.backend, park_base, args.park_height_cm)
    if not args.no_align and args.backend == "mujoco":
        try:
            fit = Handler.station.align_to_robot_points(args.park_points)
            print(f"rectangle aligned to the real arm's corners: (0, 0) at world ({fit['origin_world'][0] * 100:.1f}, {fit['origin_world'][1] * 100:.1f}) cm, "
                  f"{fit['su_m'] * 100:.1f} x {fit['sv_m'] * 100:.1f} cm, rotated {math.degrees(fit['theta']):.1f} deg, fit error {fit['residual_mm']:.1f} mm")
        except (OSError, ValueError, KeyError) as exc:
            print(f"rectangle not aligned ({exc}); using the ideal squares")
    if args.twin:
        Handler.station.start_twin(args.twin)
        print(f"digital twin: the simulated arm mirrors the real arm from {args.twin} (read-only; no separate mode)")
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
