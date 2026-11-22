"""Web page that shows what the detector sees in the overhead camera.

    .venv/bin/python -m lauzkach_eye_robot_control.server            # live camera of the RAMION server, http://127.0.0.1:8820
    .venv/bin/python -m lauzkach_eye_robot_control.server --image tests/fixtures/overhead_1.jpg   # a picture instead

The server only reads: it GETs pictures from the camera and serves the page. It never contacts the arm controller.
The camera address is fixed at start-up (never taken from a web request).
"""

from __future__ import annotations

import argparse
import json
import math
import mimetypes
import threading
import time
from collections import OrderedDict
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import urlsplit

import cv2
import numpy as np

from lauzkach_eye_robot_control.camera import FileSource, FrameSource, MjpegSource, decode
from lauzkach_eye_robot_control.detect import detect, rectify
from lauzkach_eye_robot_control.closedloop import SimMover, SimTruthObserver
from lauzkach_eye_robot_control.drive import FACES, SimClient
from lauzkach_eye_robot_control.grid import WORLD_CORNERS
from lauzkach_eye_robot_control.robot_map import CalibrationError, RobotMap
from lauzkach_eye_robot_control.service import ApiError, DriveService

STATIC = Path(__file__).resolve().parent / "static"
DEFAULT_CAMERA = "http://100.85.56.14:8780/cameras/overhead.mjpg?fps=2&profile=preview"
DEFAULT_ROBOT_POINTS = Path(__file__).resolve().parents[2] / "robotic_arm" / "calibration_poses.json"   # recorded with the real arm
DEFAULT_SIMULATOR = "http://127.0.0.1:8810"
KEEP_FRAMES = 12
SMOOTHING = 0.3          # weight of the newest frame in the smoothed rectangle (before the rectangle is locked)
JUMP_RESET_PX = 30       # a corner moving farther than this between frames restarts the smoothing
CALIBRATION_S = 5.0      # the table and the camera do not move: measure the rectangle for this long, then lock it
MIN_FRAMES = 4           # fewest usable frames needed to lock
DRIFT_PX = 6.0           # live tags / lines this far from their locked position count as "something moved"
DRIFT_FRAMES = 4         # ... for this many frames in a row
OUTLIER_PX = 10.0        # while calibrating, a frame whose rectangle is this far from the median is dropped
LEFT_SEEN_SHARE = 0.3    # the real left line beats the estimate (1-2 cm off on the real table): use it when seen in this share of frames


class Analyzer:
    """Runs the detector on every new camera frame and keeps the recent frames so the page can draw on the right one."""

    def __init__(self, source: FrameSource, calibration_s: float = CALIBRATION_S, min_frames: int = MIN_FRAMES) -> None:
        self.source = source
        self.calibration_s, self.min_frames = calibration_s, min_frames
        self.cal: dict = {"state": "waiting"}
        self._stopping = threading.Event()
        self._thread: threading.Thread | None = None
        self.lock = threading.Lock()
        self.frames: OrderedDict[int, tuple[bytes, bytes | None]] = OrderedDict()
        self.result: dict | None = None
        self._smooth: np.ndarray | None = None
        self._misses = 0
        self._last = 0

    def start(self) -> None:
        self.source.start()
        self._thread = threading.Thread(target=self._run, daemon=True, name="analyzer")
        self._thread.start()

    def stop(self) -> None:
        self._stopping.set()
        if self._thread is not None:
            self._thread.join(timeout=3)
        self.source.stop()

    def _run(self) -> None:
        while not self._stopping.is_set():
            try:
                self._step()
            except Exception as exc:  # noqa: BLE001 - one bad frame must not stop the page
                print(f"analyzer error: {type(exc).__name__}: {exc}", flush=True)
                time.sleep(0.5)

    def _step(self) -> None:
        frame = self.source.latest()
        if frame is None or frame[0] == self._last:
            time.sleep(0.05)
            return
        seq, jpeg, received = frame
        self._last = seq
        image = decode(jpeg)
        if image is None:
            return
        t0 = time.perf_counter()
        try:
            found = detect(image)
        except Exception as exc:  # noqa: BLE001 - show the failure on the page instead of stopping
            found = {"width": image.shape[1], "height": image.shape[0], "tags": [], "lines": [], "chosen": {}, "polygon": None,
                     "method": None, "issues": [f"detector error: {type(exc).__name__}: {exc}"], "ok": False,
                     "aspect": None, "side_px": None, "left_estimate": None, "left_detected": None}
        elapsed = (time.perf_counter() - t0) * 1000
        with self.lock:
            self._update_calibration(found, received)
            locked = self.cal.get("polygon") if self.cal["state"] == "locked" else None
        smooth = self._smooth_polygon(found["polygon"])
        shown = np.array(locked, float) if locked is not None else (smooth if smooth is not None else
                                                                   (None if found["polygon"] is None else np.array(found["polygon"], float)))
        crop = None
        if shown is not None:
            crop = cv2.imencode(".jpg", rectify(image, shown, 360), [cv2.IMWRITE_JPEG_QUALITY, 85])[1].tobytes()
        with self.lock:
            self.frames[seq] = (jpeg, crop)
            while len(self.frames) > KEEP_FRAMES:
                self.frames.popitem(last=False)
            self.result = {"seq": seq, "received": received, "detect_ms": round(elapsed, 1), "detection": found,
                           "polygon_shown": None if shown is None else [[round(float(x), 1), round(float(y), 1)] for x, y in shown],
                           "locked": locked is not None, "has_crop": crop is not None}

    # ---------------------------------------------------------------- calibration (called with self.lock held)
    def recalibrate(self) -> None:
        with self.lock:
            self.cal = {"state": "waiting"}
        self._smooth = None

    def _update_calibration(self, found: dict, when: float) -> None:
        cal = self.cal
        if cal["state"] == "waiting":
            cal.update(state="countdown", t0=when, frames=[])
        if cal["state"] == "countdown":
            if found["polygon"]:
                cal["frames"].append({"polygon": found["polygon"], "method": found["method"] or "",
                                      "tags": {t["id"]: t["center"] for t in found["tags"]}})
            if when - cal["t0"] >= self.calibration_s:
                self._lock_rectangle()
        elif cal["state"] == "locked":
            self._check_drift(found)

    def _lock_rectangle(self) -> None:
        cal = self.cal
        frames = cal["frames"]
        if len(frames) < self.min_frames:
            cal.update(state="failed", reason=f"only {len(frames)} usable frame(s) in {self.calibration_s:.0f} s; "
                                              "check the camera view and that the tags and lines are visible")
            return
        seen = [f for f in frames if "left line detected" in f["method"]]
        used_seen = len(seen) >= max(self.min_frames, LEFT_SEEN_SHARE * len(frames))
        use = seen if used_seen else frames
        stack = np.array([f["polygon"] for f in use], float)
        median = np.median(stack, axis=0)
        close = np.abs(stack - median).max(axis=(1, 2)) <= OUTLIER_PX      # drop the odd frame (a glitch, a hand over the table)
        dropped = int((~close).sum())
        if close.sum() >= self.min_frames:
            use = [f for f, keep in zip(use, close, strict=True) if keep]
            stack = stack[close]
        tags: dict[int, list] = {}
        for f in use:
            for tag_id, centre in f["tags"].items():
                tags.setdefault(tag_id, []).append(centre)
        polygon = np.median(stack, axis=0)
        px_to_cm = cv2.getPerspectiveTransform(polygon.astype(np.float32), WORLD_CORNERS).tolist()   # picture pixel -> table cm (0..20)
        cal.update(state="locked", polygon=polygon.tolist(), px_to_cm=px_to_cm, spread_px=float(stack.std(axis=0).max()),
                   method="left line seen and used" if used_seen else "left edge estimated (left line hidden)",
                   n_frames=len(use), of=len(frames), dropped=dropped, locked_at=time.time(), drift_px=0.0, drift_count=0, drift_warn=False,
                   tags={i: np.median(np.array(c), axis=0).tolist() for i, c in tags.items()})
        cal.pop("frames", None)

    def _check_drift(self, found: dict) -> None:
        """Compare what is visible now with the locked rectangle: tag centres, and the corners made from visible lines."""
        cal = self.cal
        moves = [math.dist(t["center"], cal["tags"][t["id"]]) for t in found["tags"] if t["id"] in cal["tags"]]
        if found["polygon"]:
            moves += [math.dist(found["polygon"][i], cal["polygon"][i]) for i in (1, 2)]  # TR and BR come from visible lines
        if not moves:
            return
        cal["drift_px"] = float(np.median(moves))   # the middle value: one bad frame or one odd tag cannot set it off
        cal["drift_count"] = cal["drift_count"] + 1 if cal["drift_px"] > DRIFT_PX else 0
        cal["drift_warn"] = cal["drift_count"] >= DRIFT_FRAMES

    def _smooth_polygon(self, polygon: list[list[float]] | None) -> np.ndarray | None:
        if polygon is None:
            self._misses += 1
            if self._misses > 5:
                self._smooth = None
            return self._smooth
        self._misses = 0
        new = np.array(polygon, float)
        if self._smooth is None or np.abs(new - self._smooth).max() > JUMP_RESET_PX:
            self._smooth = new
        else:
            self._smooth = (1 - SMOOTHING) * self._smooth + SMOOTHING * new
        return self._smooth

    def locked_matrix(self) -> list[list[float]] | None:
        """The picture-pixel -> table-cm matrix of the locked rectangle, or None until it is locked."""
        with self.lock:
            return self.cal.get("px_to_cm") if self.cal["state"] == "locked" else None

    def snapshot(self) -> dict:
        with self.lock:
            result = dict(self.result) if self.result else None
            cal = dict(self.cal)
        now = time.time()
        calibration = {"state": cal["state"], "seconds": self.calibration_s}
        if cal["state"] == "countdown":
            calibration.update(remaining=round(max(0.0, self.calibration_s - (now - cal["t0"])), 1), frames=len(cal["frames"]))
        elif cal["state"] == "failed":
            calibration["reason"] = cal["reason"]
        elif cal["state"] == "locked":
            calibration.update({k: cal[k] for k in ("polygon", "method", "spread_px", "n_frames", "of", "locked_at", "drift_px", "drift_warn", "px_to_cm")})
            calibration["spread_px"] = round(calibration["spread_px"], 1)
            calibration["drift_px"] = round(calibration["drift_px"], 1)
        out = {"source": {"status": self.source.status, "error": self.source.error}, "now": now, "calibration": calibration}
        if result:
            out.update(result)
        if cal["state"] == "locked":   # a status request can land between locking and the next analysed frame
            out["locked"], out["polygon_shown"] = True, [[round(float(x), 1), round(float(y), 1)] for x, y in cal["polygon"]]
        return out

    def frame(self, seq: int, crop: bool) -> bytes | None:
        with self.lock:
            entry = self.frames.get(seq)
        return None if entry is None else entry[1 if crop else 0]


class Handler(BaseHTTPRequestHandler):
    analyzer: Analyzer
    service: DriveService | None = None
    protocol_version = "HTTP/1.1"

    def _send(self, status: int, body: bytes, ctype: str) -> None:
        self.send_response(status)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        try:
            self.wfile.write(body)
        except (BrokenPipeError, ConnectionResetError):
            self.close_connection = True

    def do_GET(self) -> None:
        path = urlsplit(self.path).path
        if self.service is not None and path.startswith("/api/"):
            self.service.touch()                      # the page is still there (the dead-man timer)
        if path == "/api/drive":
            if self.service is None:
                return self._send(200, json.dumps({"enabled": False}).encode(), "application/json")
            return self._send(200, json.dumps({"enabled": True, **self.service.overview()}).encode(), "application/json")
        if path == "/api/detect":
            return self._send(200, json.dumps(self.analyzer.snapshot()).encode(), "application/json")
        for prefix, crop in (("/frame/", False), ("/crop/", True)):
            if path.startswith(prefix) and path.endswith(".jpg"):
                try:
                    body = self.analyzer.frame(int(path[len(prefix):-4]), crop)
                except ValueError:
                    body = None
                if body is None:
                    return self._send(404, b"gone", "text/plain")
                return self._send(200, body, "image/jpeg")
        target = (STATIC / ("index.html" if path == "/" else path.lstrip("/"))).resolve()
        if STATIC.resolve() not in target.parents or not target.is_file():
            return self._send(404, b"not found", "text/plain")
        self._send(200, target.read_bytes(), mimetypes.guess_type(target.name)[0] or "application/octet-stream")

    def do_POST(self) -> None:
        path = urlsplit(self.path).path
        origin = self.headers.get("Origin")
        if origin is not None and urlsplit(origin).netloc != self.headers.get("Host"):
            return self._send(403, b"cross-origin request refused", "text/plain")
        if self.service is not None:
            self.service.touch()
        try:
            length = int(self.headers.get("Content-Length") or 0)
            body = json.loads(self.rfile.read(length)) if 0 < length <= 4096 else {}
            if not isinstance(body, dict):
                raise ValueError("expected a JSON object")
            if path == "/api/recalibrate":
                self.analyzer.recalibrate()          # restarts the 5 s measurement; only this page's state changes
                return self._send(200, json.dumps(self.analyzer.snapshot()).encode(), "application/json")
            if self.service is None or not path.startswith("/api/drive/"):
                return self._send(404, b"not found", "text/plain")
            envelope = {k: body.get(k) for k in ("cid", "sent_ms", "gen", "max_age_ms")}
            if path == "/api/drive/go":
                result = self.service.go_click(envelope, float(body["px"]), float(body["py"]), float(body.get("height_cm", 3.0)),
                                               bool(body.get("closed_loop", False)), str(body.get("target", "sim")), bool(body.get("confirm_real", False)))
            elif path == "/api/drive/corner":
                result = self.service.go_corner(envelope, int(body["index"]), float(body.get("height_cm", 3.0)), bool(body.get("closed_loop", False)),
                                                str(body.get("target", "sim")), bool(body.get("confirm_real", False)))
            elif path == "/api/drive/stop":
                result = self.service.stop()
            elif path == "/api/drive/faces":
                result = self.service.set_faces(str(body.get("faces")))
            else:
                return self._send(404, b"not found", "text/plain")
            self._send(200, json.dumps(result).encode(), "application/json")
        except ApiError as exc:
            self._send(exc.status, json.dumps({"error": str(exc), "code": exc.code}).encode(), "application/json")
        except (KeyError, ValueError, TypeError):
            self._send(400, json.dumps({"error": "malformed request", "code": "bad_request"}).encode(), "application/json")

    def log_message(self, format: str, *args) -> None:  # noqa: A002 - quiet unless something fails
        if args and str(args[1]).startswith(("4", "5")):
            super().log_message(format, *args)


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--camera", default=DEFAULT_CAMERA, help="MJPEG stream of the overhead camera (read-only GET)")
    ap.add_argument("--image", help="use this picture instead of the camera")
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=8820)
    ap.add_argument("--calibration-seconds", type=float, default=CALIBRATION_S,
                    help="measure the rectangle for this long after start-up, then lock it (default 5)")
    ap.add_argument("--robot-points", default=str(DEFAULT_ROBOT_POINTS),
                    help="calibration points recorded with the real arm (calibration_poses.json); enables the drive panel")
    ap.add_argument("--simulator", default=DEFAULT_SIMULATOR, help="the simulator's web server (must be http://127.0.0.1:... unless you say otherwise)")
    ap.add_argument("--robot-faces", choices=FACES, default="up", help="which way the robot faces in the camera picture")
    ap.add_argument("--closed-loop-demo", metavar="DX,DY", help="demo on the simulator: pretend the board is DX,DY mm off from the arm's coordinates (e.g. 12,-8) "
                    "and let the closed loop correct it. A stand-in for the camera, not the real thing.")
    ap.add_argument("--real-arm", action="store_true",
                    help="ENABLE moves of the REAL arm (the five buttons only; hover 3 cm or more, slow, each move confirmed in the page). Off by default.")
    ap.add_argument("--dashboard", default="http://100.85.56.14:8780", help="the RAMION dashboard the real arm is reached through")
    ap.add_argument("--deadman-seconds", type=float, default=3.0, help="hold the arm if the page is silent this long during a move")
    args = ap.parse_args()
    source: FrameSource = FileSource(args.image) if args.image else MjpegSource(args.camera)
    Handler.analyzer = Analyzer(source, calibration_s=args.calibration_seconds)
    try:
        robot_map = RobotMap.load(args.robot_points)
    except (OSError, CalibrationError, ValueError, KeyError) as exc:
        robot_map = None
        print(f"drive panel limited: no robot calibration ({exc})")
    sim_client = SimClient(args.simulator)
    closed_loop, demo_error = None, (0.0, 0.0)
    if args.closed_loop_demo:
        demo_error = tuple(float(v) / 1000.0 for v in args.closed_loop_demo.split(","))
        closed_loop = (SimMover(sim_client), SimTruthObserver(sim_client, demo_error, noise_mm=0.5))
        print(f"closed-loop DEMO: the simulated board is {args.closed_loop_demo} mm off the arm's coordinates")
    real = None
    if args.real_arm:
        if robot_map is None:
            raise SystemExit("--real-arm needs the robot calibration points (--robot-points)")
        from robotic_arm.clickmove import pooled_http          # needs the robotic_arm environment (the arm's kinematics)
        from soarm_lab.kinematics import RobotKinematics
        from soarm_lab.robot import load_robot

        from lauzkach_eye_robot_control.realarm import RealArmClient
        real = RealArmClient(pooled_http(args.dashboard), RobotKinematics(load_robot()), [(p["x"], p["y"]) for p in robot_map.points], robot_map.z_table)
        print("*** REAL ARM ENABLED: the five buttons can move the physical arm (hover >= 3 cm, slow, each move confirmed). Keep Stop (Esc) ready. ***")
    Handler.service = DriveService(Handler.analyzer.locked_matrix, robot_map, sim_client, faces=args.robot_faces,
                                   deadman_s=args.deadman_seconds, closed_loop=closed_loop, sim_calibration_error=demo_error, real=real)
    Handler.service.start()
    Handler.analyzer.start()
    server = ThreadingHTTPServer((args.host, args.port), Handler)
    server.daemon_threads = True
    print(f"Rectangle detector on http://{args.host}:{args.port}  (source: {args.image or args.camera})")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        Handler.service.stop_background()
        Handler.analyzer.stop()


if __name__ == "__main__":
    main()
