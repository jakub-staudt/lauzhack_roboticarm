"""Gaze service: the eye tracker from gaze_pick_place.py (EyeGestures v3 on the laptop webcam) behind a small HTTP API,
so the Kotlin app (kmp_app) can run the full-screen gaze scene.

    .venv/bin/python -m gaze_service.server                 # http://127.0.0.1:8820, webcam 0
    .venv/bin/python -m gaze_service.server --camera 1
    .venv/bin/python -m gaze_service.server --mock          # no camera: gaze comes from POST /api/mock (testing)

The app shows the calibration dots full screen; this service decides where they go and when to move on, exactly like
the pygame prototype (25-dot snake, then 5 crosses for the drift correction). Coordinates in the API are normalised
to the screen: (0, 0) top-left, (1, 1) bottom-right.

    GET  /api/gaze        current gaze + phase (see Tracker.snapshot)
    POST /api/calibrate   {"screen": [w, h]}  full calibration, then gaze correction
    POST /api/correct     {"screen": [w, h]}  gaze correction only (5 crosses)
    POST /api/load        {"screen": [w, h]}  load the saved calibration (eg_calibration.pkl)
    POST /api/cancel      stop a running calibration
    POST /api/mock        {"x": .., "y": ..} or {"face": false}   (--mock only)

On macOS the terminal that starts this needs camera permission (System Settings > Privacy & Security > Camera).
"""

from __future__ import annotations

import argparse
import json
import logging
import threading
import time
from collections import deque
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import urlsplit

import numpy as np

log = logging.getLogger("gaze")

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
CALIB_FILE = HERE / "eg_calibration.pkl"          # your personal calibration (not committed)
CORR_FILE = HERE / "gaze_correction.json"
CTX = "reacha"

# Same values as gaze_pick_place.py
CAL_GRID = [0.05, 0.275, 0.5, 0.725, 0.95]
CAL_SETTLE = 0.8
CAL_COLLECT = 1.2
CORR_SPOTS = [(0.5, 0.5), (0.2, 0.25), (0.8, 0.25), (0.8, 0.75), (0.2, 0.75)]
CORR_TIME = 1.4
CORR_SKIP = 0.5
STABLE_WINDOW = 0.3       # gaze_scene_v2.py
MOVE_SHIFT_PX = 120       # px on a ~1440 px screen; scaled with the screen width below
FACE_TIMEOUT = 1.0


def calibration_map() -> np.ndarray:
    """Row by row, alternating direction (snake), so each eye movement is short."""
    pts = []
    for r, y in enumerate(CAL_GRID):
        row = CAL_GRID if r % 2 == 0 else CAL_GRID[::-1]
        pts += [(x, y) for x in row]
    return np.array(pts)


class Camera:
    """Webcam in a background thread; read() returns the newest frame or (False, None) - it never blocks for long
    (EyeGestures' VideoCapture blocks forever when macOS withholds the camera)."""

    def __init__(self, index: int) -> None:
        self.index = index
        self.cap = None
        self.frame = None
        self.stamp = 0.0
        self.seen = 0.0
        self.size = (0, 0)
        self.cond = threading.Condition()
        threading.Thread(target=self._reader, daemon=True).start()

    def _open(self):
        import platform

        import cv2
        system = platform.system()
        # DirectShow opens webcams quickly on Windows (as EyeGestures does); AVFoundation is the macOS camera API
        # (asking for it explicitly also makes OpenCV log macOS's permission answer).
        backend = {"Windows": cv2.CAP_DSHOW, "Darwin": cv2.CAP_AVFOUNDATION}.get(system, cv2.CAP_ANY)
        cap = cv2.VideoCapture(self.index, backend)
        return cap if cap.isOpened() else None

    def _reader(self) -> None:
        failures = 0
        while True:
            if self.cap is None:
                # Not open yet (e.g. macOS is still asking for permission): try again, the answer may come later.
                self.cap = self._open()
                if self.cap is None:
                    time.sleep(2.0)
                    continue
            ok, frame = self.cap.read()
            if not ok or frame is None:
                failures += 1
                if failures > 40:              # ~2 s without images: reopen
                    self.cap.release()
                    self.cap, failures = None, 0
                time.sleep(0.05)
                continue
            failures = 0
            with self.cond:
                self.frame, self.stamp = frame, time.time()
                self.size = (frame.shape[1], frame.shape[0])
                self.cond.notify_all()

    def wait_first(self, timeout: float) -> bool:
        with self.cond:
            return self.cond.wait_for(lambda: self.frame is not None, timeout)

    def read(self):
        with self.cond:
            self.cond.wait_for(lambda: self.stamp > self.seen, 0.2)
            if self.frame is None or self.stamp <= self.seen:
                return False, None
            self.seen = self.stamp
            return True, self.frame


class Tracker(threading.Thread):
    """Reads the webcam and runs EyeGestures in one thread; HTTP handlers only read snapshots and post requests."""

    def __init__(self, camera: int, mock: bool) -> None:
        super().__init__(daemon=True)
        self.camera_index = camera
        self.mock = mock
        self.lock = threading.Lock()
        self.W, self.H = 1440.0, 900.0
        self.corr = np.array([1.0, 0.0, 1.0, 0.0])        # x' = a*x + b, y' = c*y + d (screen px)
        self.calibrated = False
        self.camera_ok = mock
        self.error = ""
        self.phase = "idle"                                 # idle | calibrating | correcting | tracking
        self.point: tuple[float, float] | None = None       # corrected, px
        self.last_face = 0.0
        self.moving = False
        self.recent: deque = deque()
        self.target: dict | None = None
        self.requests: deque = deque()
        self.mock_point: tuple[float, float] | None = (0.5, 0.5)
        self.mock_face = True
        self.gestures = None
        self.cap = None

    # ---------------------------------------------------------------- API side
    def request(self, kind: str, screen=None) -> None:
        with self.lock:
            self.requests.append((kind, screen))

    def set_mock(self, body: dict) -> None:
        with self.lock:
            if "face" in body:
                self.mock_face = bool(body["face"])
            if "x" in body and "y" in body:
                self.mock_point = (float(body["x"]), float(body["y"]))
                self.mock_face = True

    def snapshot(self) -> dict:
        with self.lock:
            now = time.time()
            p = self.point
            return {
                "phase": self.phase,
                "camera": self.camera_ok,
                "error": self.error,
                "mock": self.mock,
                "calibrated": self.calibrated,
                "face": now - self.last_face < FACE_TIMEOUT,
                "x": None if p is None else p[0] / self.W,
                "y": None if p is None else p[1] / self.H,
                "moving": self.moving,
                "target": self.target,
                "savedCalibration": CALIB_FILE.exists(),
            }

    # ---------------------------------------------------------------- tracker thread
    def run(self) -> None:
        if not self.mock:
            try:
                self.open_camera()
            except Exception as e:  # keep serving /api/gaze so the app can say what is wrong
                log.exception("camera / EyeGestures failed")
                with self.lock:
                    self.error = f"Eye tracker failed to start: {e}"
                return
        while True:
            try:
                self.handle_requests()
                if self.phase == "calibrating":
                    self.step_calibration()
                elif self.phase == "correcting":
                    self.step_correction()
                else:
                    self.step_tracking()
            except Exception as e:
                log.exception("tracker step failed")
                with self.lock:
                    self.error = str(e)
                time.sleep(0.2)

    def open_camera(self) -> None:
        from eyeGestures import EyeGestures_v3

        self.gestures = EyeGestures_v3()
        self.gestures.uploadCalibrationMap(calibration_map(), context=CTX)
        self.gestures.setFixation(1.0)
        self.cap = Camera(self.camera_index)
        # Keep waiting: the macOS camera prompt (or a permission switched on in Settings) can come later.
        while not self.cap.wait_first(6.0):
            with self.lock:
                self.error = (f"No images from camera {self.camera_index} yet. On macOS allow the camera for the app that started "
                              "this (Gaze Station or Terminal: System Settings > Privacy & Security > Camera); on Windows allow desktop apps to use the camera "
                              "(Settings > Privacy > Camera). Or try GAZE_CAMERA=1. Still waiting…")
            log.warning("no images from camera %s yet", self.camera_index)
        with self.lock:
            self.error = ""
            self.camera_ok = True
        log.info("camera %s open, %dx%d", self.camera_index, *self.cap.size)

    def event(self, calibrate: bool):
        """One camera frame -> (raw gaze px or None, blink). None also when there is no face."""
        if self.mock:
            time.sleep(1 / 30)
            with self.lock:
                if not self.mock_face or self.mock_point is None:
                    return None
                x, y = self.mock_point
            # undo the correction so the corrected output lands exactly on the mock point
            a, b, c, d = self.corr
            return ((x * self.W - b) / a, (y * self.H - d) / c)
        import cv2
        ret, frame = self.cap.read()
        if not ret or frame is None:
            time.sleep(0.01)
            return None
        frame = np.flip(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB), axis=1)
        try:
            ev, _ = self.gestures.step(frame, calibrate, int(self.W), int(self.H), context=CTX)
        except (TypeError, IndexError, ValueError):
            return None            # no face in this frame
        if ev is None or bool(getattr(ev, "blink", False)):
            return None
        return float(ev.point[0]), float(ev.point[1])

    def handle_requests(self) -> None:
        while True:
            with self.lock:
                if not self.requests:
                    return
                kind, screen = self.requests.popleft()
            if screen:
                self.W, self.H = float(screen[0]), float(screen[1])
            if kind == "calibrate":
                self.start_calibration()
            elif kind == "correct":
                self.start_correction()
            elif kind == "load":
                self.load_calibration()
            elif kind == "cancel":
                with self.lock:
                    self.phase = "tracking" if self.calibrated else "idle"
                    self.target = None

    def load_calibration(self) -> None:
        if not self.mock:
            self.gestures.loadModel(CALIB_FILE.read_bytes(), context=CTX)
        if CORR_FILE.exists():
            self.corr = np.array(json.loads(CORR_FILE.read_text())["corr"], dtype=float)
        with self.lock:
            self.calibrated = True
            self.phase = "tracking"
        log.info("loaded %s", CALIB_FILE.name)

    # calibration: 25 dots, settle then collect, the timer pauses while no face is seen
    def start_calibration(self) -> None:
        if not self.mock:
            self.gestures.reset(context=CTX)
            self.gestures.uploadCalibrationMap(calibration_map(), context=CTX)
            clb = self.gestures.clb[CTX]
            clb.acceptance_radius = -1
            clb.calibration_radius = 10 ** 6
        self.cal_points = calibration_map()
        self.cal_i = 0
        self.cal_elapsed = 0.0
        self.cal_prev = time.time()
        self.corr = np.array([1.0, 0.0, 1.0, 0.0])
        with self.lock:
            self.calibrated = False
            self.point = None
            self.phase = "calibrating"
        self.update_cal_target()

    def cal_xy(self) -> tuple[float, float]:
        if self.mock:
            x, y = self.cal_points[self.cal_i]
            return float(x), float(y)
        tx, ty = self.gestures.clb[CTX].getCurrentPoint(self.W, self.H)
        return float(tx) / self.W, float(ty) / self.H

    def update_cal_target(self) -> None:
        x, y = self.cal_xy()
        total = CAL_SETTLE + CAL_COLLECT
        with self.lock:
            self.target = {"kind": "dot", "x": x, "y": y, "index": self.cal_i, "total": len(self.cal_points),
                           "progress": min(self.cal_elapsed / total, 1.0), "recording": self.cal_elapsed > CAL_SETTLE}

    def step_calibration(self) -> None:
        recording = self.cal_elapsed > CAL_SETTLE
        raw = self.event(recording)
        now = time.time()
        if raw is not None:
            self.cal_elapsed += now - self.cal_prev
            with self.lock:
                self.last_face = now
        self.cal_prev = now
        if self.cal_elapsed >= CAL_SETTLE + CAL_COLLECT:
            if not self.mock:
                self.gestures.clb[CTX].movePoint()
            self.cal_i += 1
            self.cal_elapsed = 0.0
            if self.cal_i >= len(self.cal_points):
                if not self.mock:
                    try:
                        CALIB_FILE.write_bytes(self.gestures.saveModel(context=CTX))
                    except Exception as e:
                        log.error("could not save calibration: %s", e)
                with self.lock:
                    self.calibrated = True
                self.start_correction()
                return
        self.update_cal_target()

    # gaze correction: look at 5 crosses, fit x' = a*x + b and y' = c*y + d (fixes shift and "squeeze")
    def start_correction(self) -> None:
        self.corr_i = 0
        self.corr_t0 = time.time()
        self.corr_samples: list = []
        self.corr_raw: list = []
        self.corr_true: list = []
        with self.lock:
            self.phase = "correcting"
        self.update_corr_target()

    def update_corr_target(self) -> None:
        x, y = CORR_SPOTS[self.corr_i]
        el = time.time() - self.corr_t0
        with self.lock:
            self.target = {"kind": "cross", "x": x, "y": y, "index": self.corr_i, "total": len(CORR_SPOTS),
                           "progress": min(el / CORR_TIME, 1.0), "recording": el > CORR_SKIP}

    def step_correction(self) -> None:
        raw = self.event(False)
        now = time.time()
        if raw is not None:
            with self.lock:
                self.last_face = now
            if now - self.corr_t0 > CORR_SKIP:
                self.corr_samples.append(raw)
        if now - self.corr_t0 >= CORR_TIME:
            if len(self.corr_samples) >= 5:
                fx, fy = CORR_SPOTS[self.corr_i]
                self.corr_raw.append(np.median(np.array(self.corr_samples), axis=0))
                self.corr_true.append((fx * self.W, fy * self.H))
            self.corr_i += 1
            self.corr_samples = []
            self.corr_t0 = now
            if self.corr_i >= len(CORR_SPOTS):
                self.finish_correction()
                return
        self.update_corr_target()

    def finish_correction(self) -> None:
        if len(self.corr_raw) >= 3 and not self.mock:
            raw, true = np.array(self.corr_raw), np.array(self.corr_true)
            new = []
            for k in (0, 1):
                if np.ptp(raw[:, k]) > 30:
                    a, _ = np.polyfit(raw[:, k], true[:, k], 1)
                    a = float(np.clip(a, 0.6, 2.5))
                    b = float(np.mean(true[:, k] - a * raw[:, k]))
                else:
                    a, b = 1.0, float(np.mean(true[:, k] - raw[:, k]))
                new += [a, b]
            self.corr = np.array(new)
            log.info("gaze correction: x' = %.2f*x %+.0f,  y' = %.2f*y %+.0f", *self.corr)
            try:
                CORR_FILE.write_text(json.dumps({"corr": self.corr.tolist(), "screen": [self.W, self.H]}))
            except OSError as e:
                log.error("could not save correction: %s", e)
        elif not self.mock:
            log.info("gaze correction skipped: not enough tracking data")
        with self.lock:
            self.phase = "tracking"
            self.target = None

    def step_tracking(self) -> None:
        if not self.calibrated:
            # Not calibrated yet: still read frames so the app can say whether a face is visible.
            raw = None
            if self.mock:
                raw = self.event(False)
            else:
                import cv2
                ret, frame = self.cap.read()
                if ret and frame is not None:
                    try:
                        frame = np.flip(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB), axis=1)
                        _, _ = self.gestures.step(frame, False, int(self.W), int(self.H), context=CTX)
                        raw = (0.0, 0.0)
                    except (TypeError, IndexError, ValueError):
                        raw = None
                else:
                    time.sleep(0.01)
            if raw is not None:
                with self.lock:
                    self.last_face = time.time()
            return
        raw = self.event(False)
        now = time.time()
        if raw is None:
            if now - self.last_face > FACE_TIMEOUT:
                with self.lock:
                    self.point = None
            return
        a, b, c, d = self.corr
        p = (float(a * raw[0] + b), float(c * raw[1] + d))
        self.recent.append((now, *p))
        while self.recent and now - self.recent[0][0] > STABLE_WINDOW:
            self.recent.popleft()
        # as gaze_scene_v2.py: moving = the second half of the last 0.3 s is far from the first half
        pts = np.array([r[1:] for r in self.recent])
        half = len(pts) // 2
        moving = bool(half >= 2 and np.linalg.norm(pts[half:].mean(0) - pts[:half].mean(0)) > MOVE_SHIFT_PX * self.W / 1440)
        with self.lock:
            self.point, self.moving, self.last_face = p, moving, now


class Handler(BaseHTTPRequestHandler):
    tracker: Tracker

    def log_message(self, fmt, *args):  # quiet: the app polls at 30 Hz
        pass

    def send_json(self, body, status=HTTPStatus.OK) -> None:
        data = json.dumps(body).encode()
        self.send_response(status)
        self.send_header("Content-Type", "application/json")
        self.send_header("Cache-Control", "no-store")
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def do_GET(self) -> None:
        if urlsplit(self.path).path == "/api/gaze":
            self.send_json(self.tracker.snapshot())
        else:
            self.send_json({"error": "Not found"}, HTTPStatus.NOT_FOUND)

    def do_POST(self) -> None:
        path = urlsplit(self.path).path
        try:
            n = int(self.headers.get("Content-Length") or 0)
            body = json.loads(self.rfile.read(n) or b"{}")
        except ValueError:
            return self.send_json({"error": "Invalid JSON"}, HTTPStatus.BAD_REQUEST)
        screen = body.get("screen")
        t = self.tracker
        if path in ("/api/calibrate", "/api/correct", "/api/load", "/api/cancel"):
            kind = path.rsplit("/", 1)[1]
            if not t.camera_ok:
                return self.send_json({"error": t.error or "The camera is not open yet."}, HTTPStatus.CONFLICT)
            if kind == "load" and not CALIB_FILE.exists() and not t.mock:
                return self.send_json({"error": "No saved calibration yet."}, HTTPStatus.CONFLICT)
            if kind == "correct" and not t.calibrated:
                return self.send_json({"error": "Calibrate first."}, HTTPStatus.CONFLICT)
            t.request(kind, screen)
            return self.send_json(t.snapshot())
        if path == "/api/mock":
            if not t.mock:
                return self.send_json({"error": "Only with --mock"}, HTTPStatus.CONFLICT)
            t.set_mock(body)
            return self.send_json(t.snapshot())
        self.send_json({"error": "Not found"}, HTTPStatus.NOT_FOUND)


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=8820)
    ap.add_argument("--camera", type=int, default=0, help="webcam index (front camera), as CAM_INDEX in the prototype")
    ap.add_argument("--mock", action="store_true", help="no camera; set the gaze with POST /api/mock")
    args = ap.parse_args()
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(name)s %(message)s")
    tracker = Tracker(args.camera, args.mock)
    tracker.start()
    Handler.tracker = tracker
    server = ThreadingHTTPServer((args.host, args.port), Handler)
    log.info("gaze service on http://%s:%d%s", args.host, args.port, " (mock)" if args.mock else "")
    server.serve_forever()


if __name__ == "__main__":
    main()
