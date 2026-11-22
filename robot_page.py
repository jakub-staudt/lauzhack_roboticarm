import json
import queue
import threading
import time
import urllib.error
import urllib.request
import uuid

import cv2
import numpy as np


MOVE_WAIT_S = 90.0


class RobotPageError(RuntimeError):
    pass


class RobotPage:
    def __init__(self, url="http://127.0.0.1:8820", poll_s=0.15, timeout_s=3.0):
        self.url = url.rstrip("/")
        self.poll_s, self.timeout_s = poll_s, timeout_s
        self.lock = threading.Lock()
        self.frame = self.poly = None
        self.seq, self.status, self.error = None, "connecting", ""
        self.drive = {}
        self.last_result, self.busy = None, False
        self.moves = queue.Queue()
        self._inflight, self._epoch = 0, 0
        self.running = True
        threading.Thread(target=self._loop, daemon=True, name="robot-page").start()
        threading.Thread(target=self._move_loop, daemon=True, name="robot-moves").start()

    def _get(self, path):
        with urllib.request.urlopen(self.url + path, timeout=self.timeout_s) as r:
            return r.read()

    def _post(self, path, body, timeout=30.0):
        req = urllib.request.Request(self.url + path, data=json.dumps(body).encode(), method="POST",
                                     headers={"Content-Type": "application/json"})
        try:
            with urllib.request.urlopen(req, timeout=timeout) as r:
                return json.loads(r.read() or b"{}")
        except urllib.error.HTTPError as e:
            try:
                info = json.loads(e.read() or b"{}")
            except ValueError:
                info = {}
            raise RobotPageError(info.get("error") or f"HTTP {e.code}") from None
        except (urllib.error.URLError, OSError) as e:
            raise RobotPageError(f"robot page not reachable at {self.url}: {e}") from None

    def _loop(self):
        last_drive = 0.0
        while self.running:
            try:
                snap = json.loads(self._get("/api/detect"))
                src = snap.get("source", {})
                poly = snap.get("polygon_shown") if snap.get("locked") else None
                seq = snap.get("seq")
                if seq is not None and seq != self.seq:
                    img = cv2.imdecode(np.frombuffer(self._get(f"/frame/{seq}.jpg"), np.uint8), cv2.IMREAD_COLOR)
                    if img is not None:
                        with self.lock:
                            self.frame, self.seq = img, seq
                cal = snap.get("calibration", {})
                with self.lock:
                    self.poly = None if poly is None else np.array(poly, np.float32)
                    if src.get("status") != "live":
                        self.status, self.error = f"camera {src.get('status')}", src.get("error", "")
                    elif poly is None:
                        self.status, self.error = f"square: {cal.get('state')}", cal.get("reason", "")
                    else:
                        self.status, self.error = "live", ""
                if time.time() - last_drive > 1.0:
                    self.drive, last_drive = json.loads(self._get("/api/drive")), time.time()
            except urllib.error.HTTPError as e:
                if e.code != 404:
                    self.status, self.error = "error", f"HTTP {e.code}"
            except (urllib.error.URLError, OSError, ValueError) as e:
                self.status, self.error = "not reachable", f"{self.url}: {e}"
                time.sleep(1.0)
            time.sleep(self.poll_s)

    def read(self):
        with self.lock:
            return None if self.frame is None else self.frame.copy()

    def polygon(self):
        with self.lock:
            return None if self.poly is None else self.poly.copy()

    def close(self):
        self.running = False

    def go(self, px, py, target="sim", height_cm=3.0, label=""):
        gen = (self.drive or {}).get("generation", 0)
        with self.lock:
            self._inflight += 1
            self.moves.put((float(px), float(py), target, float(height_cm), label or "the point", gen, self._epoch))

    def pending(self):
        return self._inflight > 0

    def _clear(self):
        with self.lock:
            self._epoch += 1
            while True:
                try:
                    self.moves.get_nowait()
                    self._inflight -= 1
                except queue.Empty:
                    return

    def _move_loop(self):
        while self.running:
            try:
                px, py, target, height, label, gen, epoch = self.moves.get(timeout=0.5)
            except queue.Empty:
                continue
            self.busy = True
            try:
                t0 = time.time()
                while True:
                    if epoch != self._epoch:
                        raise RobotPageError(f"{label} dropped (stopped)")
                    d = json.loads(self._get("/api/drive"))
                    self.drive = d
                    if not d.get("enabled"):
                        raise RobotPageError("the robot page's drive panel is off")
                    task = ((d.get("real" if target == "real" else "sim") or {}).get("task") or {}).get("state")
                    if task not in ("planning", "running"):
                        break
                    if time.time() - t0 > MOVE_WAIT_S:
                        raise RobotPageError(f"the arm was still busy after {MOVE_WAIT_S:.0f} s; {label} dropped")
                    self.last_result = (True, f"arm still moving - {label} is next")
                    time.sleep(0.4)
                if epoch != self._epoch:
                    raise RobotPageError(f"{label} dropped (stopped)")
                offset = d["server_ms"] - time.time() * 1000 if "server_ms" in d else 0.0
                body = {"cid": uuid.uuid4().hex, "sent_ms": time.time() * 1000 + offset, "gen": gen,
                        "px": px, "py": py, "height_cm": height, "closed_loop": False,
                        "target": target, "confirm_real": target == "real"}
                ans = self._post("/api/drive/go", body)
                t = ans.get("target", {})
                arm = (f", arm x {t['x'] * 100:.1f} y {t['y'] * 100:.1f} cm" if "x" in t and "y" in t else "")
                self.last_result = (True, f"{'REAL arm' if ans.get('real') else 'simulator'} moving above {label}"
                                          f" (u {t.get('u')}, v {t.get('v')}{arm})")
            except (RobotPageError, ValueError, OSError, urllib.error.URLError) as e:
                self.last_result = (False, str(e))
                self._clear()
            finally:
                self.busy = False
                with self.lock:
                    self._inflight -= 1

    def stop(self):
        self._clear()
        try:
            self._post("/api/drive/stop", {}, timeout=3.0)
            return True
        except RobotPageError as e:
            print(f"(robot page stop failed: {e})")
            return False

    def recalibrate(self):
        try:
            self._post("/api/recalibrate", {}, timeout=3.0)
        except RobotPageError as e:
            print(f"(robot page recalibrate failed: {e})")


BLOCK_BINS = [
    ("red", [(0, 6), (165, 180)], 100, 256),
    ("orange", [(6, 15)], 110, 256),
    ("yellow", [(15, 36)], 110, 256),
    ("green", [(36, 88)], 70, 256),
    ("blue", [(88, 132)], 90, 256),
    ("wood", [(6, 23)], 30, 110),
]
BLOCK_DRAW = {"red": (235, 80, 80), "orange": (255, 160, 60), "yellow": (240, 210, 70), "green": (90, 220, 120),
              "blue": (90, 140, 255), "wood": (205, 160, 110)}
BLOCK_MIN_SIDE = 0.10
BLOCK_MAX_SIDE = 0.60
BLOCK_MIN_V = 60


def detect_blocks(frame, quad):
    h, w = frame.shape[:2]
    if quad is None:
        quad = np.array([(0, 0), (w - 1, 0), (w - 1, h - 1), (0, h - 1)], np.float32)
    quad = np.asarray(quad, np.float32)
    side = float(np.sqrt(max(cv2.contourArea(quad), 1.0)))
    inside = np.zeros((h, w), np.uint8)
    cv2.fillPoly(inside, [np.rint(quad).astype(np.int32)], 255)
    inside = cv2.dilate(inside, np.ones((3, 3), np.uint8), iterations=max(1, int(0.03 * side)))
    hsv = cv2.cvtColor(cv2.GaussianBlur(frame, (5, 5), 0), cv2.COLOR_BGR2HSV)
    hue, sat, val = hsv[..., 0].astype(np.int16), hsv[..., 1], hsv[..., 2]
    k = max(3, int(0.04 * side)) | 1
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (k, k))
    found = []
    for name, hues, smin, smax in BLOCK_BINS:
        m = np.zeros((h, w), bool)
        for lo, hi in hues:
            m |= (hue >= lo) & (hue < hi)
        m &= (sat >= smin) & (sat < smax) & (val >= BLOCK_MIN_V)
        mask = (m.astype(np.uint8) * 255) & inside
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        for c in cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[0]:
            area = cv2.contourArea(c)
            if not (BLOCK_MIN_SIDE * side) ** 2 <= area <= (BLOCK_MAX_SIDE * side) ** 2:
                continue
            mom = cv2.moments(c)
            cx, cy = mom["m10"] / mom["m00"], mom["m01"] / mom["m00"]
            if cv2.pointPolygonTest(quad, (float(cx), float(cy)), False) < 0:
                continue
            x, y, bw, bh = cv2.boundingRect(c)
            found.append({"cls": "object", "label": f"{name} block", "draw": BLOCK_DRAW[name],
                          "img": (float(cx), float(cy)), "bbox": (float(x), float(y), float(bw), float(bh))})
    return found
