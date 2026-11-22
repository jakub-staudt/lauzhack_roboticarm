import json
import math
import os
import urllib.request
from collections import defaultdict
import pickle
import sys
import threading
import time
import zlib

import cv2
import numpy as np
import pygame

from object_detect import ObjectDetector
from robot_page import RobotPage, detect_blocks
from work_area import find_work_area


CAM_INDEX = 1              
SCENE_CAM_INDEX = 0         
ROTATE_180 = False            
SCENE_ROTATE_180 = False  
SCENE_SIZE = (1280, 720)
AUTO_AREA_PART = "left"

HERE = os.path.dirname(os.path.abspath(__file__))
CALIB_FILE = os.path.join(HERE, "eg_calibration.pkl")
COMMAND_LOG = os.path.join(HERE, "scene_commands.jsonl")

CAL_GRID = [0.05, 0.275, 0.5, 0.725, 0.95]
CAL_SETTLE = 0.8
CAL_COLLECT = 1.2

SIGMA = 200
ON_INPUT = 0.45
DWELL_OBJECT_S = 2.0
DWELL_BUTTON_S = 1.5
DWELL_SPOT_S = 2.0
DEST_MODE = "zones"
DEST_ZONES = (2, 3)
ZONE_PENALTY_PX = 10
SPOT_RADIUS = 120
DECAY = 1.5
HYSTERESIS_PX = 20
STEP_GRACE_S = 0.8
STABLE_WINDOW = 0.3
MOVE_SHIFT_PX = 120
SHOW_DONE = 3.0

ADAPT = True
ADAPT_RATE = 0.3
ADAPT_MAX_STEP = 60
ADAPT_MAX_TOTAL = 150
ADAPT_WINDOW = 0.6

DETECTOR = "blobs"

BACKGROUND_FILE = os.path.join(HERE, "table_background.png")
SCENE_ROI = (0.0, 0.0, 0.5, 1.0)
WORK_INSET_PX = -10
BLOB_YOLO_NAMES = True
BLOB_YOLO_MIN_CONF = 0.60
YOLO_MODEL = os.path.join(HERE, "yolo11n.onnx")
YOLO_URL = "https://github.com/ultralytics/assets/releases/download/v8.3.0/yolo11n.onnx"
YOLO_CONF = 0.40
YOLO_SIZE = 640
YOLO_THREADS = 2
DETECT_MAX_FPS = 8
YOLO_EXCLUDE = {"person", "dining table"}
GRASPABLE_ONLY = True

GRASPABLE = {"bottle", "wine glass", "cup", "fork", "knife", "spoon", "bowl", "banana", "apple",
             "sandwich", "orange", "broccoli", "carrot", "hot dog", "donut", "cake", "cell phone",
             "remote", "mouse", "book", "clock", "vase", "scissors", "teddy bear", "toothbrush",
             "sports ball", "hair drier", "keyboard", "handbag", "potted plant"}
TRACK_MAX_JUMP = 150         
TRACK_JUMP_FACTOR = 0.6     
TRACK_KEEP_S = 1.5
TRACK_CONFIRM = 2
COCO_NAMES = ["person", "bicycle", "car", "motorcycle", "airplane", "bus", "train", "truck", "boat",
              "traffic light", "fire hydrant", "stop sign", "parking meter", "bench", "bird", "cat", "dog",
              "horse", "sheep", "cow", "elephant", "bear", "zebra", "giraffe", "backpack", "umbrella",
              "handbag", "tie", "suitcase", "frisbee", "skis", "snowboard", "sports ball", "kite",
              "baseball bat", "baseball glove", "skateboard", "surfboard", "tennis racket", "bottle",
              "wine glass", "cup", "fork", "knife", "spoon", "bowl", "banana", "apple", "sandwich", "orange",
              "broccoli", "carrot", "hot dog", "pizza", "donut", "cake", "chair", "couch", "potted plant",
              "bed", "dining table", "toilet", "tv", "laptop", "mouse", "remote", "keyboard", "cell phone",
              "microwave", "oven", "toaster", "sink", "refrigerator", "book", "clock", "vase", "scissors",
              "teddy bear", "hair drier", "toothbrush"]

COLOR_RANGES = {
    "red":    [((0, 120, 80), (8, 255, 255)), ((170, 120, 80), (179, 255, 255))],
    "green":  [((40, 80, 60), (85, 255, 255))],
    "blue":   [((95, 120, 60), (130, 255, 255))],
    "yellow": [((20, 120, 120), (35, 255, 255))],
}
DRAW_COLORS = {"red": (235, 80, 80), "green": (90, 220, 120), "blue": (90, 140, 255), "yellow": (240, 210, 70)}
MIN_AREA = 600              

TABLE_MARKERS_CM = {0: (-20.0, 40.0), 1: (20.0, 40.0), 2: (20.0, 10.0), 3: (-20.0, 10.0)}
ARUCO_DICT = cv2.aruco.DICT_4X4_50
USE_ARUCO = True             

PART_CORNERS_CM = {
    "left":   [(-20.0, 50.0), (0.0, 50.0), (0.0, 10.0), (-20.0, 10.0)],
    "right":  [(0.0, 50.0), (20.0, 50.0), (20.0, 10.0), (0.0, 10.0)],
    "top":    [(-20.0, 50.0), (20.0, 50.0), (20.0, 30.0), (-20.0, 30.0)],
    "bottom": [(-20.0, 30.0), (20.0, 30.0), (20.0, 10.0), (-20.0, 10.0)],
}
TABLE_POINTS_CM = list(zip(["top-left corner", "top-right corner", "bottom-right corner", "bottom-left corner"],
                           PART_CORNERS_CM[AUTO_AREA_PART]))

AUTO_AREA = True
AUTO_AREA_FRAMES = 5
TABLE_CALIB_FILE = os.path.join(HERE, "table_calibration.json")

OPERATIONS = [
    ("pick_place", "Pick & place", True),
    ("bring", "Bring to me", False),
    ("push", "Push towards", True),
    ("point", "Point at it", False),
]

ARM_ENABLED = False
CTX = "scene"

ROBOT_PAGE = None
ROBOT_TARGET = "sim"
if "--robot-page" in sys.argv:
    _i = sys.argv.index("--robot-page") + 1
    ROBOT_PAGE = sys.argv[_i] if _i < len(sys.argv) and not sys.argv[_i].startswith("--") else "http://127.0.0.1:8820"
if "--real" in sys.argv:
    ROBOT_TARGET = "real"
if ROBOT_PAGE:
    OPERATIONS = [("hover", "Hover above A, then B", True)]
    DEST_MODE = "free"

BG, FG, DIM = (30, 30, 30), (235, 235, 235), (110, 110, 110)
BLUE, ORANGE, RED, GREEN = (80, 140, 255), (255, 170, 60), (255, 90, 90), (90, 220, 120)


if "--markers" in sys.argv:
    d = cv2.aruco.getPredefinedDictionary(ARUCO_DICT)
    page = np.full((1400, 1000), 255, np.uint8)
    for i in range(4):
        m = cv2.aruco.generateImageMarker(d, i, 380)
        x, y = 60 + (i % 2) * 500, 80 + (i // 2) * 640
        page[y:y + 380, x:x + 380] = m
        cx, cy = TABLE_MARKERS_CM[i]
        cv2.putText(page, f"id {i}: center at ({cx:g}, {cy:g}) cm", (x, y + 440),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.9, 0, 2)
    out = os.path.join(HERE, "aruco_markers.png")
    cv2.imwrite(out, page)
    print(f"Wrote {out}. Print it, cut out the 4 markers and tape them flat at the table "
          f"positions printed under each one (or edit TABLE_MARKERS_CM).")
    sys.exit()

from eyeGestures import EyeGestures_v3         
from eyeGestures.utils import VideoCapture    


def send_to_arm(operation, obj_name, obj_cm, dest_name=None, dest_cm=None):
    print(f"   [arm] {operation}: {obj_name} at {obj_cm} cm" + (f" -> {dest_name} at {dest_cm} cm" if dest_cm else ""))


pygame.init()
info = pygame.display.Info()
W, H = info.current_w, info.current_h
screen = pygame.display.set_mode((W, H), pygame.FULLSCREEN)
font = pygame.font.Font(None, 36)
big = pygame.font.Font(None, 64)
clock = pygame.time.Clock()

gestures = EyeGestures_v3()
cap = VideoCapture(CAM_INDEX)


def new_calibration_map():
    pts = []
    for r, y in enumerate(CAL_GRID):
        row = CAL_GRID if r % 2 == 0 else CAL_GRID[::-1]
        pts += [(x, y) for x in row]
    return np.array(pts)


calibration_map = new_calibration_map()
gestures.uploadCalibrationMap(calibration_map, context=CTX)
gestures.setFixation(1.0)


def get_event(calibrate):
    ret, frame = cap.read()
    if not ret or frame is None:
        return None, None
    if ROTATE_180:
        frame = cv2.rotate(frame, cv2.ROTATE_180)
    frame = np.flip(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB), axis=1)
    try:
        return gestures.step(frame, calibrate, W, H, context=CTX)
    except (TypeError, IndexError, ValueError):
        return None, None


def poll_keys():
    keys = set()
    for e in pygame.event.get():
        if e.type == pygame.QUIT:
            keys.add(pygame.K_ESCAPE)
        elif e.type == pygame.KEYDOWN:
            keys.add(e.key)
    if pygame.K_ESCAPE in keys:
        finish()
    return keys


def text_center(s, y, f=font, color=FG):
    surf = f.render(s, True, color)
    screen.blit(surf, surf.get_rect(center=(W // 2, int(y))))


def message(lines, accept=(pygame.K_SPACE,)):
    while True:
        keys = poll_keys()
        for k in accept:
            if k in keys:
                return k
        get_event(False)
        screen.fill(BG)
        for i, line in enumerate(lines):
            text_center(line, H * 0.28 + i * 50, big if i == 0 else font, BLUE if i == 0 else FG)
        pygame.display.flip()
        clock.tick(60)


def calibrate():
    global calibration_map
    calibration_map = new_calibration_map()
    gestures.reset(context=CTX)
    gestures.uploadCalibrationMap(calibration_map, context=CTX)
    clb = gestures.clb[CTX]
    clb.acceptance_radius = -1       
    clb.calibration_radius = 10 ** 6  
    n_cal = len(calibration_map)
    message(["Calibration", f"{n_cal} dots across the whole screen, about 2 seconds each.",
             "Keep your head still. Look at the center of each dot until it moves on.",
             "Press SPACE to start"])
    for i in range(n_cal):
        tx, ty = (float(v) for v in clb.getCurrentPoint(W, H))
        t0, prev_t, last_face = time.time(), time.time(), time.time()
        while True:
            now = time.time()
            el = now - t0
            if el >= CAL_SETTLE + CAL_COLLECT:
                break
            poll_keys()
            recording = el > CAL_SETTLE
            ev, _ = get_event(recording)
            if ev is None:
                t0 += now - prev_t        
            else:
                last_face = now
            prev_t = now
            screen.fill(BG)
            frac = min(el / (CAL_SETTLE + CAL_COLLECT), 1.0)
            pygame.draw.circle(screen, BLUE, (tx, ty), int(40 * (1 - frac) + 8), 4)
            pygame.draw.circle(screen, GREEN if recording else FG, (tx, ty), 6)
            text_center(f"Calibration {i + 1}/{n_cal}", 30, font, DIM)
            if now - last_face > 1.0:
                text_center("No face detected - check camera, lighting and position", H / 2, font, RED)
            pygame.display.flip()
            clock.tick(60)
        clb.movePoint()
    try:
        data = gestures.saveModel(context=CTX)
        with open(CALIB_FILE, "wb") as f:
            f.write(data)
    except Exception as e:
        print(f"(could not save calibration: {e})")


corr = np.array([1.0, 0.0, 1.0, 0.0])
corr_base = corr.copy()


def correct(raw):
    return (corr[0] * raw[0] + corr[1], corr[2] * raw[1] + corr[3])


def drift_correct():
    global corr, corr_base
    spots = [(0.5, 0.5), (0.2, 0.25), (0.8, 0.25), (0.8, 0.75), (0.2, 0.75)]
    raw, true = [], []
    for fx, fy in spots:
        tx, ty = fx * W, fy * H
        samples, t0 = [], time.time()
        while time.time() - t0 < 1.4:
            poll_keys()
            ev, _ = get_event(False)
            if ev is not None and not bool(getattr(ev, "blink", False)) and time.time() - t0 > 0.5:
                samples.append(np.array(ev.point, dtype=float))
            screen.fill(BG)
            pygame.draw.line(screen, FG, (tx - 25, ty), (tx + 25, ty), 3)
            pygame.draw.line(screen, FG, (tx, ty - 25), (tx, ty + 25), 3)
            text_center("Gaze correction: look at the cross", 30, font, DIM)
            pygame.display.flip()
            clock.tick(60)
        if len(samples) >= 5:
            raw.append(np.median(samples, axis=0))
            true.append((tx, ty))
    if len(raw) < 3:
        print("Gaze correction skipped: not enough tracking data")
        return
    raw, true = np.array(raw), np.array(true)
    new = []
    for k in (0, 1):
        if np.ptp(raw[:, k]) > 30:                     
            a, b = np.polyfit(raw[:, k], true[:, k], 1)
            a = float(np.clip(a, 0.6, 2.5))
            b = float(np.mean(true[:, k] - a * raw[:, k]))
        else:
            a, b = 1.0, float(np.mean(true[:, k] - raw[:, k]))
        new += [a, b]
    corr = np.array(new)
    corr_base = corr.copy()
    print(f"Gaze correction: x' = {corr[0]:.2f}*x {corr[1]:+.0f},  y' = {corr[2]:.2f}*y {corr[3]:+.0f}")


def finish():
    pygame.quit()
    if ROBOT_PAGE and "scene" in globals():
        scene.stop()
    for c in (cap, scene):
        try:
            c.close()
        except Exception:
            pass
    print("\n# Commands as (operation, object, object_cm, object_px, destination, destination_cm, destination_px)")
    if ROBOT_PAGE:
        print("# px = robot camera pixels; cm = None here: the robot page converts px to arm coordinates itself "
              "(see the 'robot page: ... arm x .. y .. cm' lines above)")
    else:
        print("# cm = arm coordinates (None if the table isn't calibrated: press T); px = scene camera pixels")
    print(f"commands = {commands}")
    sys.stdout.flush()
    os._exit(0)             


commands = []


class SceneCamera:

    def __init__(self, index):
        self.cap = cv2.VideoCapture(index)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, SCENE_SIZE[0])
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, SCENE_SIZE[1])
        self.frame, self.running = None, True
        self.lock = threading.Lock()
        threading.Thread(target=self._loop, daemon=True).start()

    def _loop(self):
        while self.running:
            ok, f = self.cap.read()
            if ok:
                if SCENE_ROTATE_180:
                    f = cv2.rotate(f, cv2.ROTATE_180)
                with self.lock:
                    self.frame = f
            else:
                time.sleep(0.02)

    def read(self):
        with self.lock:
            return None if self.frame is None else self.frame.copy()

    def close(self):
        self.running = False
        self.cap.release()


scene = RobotPage(ROBOT_PAGE) if ROBOT_PAGE else SceneCamera(SCENE_CAM_INDEX)
aruco = cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(ARUCO_DICT), cv2.aruco.DetectorParameters())
table_H = None                 
table_source = None
work_poly = None
saved_table_source = None
if ROBOT_PAGE:
    table_source = "robot page"
elif os.path.exists(TABLE_CALIB_FILE):
    try:
        _tc = json.load(open(TABLE_CALIB_FILE))
        if _tc.get("scene_cam_index") != SCENE_CAM_INDEX:
            print(f"(saved table calibration is for scene camera {_tc.get('scene_cam_index')} - ignored)")
        elif _tc.get("part") != AUTO_AREA_PART:
            print(f"(saved table calibration is for the {_tc.get('part', 'right')} part, not the "
                  f"{AUTO_AREA_PART} part - ignored)")
        else:
            table_H, table_source = np.array(_tc["H"], dtype=np.float64), "saved"
            saved_table_source = _tc.get("source")
            if "clicked_px" in _tc:
                work_poly = np.array(_tc["clicked_px"], np.float32)
            print(f"Loaded table calibration from {TABLE_CALIB_FILE}")
    except Exception as e:
        print(f"(could not load table calibration: {e})")
view = {"s": 1.0, "ox": 0, "oy": 0, "fw": 1, "fh": 1, "x0": 0, "y0": 0, "cw": 1, "ch": 1}
ZOOM_TO_WORK_AREA = True
ZOOM_MARGIN = 0.12


def fit_view(fw, fh, zoom=False):
    x0, y0, cw, ch = 0, 0, fw, fh
    poly = work_area_poly(fw, fh) if zoom and ZOOM_TO_WORK_AREA else None
    if poly is not None:
        (ax, ay), (bx, by) = poly.min(0), poly.max(0)
        m = ZOOM_MARGIN * max(bx - ax, by - ay)
        cx0, cy0 = int(max(0, ax - m)), int(max(0, ay - m))
        cx1, cy1 = int(min(fw, bx + m + 1)), int(min(fh, by + m + 1))
        if cx1 - cx0 > 20 and cy1 - cy0 > 20:
            x0, y0, cw, ch = cx0, cy0, cx1 - cx0, cy1 - cy0
    s = min(W / cw, H / ch)
    view.update(s=s, ox=(W - cw * s) / 2, oy=(H - ch * s) / 2, fw=fw, fh=fh, x0=x0, y0=y0, cw=cw, ch=ch)


def img2scr(x, y):
    return view["ox"] + (x - view["x0"]) * view["s"], view["oy"] + (y - view["y0"]) * view["s"]


def scr2img(x, y):
    return (x - view["ox"]) / view["s"] + view["x0"], (y - view["oy"]) / view["s"] + view["y0"]


def detect_colors(frame):
    small = cv2.resize(frame, None, fx=0.5, fy=0.5)
    hsv = cv2.cvtColor(cv2.GaussianBlur(small, (5, 5), 0), cv2.COLOR_BGR2HSV)
    kernel = np.ones((5, 5), np.uint8)
    found = []
    for name, ranges in COLOR_RANGES.items():
        mask = np.zeros(hsv.shape[:2], np.uint8)
        for lo, hi in ranges:
            mask |= cv2.inRange(hsv, np.array(lo), np.array(hi))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            continue
        c = max(contours, key=cv2.contourArea)
        if cv2.contourArea(c) * 4 < MIN_AREA:
            continue
        m = cv2.moments(c)
        x, y, w, h = cv2.boundingRect(c)
        found.append({"cls": f"{name} object", "draw": DRAW_COLORS[name],
                      "img": (2 * m["m10"] / m["m00"], 2 * m["m01"] / m["m00"]),
                      "bbox": (2 * x, 2 * y, 2 * w, 2 * h)})
    return found


class YoloDetector:

    def __init__(self):
        try:
            import onnxruntime as ort
        except ImportError:
            sys.exit('onnxruntime is missing:  pip install onnxruntime "protobuf>=4.25.8,<5"\n'
                     '(or set DETECTOR = "color" at the top of the file)')
        if not os.path.exists(YOLO_MODEL):
            print(f"Downloading the object detection model (11 MB) to {YOLO_MODEL} ...")
            urllib.request.urlretrieve(YOLO_URL, YOLO_MODEL)
        opts = ort.SessionOptions()
        opts.intra_op_num_threads = YOLO_THREADS
        opts.inter_op_num_threads = 1
        opts.execution_mode = ort.ExecutionMode.ORT_SEQUENTIAL
        opts.add_session_config_entry("session.intra_op.allow_spinning", "0")
        self.session = ort.InferenceSession(YOLO_MODEL, sess_options=opts, providers=["CPUExecutionProvider"])
        self.input = self.session.get_inputs()[0].name

    def __call__(self, frame, graspable_filter=True):
        h, w = frame.shape[:2]
        s = YOLO_SIZE / max(h, w)
        nh, nw = int(round(h * s)), int(round(w * s))
        canvas = np.full((YOLO_SIZE, YOLO_SIZE, 3), 114, np.uint8)    
        canvas[:nh, :nw] = cv2.resize(frame, (nw, nh))
        blob = cv2.dnn.blobFromImage(canvas, 1 / 255, swapRB=True)
        p = self.session.run(None, {self.input: blob})[0][0].T            
        conf, cls = p[:, 4:].max(1), p[:, 4:].argmax(1)
        keep = conf > YOLO_CONF
        boxes = [[float((cx - bw / 2) / s), float((cy - bh / 2) / s), float(bw / s), float(bh / s)]
                 for cx, cy, bw, bh in p[keep, :4]]
        found = []
        for k in np.array(cv2.dnn.NMSBoxes(boxes, conf[keep].tolist(), YOLO_CONF, 0.5)).flatten():
            name = COCO_NAMES[int(cls[keep][k])]
            if name in YOLO_EXCLUDE or (graspable_filter and GRASPABLE_ONLY and name not in GRASPABLE):
                continue
            x0, y0, bw, bh = boxes[k]
            found.append({"cls": name, "draw": class_color(name), "conf": float(conf[keep][k]),
                          "img": (x0 + bw / 2, y0 + bh / 2), "bbox": (x0, y0, bw, bh)})
        return found


background = {"img": None}


def set_background(img):
    background["img"] = img


def capture_background():
    frames, t0 = [], time.time()
    while len(frames) < 12 and time.time() - t0 < 3:
        f = scene.read()
        if f is not None:
            frames.append(f)
        time.sleep(0.05)
    if not frames:
        print("No scene image - cannot take the empty-table picture.")
        return
    img = np.median(np.stack(frames), axis=0).astype(np.uint8)
    set_background(img)
    cv2.imwrite(BACKGROUND_FILE, img)
    print(f"Empty-table picture saved to {BACKGROUND_FILE}")


def detect_blobs(frame, tracker, namer=None):
    found = tracker(frame, work_area_poly(frame.shape[1], frame.shape[0]), background["img"])
    if namer is not None and found:
        for y_obj in namer(frame):                              
            if y_obj["conf"] < BLOB_YOLO_MIN_CONF:
                continue
            yx, yy, yw, yh = y_obj["bbox"]
            for b in found:
                bx, by, bw, bh = b["bbox"]
                ix = max(0, min(bx + bw, yx + yw) - max(bx, yx))
                iy = max(0, min(by + bh, yy + yh) - max(by, yy))
                inter = ix * iy
                if inter / (bw * bh + yw * yh - inter + 1e-6) > 0.3:
                    b["label"] = y_obj["cls"]
    return found


def make_detector():
    if ROBOT_PAGE:
        return lambda frame: [] if work_poly is None else detect_blocks(frame, work_poly)
    if DETECTOR == "yolo":
        return YoloDetector()
    if DETECTOR == "color":
        return detect_colors
    namer = None
    if BLOB_YOLO_NAMES:
        try:
            import onnxruntime
            namer = YoloDetector()
        except (ImportError, SystemExit):
            print("(onnxruntime not installed: objects are named by color only)")
    tracker = ObjectDetector()
    return lambda frame: detect_blobs(frame, tracker, namer)


def class_color(name):
    rng = np.random.default_rng(zlib.crc32(name.encode()))
    return tuple(int(v) for v in rng.integers(90, 255, 3))


class DetectionWorker:

    def __init__(self, detect):
        self.detect, self.results, self.stamp, self.fps = detect, [], 0.0, 0.0
        self.lock = threading.Lock()
        threading.Thread(target=self._loop, daemon=True).start()

    def _loop(self):
        while True:
            frame = scene.read()
            if frame is None:
                time.sleep(0.02)
                continue
            t0 = time.time()
            try:
                found = self.detect(frame)
            except Exception as e:
                print(f"(detection error: {e})")
                time.sleep(0.5)
                continue
            with self.lock:
                self.results, self.stamp = found, time.time()
            spent = time.time() - t0
            time.sleep(max(0.0, 1.0 / DETECT_MAX_FPS - spent))     
            self.fps = 0.8 * self.fps + 0.2 / max(time.time() - t0, 1e-3)

    def get(self):
        with self.lock:
            return list(self.results), self.stamp


tracks = []                  
last_stamp = 0.0


def update_tracks(dets, now):
    pairs = []
    for i, tr in enumerate(tracks):
        for j, d in enumerate(dets):
            if d["cls"] != tr["cls"]:
                continue
            _, _, w, h = d["bbox"]
            dist = math.dist(d["img"], tr["img"])
            if dist < max(TRACK_MAX_JUMP, TRACK_JUMP_FACTOR * math.hypot(w, h)):
                pairs.append((dist, i, j))
    matched, free = set(), set(range(len(dets)))
    for _, i, j in sorted(pairs):
        if i in matched or j not in free:
            continue
        matched.add(i)
        free.discard(j)
        tr = tracks[i]
        tr.update(img=dets[j]["img"], bbox=dets[j]["bbox"], last=now, hits=tr["hits"] + 1)
        new_label = dets[j].get("label")
        if new_label and new_label != tr.get("label") and not new_label.endswith(" object") \
                and tr.get("label", "").endswith(" object") and state == "object":
            used = {t["n"] for t in tracks if t.get("label") == new_label}
            n = next(k for k in range(1, 100) if k not in used)
            tr.update(label=new_label, n=n, name=new_label if n == 1 else f"{new_label} {n}")
    tracks[:] = [t for t in tracks if now - t["last"] < TRACK_KEEP_S]
    for j in sorted(free):
        d = dets[j]
        label = d.get("label", d["cls"])
        used = {t["n"] for t in tracks if t["label"] == label}
        n = next(k for k in range(1, 100) if k not in used)
        tracks.append({"name": label if n == 1 else f"{label} {n}", "cls": d["cls"], "label": label, "n": n,
                       "draw": d["draw"], "img": d["img"], "bbox": d["bbox"], "last": now, "hits": 1})


def update_table(frame):
    global table_H, table_source, work_poly
    if not USE_ARUCO:
        return {}
    corners, ids, _ = aruco.detectMarkers(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY))
    seen = {}
    if ids is not None:
        for c, i in zip(corners, ids.flatten()):
            if int(i) in TABLE_MARKERS_CM:
                seen[int(i)] = c[0].mean(axis=0)
    if len(seen) == 4:
        src = np.array([seen[i] for i in sorted(seen)], np.float32)
        dst = np.array([TABLE_MARKERS_CM[i] for i in sorted(seen)], np.float32)
        table_H, _ = cv2.findHomography(src, dst)
        table_source = "markers"
        work_poly = src
    return seen


def click_table_calibration():
    global table_H, table_source, work_poly
    frame = scene.read()
    if frame is None:
        print("No scene image - cannot calibrate the table.")
        return
    fit_view(frame.shape[1], frame.shape[0])
    surf = frame_surface(frame)
    clicks = []
    while len(clicks) < len(TABLE_POINTS_CM):
        for e in pygame.event.get():
            if e.type == pygame.KEYDOWN and e.key == pygame.K_ESCAPE:
                print("Table calibration cancelled.")
                return
            if e.type == pygame.MOUSEBUTTONDOWN and e.button == 1:
                clicks.append(e.pos)
            if e.type == pygame.KEYDOWN and e.key == pygame.K_BACKSPACE and clicks:
                clicks.pop()
        screen.fill(BG)
        screen.blit(surf, (view["ox"], view["oy"]))
        for i, c in enumerate(clicks):
            pygame.draw.circle(screen, GREEN, c, 9)
            screen.blit(font.render(str(i + 1), True, GREEN), (c[0] + 12, c[1] - 12))
        if len(clicks) > 1:
            pygame.draw.lines(screen, GREEN, False, clicks, 2)
        if len(clicks) < len(TABLE_POINTS_CM):
            name, cm = TABLE_POINTS_CM[len(clicks)]
            text_center(f"Table calibration: click point {len(clicks) + 1}/{len(TABLE_POINTS_CM)}: "
                        f"the {name}  ({cm[0]:g}, {cm[1]:g}) cm", 40, font, ORANGE)
        text_center("Backspace = undo last click    ESC = cancel", H - 40, font, DIM)
        pygame.display.flip()
        clock.tick(60)
    src = np.array([scr2img(*c) for c in clicks], np.float32)
    dst = np.array([cm for _, cm in TABLE_POINTS_CM], np.float32)
    table_H, _ = cv2.findHomography(src, dst)
    table_source = "clicked"
    work_poly = src
    json.dump({"scene_cam_index": SCENE_CAM_INDEX, "part": AUTO_AREA_PART, "source": "clicked", "H": table_H.tolist(),
               "clicked_px": src.tolist(), "points_cm": dst.tolist()}, open(TABLE_CALIB_FILE, "w"), indent=2)
    print(f"Table calibrated by clicks, saved to {TABLE_CALIB_FILE}")


def work_area_poly(fw, fh):
    if work_poly is not None:
        return work_poly
    if SCENE_ROI is None or ROBOT_PAGE:
        return None
    l, t, r, b = SCENE_ROI
    return np.array([(l * fw, t * fh), (r * fw, t * fh), (r * fw, b * fh), (l * fw, b * fh)], np.float32)


def in_work_area(x, y, fw, fh):
    poly = work_area_poly(fw, fh)
    if poly is None:
        return True
    return cv2.pointPolygonTest(poly, (float(x), float(y)), True) >= WORK_INSET_PX


def auto_table_calibration(preview_s=1.5):
    global table_H, table_source, work_poly
    frames, t0 = [], time.time()
    while len(frames) < AUTO_AREA_FRAMES and time.time() - t0 < 3:
        f = scene.read()
        if f is not None and (not frames or not np.array_equal(f, frames[-1])):
            frames.append(f)
        time.sleep(0.1)
    if not frames:
        print("No scene image - cannot look for the black lines.")
        return False
    found = find_work_area(frames, AUTO_AREA_PART)
    if found is None:
        print("Black lines not found - check that the whole border and the middle line are in view, or press T "
              "and click the corners.  Check a photo with:  python examples/work_area.py photo.jpg left out.png")
        return False
    quad = found["quad"].astype(np.float32)
    table_H, _ = cv2.findHomography(quad, np.array([cm for _, cm in TABLE_POINTS_CM], np.float32))
    table_source, work_poly = "black lines", quad
    json.dump({"scene_cam_index": SCENE_CAM_INDEX, "part": AUTO_AREA_PART, "source": "black lines",
               "H": table_H.tolist(), "clicked_px": quad.tolist(),
               "points_cm": [list(cm) for _, cm in TABLE_POINTS_CM]}, open(TABLE_CALIB_FILE, "w"), indent=2)
    print(f"Work area found from black lines: the {AUTO_AREA_PART} part, confidence {found['confidence']:.2f}, "
          f"corners {np.round(quad).astype(int).tolist()}")
    frame = frames[-1]
    fit_view(frame.shape[1], frame.shape[0])
    surf = frame_surface(frame)
    t0 = time.time()
    while time.time() - t0 < preview_s:
        pygame.event.pump()
        screen.fill(BG)
        screen.blit(surf, (view["ox"], view["oy"]))
        pygame.draw.polygon(screen, (150, 150, 150), [img2scr(*p) for p in found["paper_quad"]], 2)
        pygame.draw.polygon(screen, GREEN, [img2scr(*p) for p in quad], 5)
        text_center(f"Work area: the {AUTO_AREA_PART} part of the paper", 40, font, GREEN)
        pygame.display.flip()
        clock.tick(30)
    return True


def img_to_table(x, y):
    if table_H is None:
        return None
    p = cv2.perspectiveTransform(np.array([[[x, y]]], np.float32), table_H)[0, 0]
    return round(float(p[0]), 1), round(float(p[1]), 1)


if os.path.exists(BACKGROUND_FILE):
    _bg = cv2.imread(BACKGROUND_FILE)
    if _bg is not None:
        set_background(_bg)
        print(f"Loaded empty-table picture from {BACKGROUND_FILE} (press B to retake)")
detector = DetectionWorker(make_detector())


CANCEL_POS = (W - 140, H - 150) 


def buttons():
    bw, bh, gap = int(W * 0.24), int(H * 0.13), int(H * 0.035)
    items = [(k, label) for k, label, _ in OPERATIONS] + [("cancel", "Cancel")]
    total = len(items) * bh + (len(items) - 1) * gap
    y0, x0 = (H - total) / 2, W - bw - 40
    return [{"key": k, "label": label, "rect": pygame.Rect(x0, y0 + i * (bh + gap), bw, bh),
             "pos": (x0 + bw / 2, y0 + i * (bh + gap) + bh / 2)} for i, (k, label) in enumerate(items)]


def gaze_distance(gaze, center, payload):
    if "poly_img" in payload:                 
        poly = np.array([img2scr(*p) for p in payload["poly_img"]], np.float32)
        inside = cv2.pointPolygonTest(poly, (float(gaze[0]), float(gaze[1])), True)
        return max(0.0, -inside) + 0.3 * math.dist(gaze, center) + ZONE_PENALTY_PX
    if "bbox" not in payload:
        return math.dist(gaze, center)
    x, y, w, h = payload["bbox"]
    x0, y0 = img2scr(x, y)
    x1, y1 = img2scr(x + w, y + h)
    dx = max(x0 - gaze[0], 0, gaze[0] - x1)
    dy = max(y0 - gaze[1], 0, gaze[1] - y1)
    return math.hypot(dx, dy) + 0.3 * math.dist(gaze, center)


def dest_zones():
    cols, rows = DEST_ZONES
    if table_H is not None:
        xs = [p[0] for _, p in TABLE_POINTS_CM]
        ys = [p[1] for _, p in TABLE_POINTS_CM]
        inv = np.linalg.inv(table_H)

        def at(u, v):
            cm = np.array([[[min(xs) + u * (max(xs) - min(xs)), max(ys) - v * (max(ys) - min(ys))]]], np.float32)
            return tuple(float(t) for t in cv2.perspectiveTransform(cm, inv)[0, 0])
    else:
        fw, fh = view["fw"], view["fh"]
        l, t, r, b = SCENE_ROI if SCENE_ROI is not None else (0.0, 0.0, 1.0, 1.0)
        l, r = l + 0.1 * (r - l), r - 0.1 * (r - l)       
        t, b = t + 0.1 * (b - t), b - 0.1 * (b - t)

        def at(u, v):
            return (fw * (l + (r - l) * u), fh * (t + (b - t) * v))
    zones = []
    for r in range(rows):
        for c in range(cols):
            u0, u1, v0, v1 = c / cols, (c + 1) / cols, r / rows, (r + 1) / rows
            poly = [at(u0, v0), at(u1, v0), at(u1, v1), at(u0, v1)]
            zones.append({"name": f"zone {chr(65 + r)}{c + 1}", "zone": True, "poly_img": poly,
                          "img": at((u0 + u1) / 2, (v0 + v1) / 2)})
    return zones


def draw_ring(center, frac, radius=70, color=BLUE):
    pygame.draw.circle(screen, (60, 60, 60), center, radius, 5)
    if frac > 0.01:
        r = pygame.Rect(center[0] - radius, center[1] - radius, 2 * radius, 2 * radius)
        pygame.draw.arc(screen, color, r, math.pi / 2 - 2 * math.pi * min(frac, 1.0), math.pi / 2, 8)


def frame_surface(frame, dim=False):
    rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    poly = work_area_poly(frame.shape[1], frame.shape[0])
    if poly is not None:                         
        mask = np.zeros(frame.shape[:2], np.uint8)
        cv2.fillPoly(mask, [poly.astype(np.int32)], 255)
        rgb = np.where(mask[..., None] > 0, rgb, (rgb * 0.3).astype(np.uint8))
    if dim:
        rgb = (rgb * 0.45).astype(np.uint8)
    x0, y0, cw, ch = view["x0"], view["y0"], view["cw"], view["ch"]
    rgb = np.ascontiguousarray(rgb[y0:y0 + ch, x0:x0 + cw])
    size = (int(cw * view["s"]), int(ch * view["s"]))
    rgb = cv2.resize(rgb, size)
    return pygame.image.frombuffer(rgb.tobytes(), size, "RGB")


if os.path.exists(CALIB_FILE) and os.path.getsize(CALIB_FILE) > 0:
    key = message(["Calibration found", "L = load the saved calibration (same seat, same camera position)",
                   "K = calibrate again"], accept=(pygame.K_l, pygame.K_k))
    if key == pygame.K_l:
        try:
            with open(CALIB_FILE, "rb") as f:
                gestures.loadModel(f.read(), context=CTX)
        except Exception as e:
            print(f"(saved calibration is broken: {e} - calibrating again)")
            calibrate()
    else:
        calibrate()
else:
    calibrate()

if AUTO_AREA and saved_table_source != "clicked" and not ROBOT_PAGE:
    auto_table_calibration()

message(["Look at the world",
         "The table camera is shown full screen. Look at an object for 2 s to choose it,",
         "then look at an operation button, then (if asked) at a destination.",
         "C = cancel   D = correction   ESC = quit",
         "Press SPACE to start"])
drift_correct()

state = "object"           
chosen = operation = dest = None
dwell = {}                 
spot_anchor, spot_dwell, spot_pts = None, 0.0, []
state_until = 0.0
show_gaze = True
recent, pre_sel = [], []
moving = False
last_t = time.time()
objects, markers_seen = [], {}
last_frame = None
last_on_id = None
zones = []
grace_until = 0.0
robot_reported = None
robot_flash = ("", 0.0)


def adapt_to(pos):
    if ADAPT and len(pre_sel) >= 5:
        seen = np.median(np.array([p[1:] for p in pre_sel]), axis=0)
        step = np.clip(ADAPT_RATE * (np.array(pos) - seen), -ADAPT_MAX_STEP, ADAPT_MAX_STEP)
        corr[1] = np.clip(corr[1] + step[0], corr_base[1] - ADAPT_MAX_TOTAL, corr_base[1] + ADAPT_MAX_TOTAL)
        corr[3] = np.clip(corr[3] + step[1], corr_base[3] - ADAPT_MAX_TOTAL, corr_base[3] + ADAPT_MAX_TOTAL)


def reset_flow():
    global state, chosen, operation, dest, spot_anchor, spot_dwell, spot_pts
    state, chosen, operation, dest = "object", None, None, None
    spot_anchor, spot_dwell, spot_pts = None, 0.0, []
    dwell.clear()


def finish_command(now):
    global state, state_until
    for o in objects:                        
        if o["name"] == chosen["name"]:
            chosen.update(img=o["img"], bbox=o["bbox"])
        if dest and dest.get("name") == o["name"]:
            dest.update(img=o["img"])
    obj_cm = img_to_table(*chosen["img"])
    dest_cm = img_to_table(*dest["img"]) if dest else None
    rec = {"t": round(now, 2), "operation": operation[0], "object": chosen["name"],
           "object_px": [round(v) for v in chosen["img"]], "object_cm": obj_cm,
           "destination": dest["name"] if dest else None,
           "destination_px": [round(v) for v in dest["img"]] if dest else None, "destination_cm": dest_cm}
    obj_px = tuple(round(v) for v in chosen["img"])
    dest_px = tuple(round(v) for v in dest["img"]) if dest else None
    commands.append((operation[0], chosen["name"], obj_cm, obj_px, dest["name"] if dest else None, dest_cm, dest_px))
    with open(COMMAND_LOG, "a") as f:
        f.write(json.dumps(rec) + "\n")
    print(f"COMMAND: {operation[1]} | {chosen['name']} at {obj_cm} cm, pixel {obj_px}"
          + (f" -> {dest['name']} at {dest_cm} cm, pixel {dest_px}" if dest else ""))
    if ROBOT_PAGE:
        if dest:
            scene.go(*dest["img"], target=ROBOT_TARGET, label="the chosen place (B)")
        print(f"   -> robot page ({'REAL arm' if ROBOT_TARGET == 'real' else 'simulator'}): above A {obj_px}, then B {dest_px}")
    elif obj_cm is None:
        print("   (table not calibrated: press T and click 4 known points - cm unknown, pixels given)")
    if ARM_ENABLED and obj_cm is not None:
        send_to_arm(operation[0], chosen["name"], obj_cm, dest["name"] if dest else None, dest_cm)
    state, state_until = "done", now + SHOW_DONE


while True:
    keys = poll_keys()
    if pygame.K_c in keys:
        reset_flow()
    if pygame.K_g in keys:
        show_gaze = not show_gaze
    if pygame.K_s in keys and ROBOT_PAGE:
        print("STOP: robot page soft stop" if scene.stop() else "STOP failed - use the robot page's Stop")
        reset_flow()
    if pygame.K_t in keys and not ROBOT_PAGE:
        click_table_calibration()
        reset_flow()
    if pygame.K_a in keys:
        if ROBOT_PAGE:
            scene.recalibrate()
        else:
            auto_table_calibration()
        tracks.clear()
        reset_flow()
    if pygame.K_b in keys:
        capture_background()
        tracks.clear()
        reset_flow()
    if pygame.K_d in keys:
        drift_correct()
        dwell.clear()
        pre_sel.clear()
    if pygame.K_k in keys:
        calibrate()
        drift_correct()
        reset_flow()

    frame = scene.read()
    if frame is not None:
        last_frame = frame
        if ROBOT_PAGE:
            work_poly = scene.polygon()
        else:
            markers_seen = update_table(frame)
        fit_view(frame.shape[1], frame.shape[0], zoom=True)
    if ROBOT_PAGE and scene.last_result is not None and scene.last_result is not robot_reported:
        robot_reported = scene.last_result
        print(("   robot page: " if robot_reported[0] else "   robot page REFUSED: ") + robot_reported[1])
        if not robot_reported[0]:
            robot_flash = (f"Robot: {robot_reported[1]}", time.time() + 6.0)
            if state == "destination":
                reset_flow()
    dets, stamp = detector.get()
    if stamp != last_stamp:
        last_stamp = stamp
        dets = [d for d in dets if in_work_area(*d["img"], view["fw"], view["fh"])]
        update_tracks(dets, stamp)
    objects = [t for t in tracks if time.time() - t["last"] < TRACK_KEEP_S and t["hits"] >= TRACK_CONFIRM]

    ev, _ = get_event(False)
    now = time.time()
    dt = min(now - last_t, 0.2)       
    last_t = now

    gaze = None
    if ev is not None and not bool(getattr(ev, "blink", False)):
        gaze = correct((float(ev.point[0]), float(ev.point[1])))
        recent.append((now, *gaze))
        pre_sel.append((now, *gaze))
        while recent and now - recent[0][0] > STABLE_WINDOW:
            recent.pop(0)
        while pre_sel and now - pre_sel[0][0] > ADAPT_WINDOW:
            pre_sel.pop(0)
        pts = np.array([r[1:] for r in recent])
        half = len(pts) // 2
        moving = bool(half >= 2 and np.linalg.norm(pts[half:].mean(0) - pts[:half].mean(0)) > MOVE_SHIFT_PX)

    targets = []
    if state == "object":
        targets = [(o["name"], img2scr(*o["img"]), DWELL_OBJECT_S, o) for o in objects]
    elif state == "operation":
        targets = [(b["key"], b["pos"], DWELL_BUTTON_S, b) for b in buttons()]
    elif state == "destination":
        targets = [] if ROBOT_PAGE else [(o["name"], img2scr(*o["img"]), DWELL_OBJECT_S, o) for o in objects
                                         if o["name"] != chosen["name"]]
        if DEST_MODE == "zones":
            zones = dest_zones()
            for z in zones:
                zx, zy = z["img"]
                z["occupied"] = any(o["bbox"][0] <= zx <= o["bbox"][0] + o["bbox"][2] and
                                    o["bbox"][1] <= zy <= o["bbox"][1] + o["bbox"][3] for o in objects)
            targets += [(z["name"], img2scr(*z["img"]), DWELL_SPOT_S, z) for z in zones if not z["occupied"]]
        cx, cy = CANCEL_POS
        targets.append(("cancel", (cx, cy), DWELL_BUTTON_S, {"key": "cancel"}))

    on = None
    if gaze is not None and not moving and targets and state != "done":
        dists = [gaze_distance(gaze, p, payload) for _, p, _, payload in targets]
        best = int(np.argmin(dists))
        ids = [t[0] for t in targets]
        if last_on_id in ids:
            li = ids.index(last_on_id)
            if dists[li] - dists[best] < HYSTERESIS_PX:
                best = li
        if math.exp(-dists[best] ** 2 / (2 * SIGMA ** 2)) >= ON_INPUT:
            on = best
    last_on_id = targets[on][0] if on is not None else None

    active_ids = {t[0] for t in targets}
    for k in list(dwell):
        if k not in active_ids:
            del dwell[k]
    if now < grace_until:
        on = None
    for i, (tid, pos, need, payload) in enumerate(targets):
        if i == on:
            dwell[tid] = dwell.get(tid, 0.0) + dt
        else:
            dwell[tid] = max(0.0, dwell.get(tid, 0.0) - DECAY * dt)

    if DEST_MODE == "free" and state == "destination" and gaze is not None and on is None and not moving \
            and now >= grace_until and math.dist(gaze, img2scr(*chosen["img"])) > 0.6 * SPOT_RADIUS:
        ix, iy = scr2img(*gaze)
        inside = 0 <= ix < view["fw"] and 0 <= iy < view["fh"] and in_work_area(ix, iy, view["fw"], view["fh"])
        if inside:
            if spot_anchor is None or math.dist(gaze, spot_anchor) > SPOT_RADIUS:
                spot_anchor, spot_dwell, spot_pts = gaze, 0.0, []
            spot_pts.append(gaze)
            spot_anchor = tuple(np.median(np.array(spot_pts), axis=0))
            spot_dwell += dt
    elif DEST_MODE == "free" and state == "destination":
        spot_dwell = max(0.0, spot_dwell - DECAY * dt)

    if state == "done" and now >= state_until and not (ROBOT_PAGE and scene.pending()):
        reset_flow()
    if on is not None:
        tid, pos, need, payload = targets[on]
        if dwell.get(tid, 0) >= need:
            if state == "object":
                adapt_to(pos)
            dwell.clear()
            grace_until = now + STEP_GRACE_S
            if state == "object":
                chosen, state = dict(payload), "operation"
                print(f"OBJECT: {chosen['name']} at scene pixel {tuple(round(v) for v in chosen['img'])}")
                if ROBOT_PAGE:
                    operation = OPERATIONS[0]
                    scene.go(*chosen["img"], target=ROBOT_TARGET, label=f"{chosen['name']} (A)")
                    state, spot_anchor, spot_dwell, spot_pts = "destination", None, 0.0, []
            elif state == "operation":
                if tid == "cancel":
                    reset_flow()
                else:
                    operation = next(op for op in OPERATIONS if op[0] == tid)
                    if operation[2]:
                        state, spot_anchor, spot_dwell = "destination", None, 0.0
                    else:
                        finish_command(now)
            elif state == "destination":
                if tid == "cancel":
                    reset_flow()
                else:
                    dest = dict(payload)
                    finish_command(now)
    if state == "destination" and spot_anchor is not None and spot_dwell >= DWELL_SPOT_S:
        sx, sy = scr2img(*spot_anchor)
        dest = {"name": "spot", "img": (sx, sy)}
        finish_command(now)

    screen.fill(BG)
    if last_frame is not None:
        screen.blit(frame_surface(last_frame, dim=(state == "operation")), (view["ox"], view["oy"]))
    else:
        text_center(f"No picture from the robot page {ROBOT_PAGE}: {scene.status} {scene.error[:80]}" if ROBOT_PAGE else
                    f"No image from scene camera {SCENE_CAM_INDEX} - check SCENE_CAM_INDEX", H / 2, font, RED)

    for i, (tid, pos, need, payload) in enumerate(targets):
        if state in ("object", "destination") and "bbox" in payload:
            x, y, w, h = payload["bbox"]
            x0, y0 = img2scr(x, y)
            rect = pygame.Rect(x0, y0, w * view["s"], h * view["s"])
            pygame.draw.rect(screen, payload["draw"], rect, 3, border_radius=8)
            draw_ring(pos, dwell.get(tid, 0) / need, radius=int(min(max(40, min(rect.w, rect.h) * 0.45), 80)))
            lab = font.render(payload["name"], True, FG)
            screen.blit(lab, lab.get_rect(center=(pos[0], rect.bottom + 22)))
    if chosen is not None and state != "object":
        p = img2scr(*chosen["img"])
        pygame.draw.circle(screen, ORANGE, p, 60, 5)
        tag = font.render(f"1  {chosen['name']}", True, ORANGE)
        screen.blit(tag, tag.get_rect(center=(p[0], p[1] - 85)))
    if state == "operation":
        bl = buttons()
        label_font = big if all(big.size(x["label"])[0] <= x["rect"].w - 24 for x in bl) else font
        for b in bl:
            frac = dwell.get(b["key"], 0) / DWELL_BUTTON_S
            pygame.draw.rect(screen, (50, 50, 50), b["rect"], border_radius=14)
            if frac > 0:
                fill = b["rect"].copy()
                fill.w = int(fill.w * min(frac, 1))
                pygame.draw.rect(screen, (70, 110, 160), fill, border_radius=14)
            pygame.draw.rect(screen, RED if b["key"] == "cancel" else FG, b["rect"], 3, border_radius=14)
            lab = label_font.render(b["label"], True, FG)
            screen.blit(lab, lab.get_rect(center=b["rect"].center))
    if state == "destination" and DEST_MODE == "zones":
        for z in zones:
            if z["occupied"]:
                poly = [img2scr(*p) for p in z["poly_img"]]
                pygame.draw.polygon(screen, (90, 90, 90), poly, 1)
        for tid, pos, need, payload in targets:
            if "poly_img" in payload:
                poly = [img2scr(*p) for p in payload["poly_img"]]
                frac = dwell.get(tid, 0) / need
                pygame.draw.polygon(screen, GREEN if frac > 0.02 else (200, 200, 200), poly, 2)
                if frac > 0.02:
                    draw_ring(pos, frac, radius=35, color=GREEN)
                lab = font.render(tid.replace("zone ", ""), True, (220, 220, 220))
                screen.blit(lab, lab.get_rect(center=(poly[0][0] + 22, poly[0][1] + 18)))
    if state == "destination":
        pygame.draw.rect(screen, RED, pygame.Rect(CANCEL_POS[0] - 100, CANCEL_POS[1] - 40, 200, 80), 3,
                         border_radius=12)
        lab = font.render("Cancel", True, RED)
        screen.blit(lab, lab.get_rect(center=CANCEL_POS))
        if spot_anchor is not None and spot_dwell > 0.05:
            draw_ring(spot_anchor, spot_dwell / DWELL_SPOT_S, radius=SPOT_RADIUS // 2, color=GREEN)
    if state == "done":
        a = img2scr(*chosen["img"])
        if dest:
            b = img2scr(*dest["img"])
            pygame.draw.line(screen, GREEN, a, b, 6)
            pygame.draw.circle(screen, GREEN, b, 40, 5)
        pygame.draw.circle(screen, GREEN, a, 60, 6)
    if ROBOT_PAGE:
        tip = (scene.drive.get("real" if ROBOT_TARGET == "real" else "sim") or {}).get("tip_px")
        if tip:
            tx, ty = img2scr(*tip)
            pygame.draw.circle(screen, (255, 80, 200), (tx, ty), 18, 3)
            pygame.draw.line(screen, (255, 80, 200), (tx - 26, ty), (tx + 26, ty), 2)
            pygame.draw.line(screen, (255, 80, 200), (tx, ty - 26), (tx, ty + 26), 2)
            screen.blit(font.render("arm", True, (255, 80, 200)), (tx + 22, ty - 34))
    if show_gaze and gaze:
        pygame.draw.circle(screen, DIM if moving else ORANGE, gaze, 12, 3)

    if state == "object":
        banner, col = f"1) Look at an object to choose it ({DWELL_OBJECT_S:.0f} s)", FG
        if ROBOT_PAGE and time.time() < robot_flash[1]:
            banner, col = robot_flash[0], RED
    elif state == "operation":
        banner, col = f"2) What should the arm do with the {chosen['name']}? Look at a button", ORANGE
    elif state == "destination" and ROBOT_PAGE:
        banner, col = f"2) The arm goes above the {chosen['name']}. Now look at the place B ({DWELL_SPOT_S:.0f} s)", ORANGE
    elif state == "destination":
        banner, col = (f"3) Look at the destination: another object, or a zone "
                        f"(A1-{chr(64 + DEST_ZONES[1])}{DEST_ZONES[0]})" if DEST_MODE == "zones"
                       else "3) Look at the destination: another object, or a free spot on the table"), ORANGE
    else:
        banner = f"Sent: {operation[1]} - {chosen['name']}" + (f" -> {dest['name']}" if dest else "")
        col = GREEN
        if ROBOT_PAGE:
            res = scene.last_result
            banner, col = (("Sending to the robot page ...", ORANGE) if res is None else
                           (res[1], GREEN) if res[0] else (f"Refused: {res[1]}", RED))
    if ev is None:
        banner, col = "No face detected (eye camera)", RED
    text_center(banner, H - 40, big if state == "done" else font, col)
    if ROBOT_PAGE:
        table_txt = (f"robot page: {scene.status}" + (f" ({scene.error[:70]})" if scene.error else "")
                     + f"   |   moves: {'REAL ARM' if ROBOT_TARGET == 'real' else 'simulator'}   |   S = stop")
    else:
        table_txt = (f"table: {table_source}" if table_H is not None
                     else "work area: default left half (A = find black lines, T = click)")
    bg_txt = ("" if DETECTOR != "blobs" else
              "   |   empty table: " + ("saved (B)" if background["img"] is not None else "optional, B"))
    status = font.render(f"{len(objects)} objects ({DETECTOR}, {detector.fps:.0f}/s)   |   {table_txt}{bg_txt}",
                         True, GREEN if table_H is not None or (ROBOT_PAGE and scene.status == "live") else DIM)
    screen.blit(status, (20, 15))
    pygame.display.flip()
    clock.tick(60)
