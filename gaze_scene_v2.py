import json
import math
import os
import urllib.request
from collections import defaultdict
import pickle
import sys
import threading
import time

import cv2
import numpy as np
import pygame


CAM_INDEX = 1              
SCENE_CAM_INDEX = 0         
ROTATE_180 = False            
SCENE_ROTATE_180 = False  
SCENE_SIZE = (1280, 720)
AUTO_AREA_PART = "right"  

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
ADAPT_MAX_STEP = 120
ADAPT_WINDOW = 0.6

DETECTOR = "blobs"

BACKGROUND_FILE = os.path.join(HERE, "table_background.png")
BLOB_THRESHOLD = 28
BLOB_SHADOW_WEIGHT = 0.4
BLOB_MIN_AREA = 1500
BLOB_MAX_FRACTION = 0.25
BLOB_IGNORE_EDGE = True
SCENE_ROI = (0.0, 0.0, 0.5, 1.0)
WORK_INSET_PX = 10
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

TABLE_POINTS_CM = [
    ("top-left corner (top of the middle line)", (0.0, 50.0)),
    ("top-right corner", (20.0, 50.0)),
    ("bottom-right corner", (20.0, 10.0)),
    ("bottom-left corner (bottom of the middle line)", (0.0, 10.0)),
]

AUTO_AREA = True              
AUTO_AREA_PART = "right"     
AUTO_LINE_C = 25             
AUTO_MIN_REGION = 0.02  
TABLE_CALIB_FILE = os.path.join(HERE, "table_calibration.json")

OPERATIONS = [
    ("pick_place", "Pick & place", True),
    ("bring", "Bring to me", False),
    ("push", "Push towards", True),
    ("point", "Point at it", False),
]

ARM_ENABLED = False    
CTX = "scene"

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
    """Replace the body with your arm's interface, e.g. over USB serial:
        arm.write(f"{operation} {obj_cm[0]} {obj_cm[1]} {dest_cm[0] if dest_cm else ''} ...\\n".encode())
    obj_cm / dest_cm are (x, y) in cm in the arm's frame (None if the table isn't calibrated)."""
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
    """Row by row, alternating direction (snake), so each eye movement is short."""
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
        return None, None          # no face in this frame


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
    """Show text until one of the accepted keys is pressed; returns that key."""
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
        with open(CALIB_FILE, "wb") as f:
            f.write(gestures.saveModel(context=CTX))
    except Exception as e:
        print(f"(could not save calibration: {e})")


corr = np.array([1.0, 0.0, 1.0, 0.0]) 


def correct(raw):
    return (corr[0] * raw[0] + corr[1], corr[2] * raw[1] + corr[3])


def drift_correct():
    """Look at 5 crosses. Fixes both a shift AND the typical webcam 'squeeze toward the
    center' (gaze estimates that don't reach the screen edges), per axis."""
    global corr
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
        else:                                           # only a shift
            a, b = 1.0, float(np.mean(true[:, k] - raw[:, k]))
        new += [a, b]
    corr = np.array(new)
    print(f"Gaze correction: x' = {corr[0]:.2f}*x {corr[1]:+.0f},  y' = {corr[2]:.2f}*y {corr[3]:+.0f}")




def finish():
    pygame.quit()
    for c in (cap, scene):
        try:
            c.close()
        except Exception:
            pass
    print("\n# Commands as (operation, object, object_cm, object_px, destination, destination_cm, destination_px)")
    print("# cm = arm coordinates (None if the table isn't calibrated: press T); px = scene camera pixels")
    print(f"commands = {commands}")
    sys.stdout.flush()
    os._exit(0)             


commands = []


class SceneCamera:
    """Reads the scene camera in a background thread; .frame is always the newest image."""

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


scene = SceneCamera(SCENE_CAM_INDEX)
aruco = cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(ARUCO_DICT), cv2.aruco.DetectorParameters())
table_H = None                 
table_source = None         
work_poly = None            
if os.path.exists(TABLE_CALIB_FILE):
    try:
        _tc = json.load(open(TABLE_CALIB_FILE))
        if _tc.get("scene_cam_index") == SCENE_CAM_INDEX:
            table_H, table_source = np.array(_tc["H"], dtype=np.float64), "saved"
            if "clicked_px" in _tc:
                work_poly = np.array(_tc["clicked_px"], np.float32)
            print(f"Loaded table calibration from {TABLE_CALIB_FILE}")
    except Exception as e:
        print(f"(could not load table calibration: {e})")
view = {"s": 1.0, "ox": 0, "oy": 0, "fw": 1, "fh": 1}


def fit_view(fw, fh):
    s = min(W / fw, H / fh)
    view.update(s=s, ox=(W - fw * s) / 2, oy=(H - fh * s) / 2, fw=fw, fh=fh)


def img2scr(x, y):
    return view["ox"] + x * view["s"], view["oy"] + y * view["s"]


def scr2img(x, y):
    return (x - view["ox"]) / view["s"], (y - view["oy"]) / view["s"]


def detect_colors(frame):
    """Largest blob of each color -> object. Positions in scene-image pixels."""
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
    """YOLO11n (80 COCO object types) with ONNX Runtime - no PyTorch needed."""

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
        opts.add_session_config_entry("session.intra_op.allow_spinning", "0")   # don't busy-wait on cores
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


background = {"img": None, "lab": None}    


def _lab_small(frame):
    small = cv2.resize(frame, None, fx=0.5, fy=0.5)
    return cv2.cvtColor(cv2.GaussianBlur(small, (5, 5), 0), cv2.COLOR_BGR2LAB).astype(np.float32)


def set_background(img):
    background["img"], background["lab"] = img, _lab_small(img)


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


def color_name(hsv_pixels):
    h, s, v = hsv_pixels[:, 0], hsv_pixels[:, 1], hsv_pixels[:, 2]
    colorful = s > 70
    if colorful.mean() < 0.35:
        mv = np.median(v)
        return "white" if mv > 190 else "black" if mv < 70 else "gray"
    hue, val = np.median(h[colorful]), np.median(v[colorful])
    if hue < 8 or hue >= 170:
        return "red"
    if hue < 20:
        return "brown" if val < 150 else "orange"
    if hue < 35:
        return "yellow"
    if hue < 85:
        return "green"
    if hue < 130:
        return "blue"
    if hue < 150:
        return "purple"
    return "pink"


def detect_blobs(frame, namer=None):
    """Any object = region that differs from the empty table (or from the table's main color)."""
    lab = _lab_small(frame)
    ref = background["lab"]
    if ref is None or ref.shape != lab.shape:
        ref = np.median(lab.reshape(-1, 3), axis=0)          
    d = lab - ref
    diff = np.sqrt((BLOB_SHADOW_WEIGHT * d[..., 0]) ** 2 + d[..., 1] ** 2 + d[..., 2] ** 2)
    mask = (diff > BLOB_THRESHOLD).astype(np.uint8) * 255
    area = work_area_poly(frame.shape[1], frame.shape[0])
    if area is not None:                                  
        inside = np.zeros_like(mask)
        cv2.fillPoly(inside, [(area * 0.5).astype(np.int32)], 255)
        inside = cv2.erode(inside, np.ones((9, 9), np.uint8))
        mask &= inside
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((9, 9), np.uint8))
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    sh, sw = mask.shape
    hsv = cv2.cvtColor(cv2.resize(frame, (sw, sh)), cv2.COLOR_BGR2HSV)
    found = []
    for c in contours:
        area = cv2.contourArea(c) * 4
        if area < BLOB_MIN_AREA or area > BLOB_MAX_FRACTION * sw * sh * 4:
            continue
        x, y, w, h = cv2.boundingRect(c)
        if BLOB_IGNORE_EDGE and (x <= 1 or y <= 1 or x + w >= sw - 1 or y + h >= sh - 1):
            continue
        m = cv2.moments(c)
        cx, cy = 2 * m["m10"] / m["m00"], 2 * m["m01"] / m["m00"]
        obj_mask = np.zeros_like(mask)
        cv2.drawContours(obj_mask, [c], -1, 255, -1)
        label = f"{color_name(hsv[obj_mask > 0])} object"
        found.append({"cls": "object", "label": label, "draw": class_color(label),
                      "img": (cx, cy), "bbox": (2 * x, 2 * y, 2 * w, 2 * h)})
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
    return lambda frame: detect_blobs(frame, namer)


def class_color(name):
    rng = np.random.default_rng(abs(hash(name)) % (2 ** 32))
    return tuple(int(v) for v in rng.integers(90, 255, 3))


class DetectionWorker:
    """Runs the detector on the newest scene frame in a background thread."""

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
    free = list(range(len(dets)))
    for tr in sorted(tracks, key=lambda t: -t["last"]):
        best, best_d = None, None
        for j in free:
            if dets[j]["cls"] != tr["cls"]:
                continue
            _, _, w, h = dets[j]["bbox"]
            limit = max(TRACK_MAX_JUMP, TRACK_JUMP_FACTOR * math.hypot(w, h))
            d = math.dist(dets[j]["img"], tr["img"])
            if d < limit and (best_d is None or d < best_d):
                best, best_d = j, d
        if best is not None:
            tr.update(img=dets[best]["img"], bbox=dets[best]["bbox"], last=now)
            new_label = dets[best].get("label")
            if new_label and new_label != tr.get("label") and not new_label.endswith(" object") \
                    and tr.get("label", "").endswith(" object") and state == "object":
                used = {t["n"] for t in tracks if t.get("label") == new_label}
                n = next(k for k in range(1, 100) if k not in used)
                tr.update(label=new_label, n=n, name=new_label if n == 1 else f"{new_label} {n}")
            free.remove(best)
    tracks[:] = [t for t in tracks if now - t["last"] < TRACK_KEEP_S]
    for j in free:
        d = dets[j]
        label = d.get("label", d["cls"])
        used = {t["n"] for t in tracks if t["label"] == label}
        n = next(k for k in range(1, 100) if k not in used)
        tracks.append({"name": label if n == 1 else f"{label} {n}", "cls": d["cls"], "label": label, "n": n,
                       "draw": d["draw"], "img": d["img"], "bbox": d["bbox"], "last": now})


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
    json.dump({"scene_cam_index": SCENE_CAM_INDEX, "H": table_H.tolist(),
               "clicked_px": src.tolist(), "points_cm": dst.tolist()}, open(TABLE_CALIB_FILE, "w"), indent=2)
    print(f"Table calibrated by clicks, saved to {TABLE_CALIB_FILE}")


def work_area_poly(fw, fh):
    """The work area as a polygon in scene-image pixels (None = whole image)."""
    if work_poly is not None:
        return work_poly
    if SCENE_ROI is None:
        return None
    l, t, r, b = SCENE_ROI
    return np.array([(l * fw, t * fh), (r * fw, t * fh), (r * fw, b * fh), (l * fw, b * fh)], np.float32)


def in_work_area(x, y, fw, fh):
    poly = work_area_poly(fw, fh)
    if poly is None:
        return True
    return cv2.pointPolygonTest(poly, (float(x), float(y)), True) >= WORK_INSET_PX


def order_corners(q):
    """4 points -> top-left, top-right, bottom-right, bottom-left (image coordinates)."""
    q = np.asarray(q, np.float32)
    s, d = q.sum(1), q[:, 1] - q[:, 0]
    return np.array([q[np.argmin(s)], q[np.argmin(d)], q[np.argmax(s)], q[np.argmax(d)]], np.float32)


def ink_mask(frame, work_w=640):
    """White where the image is clearly darker than its surroundings (drawn black lines). Returns (mask, scale)."""
    h, w = frame.shape[:2]
    s = work_w / w
    small = cv2.resize(frame, (work_w, int(round(h * s))))
    gray = cv2.GaussianBlur(cv2.cvtColor(small, cv2.COLOR_BGR2GRAY), (3, 3), 0)
    lines = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY_INV, 31, AUTO_LINE_C)
    lines = cv2.morphologyEx(lines, cv2.MORPH_CLOSE, np.ones((7, 7), np.uint8))    # bridge small gaps
    return cv2.dilate(lines, np.ones((3, 3), np.uint8)), s



def _fit(points):
    vx, vy, x0, y0 = cv2.fitLine(np.asarray(points, np.float32), cv2.DIST_HUBER, 0, 0.01, 0.01).flatten()
    return np.array([x0, y0]), np.array([vx, vy])

def _intersect(l1, l2):
    (p, r), (q, s) = l1, l2
    den = r[0] * s[1] - r[1] * s[0]
    if abs(den) < 1e-6:
        return None
    t = ((q[0] - p[0]) * s[1] - (q[1] - p[1]) * s[0]) / den
    return p + t * r

def _clusters(segs, key, gap):
    """Group segments whose position (key) is close; return list of point lists, sorted by key."""
    segs = sorted(segs, key=key)
    groups, cur, last = [], [], None
    for sg in segs:
        k = key(sg)
        if last is not None and k - last > gap:
            groups.append(cur); cur = []
        cur.append(sg); last = k
    if cur:
        groups.append(cur)
    return [([(s[0], s[1]) for s in g] + [(s[2], s[3]) for s in g], sum(np.hypot(s[2]-s[0], s[3]-s[1]) for s in g))
            for g in groups]

def _frame_from_group(mask, paper, bbox, part):
    x, y, w, h = bbox
    size = min(w, h)
    segs = cv2.HoughLinesP(paper, 1, np.pi / 180, threshold=max(20, int(0.1 * size)),
                           minLineLength=int(0.15 * size), maxLineGap=int(0.08 * size))
    if segs is None:
        return None
    segs = [tuple(map(float, s)) for s in segs[:, 0]]
    vert = [s for s in segs if abs(s[3] - s[1]) > abs(s[2] - s[0])]
    horiz = [s for s in segs if abs(s[3] - s[1]) <= abs(s[2] - s[0])]

    def keep_parallel(family, vertical):
        """Drop stray strokes: keep segments within 15 degrees of the family's typical direction."""
        if not family:
            return family
        ang = [np.degrees(np.arctan2(s[3] - s[1], s[2] - s[0])) for s in family]
        if vertical:
            ang = [a % 180 for a in ang]                       # 0..180, vertical ~ 90
        else:
            ang = [((a + 90) % 180) - 90 for a in ang]         # -90..90, horizontal ~ 0
        lens = [np.hypot(s[2] - s[0], s[3] - s[1]) for s in family]
        order = np.argsort(ang)
        cum = np.cumsum(np.array(lens)[order])
        med = np.array(ang)[order][np.searchsorted(cum, cum[-1] / 2)]   # length-weighted median
        return [s for s, a in zip(family, ang) if abs(a - med) <= 15]
    vert, horiz = keep_parallel(vert, True), keep_parallel(horiz, False)
    cy, cx = y + h / 2, x + w / 2
    def x_at_mid(s):
        x1, y1, x2, y2 = s
        return x1 + (x2 - x1) * (cy - y1) / (y2 - y1) if y2 != y1 else x1
    def y_at_mid(s):
        x1, y1, x2, y2 = s
        return y1 + (y2 - y1) * (cx - x1) / (x2 - x1) if x2 != x1 else y1
    vg = [g for g in _clusters(vert, x_at_mid, 0.06 * w) if g[1] > 0.3 * h]    
    hg = [g for g in _clusters(horiz, y_at_mid, 0.06 * h) if g[1] > 0.3 * w]
    if len(vg) < 2 or len(hg) < 2:
        return None
    if (part in ("left", "right") and len(vg) < 3) or (part in ("top", "bottom") and len(hg) < 3):
        return None                    
    V = [_fit(g[0]) for g in vg]
    Hl = [_fit(g[0]) for g in hg]
    if part == "left":
        sides, ends = (V[0], V[1]), (Hl[0], Hl[-1])
    elif part == "right":
        sides, ends = (V[-2], V[-1]), (Hl[0], Hl[-1])
    elif part == "top":
        sides, ends = (V[0], V[-1]), (Hl[0], Hl[1])
    elif part == "bottom":
        sides, ends = (V[0], V[-1]), (Hl[-2], Hl[-1])
    else:
        sides, ends = (V[0], V[-1]), (Hl[0], Hl[-1])
    pts = [_intersect(ends[0], sides[0]), _intersect(ends[0], sides[1]),
           _intersect(ends[1], sides[1]), _intersect(ends[1], sides[0])]
    if any(p is None for p in pts):
        return None
    return np.array(pts, np.float32), len(vg), len(hg)


def _valid(quad, mask):
    """A real work area: convex, big enough, and its inside mostly free of ink (empty paper)."""
    H, W = mask.shape
    q = quad.astype(np.float32)
    if not cv2.isContourConvex(q.reshape(-1, 1, 2)):
        return False
    if cv2.contourArea(q) < 0.02 * W * H:
        return False
    if min(np.linalg.norm(q[i] - q[(i + 1) % 4]) for i in range(4)) < 0.08 * min(W, H):
        return False
    if q[:, 0].min() < -0.05 * W or q[:, 0].max() > 1.05 * W or q[:, 1].min() < -0.05 * H or q[:, 1].max() > 1.05 * H:
        return False                                          # corners must be (about) inside the image
    near = cv2.dilate(mask, np.ones((7, 7), np.uint8))
    support = []
    for i in range(4):                                        # every edge must be a drawn (maybe dashed) line
        a_, b_ = q[i], q[(i + 1) % 4]
        t = np.linspace(0.1, 0.9, 40)[:, None]
        pts = (a_ + t * (b_ - a_)).astype(int)
        pts[:, 0] = pts[:, 0].clip(0, W - 1)
        pts[:, 1] = pts[:, 1].clip(0, H - 1)
        support.append((near[pts[:, 1], pts[:, 0]] > 0).mean())
    if min(support) < 0.35 or np.mean(support) < 0.55:
        return False
    c = q.mean(0)
    inner = (c + 0.8 * (q - c)).astype(np.int32)              # shrink 20 % to stay off the lines
    m = np.zeros_like(mask)
    cv2.fillPoly(m, [inner], 255)
    inside = m > 0
    return inside.sum() > 0 and (mask[inside] > 0).mean() < 0.12


def paper_part_lines(mask, part="left"):
    """mask: white = black ink. Find the paper's border + divider as straight lines (Hough) and return
    (4 corners TL, TR, BR, BL of the requested part, n vertical, n horizontal) in mask pixels, or None."""
    H, W = mask.shape
    k = max(5, int(0.012 * W)) | 1
    grouped = cv2.dilate(mask, np.ones((k, k), np.uint8))       # join line pieces separated by small gaps
    n, lab, stats, _ = cv2.connectedComponentsWithStats(grouped, 8)
    cands = [i for i in range(1, n) if stats[i][2] > 0.1 * W and stats[i][3] > 0.1 * H]
    for i in sorted(cands, key=lambda i: -stats[i][2] * stats[i][3]):   # biggest line groups first
        paper = ((lab == i) & (mask > 0)).astype(np.uint8) * 255
        res = _frame_from_group(mask, paper, stats[i][:4], part)
        if res is not None and _valid(res[0], mask):
            return res
    return None


def find_line_regions(frame, work_w=640):
    """Fallback: regions completely enclosed by dark drawn lines, each as 4 corners."""
    lines, s = ink_mask(frame, work_w)
    n, lab, stats, cents = cv2.connectedComponentsWithStats(cv2.bitwise_not(lines), 4)
    sh, sw = lines.shape
    regions = []
    for i in range(1, n):
        x, y, ww, hh, area = stats[i]
        if x <= 0 or y <= 0 or x + ww >= sw or y + hh >= sh or area < AUTO_MIN_REGION * sw * sh:
            continue                               
        cnts, _ = cv2.findContours((lab == i).astype(np.uint8) * 255, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        c = max(cnts, key=cv2.contourArea)
        hull = cv2.convexHull(c)
        peri, quad = cv2.arcLength(hull, True), None
        for eps in np.linspace(0.01, 0.12, 23):
            ap = cv2.approxPolyDP(hull, eps * peri, True)
            if len(ap) == 4:
                quad = ap.reshape(4, 2)
                break
        if quad is None:
            quad = cv2.boxPoints(cv2.minAreaRect(c))
        regions.append({"quad": order_corners(quad) / s, "center": (cents[i][0] / s, cents[i][1] / s),
                        "area": area / (s * s)})
    return regions


def pick_region(regions):
    key = {"left": lambda r: r["center"][0], "right": lambda r: -r["center"][0],
           "top": lambda r: r["center"][1], "bottom": lambda r: -r["center"][1],
           "largest": lambda r: -r["area"]}[AUTO_AREA_PART]
    return min(regions, key=key)


def auto_table_calibration(preview_s=1.5):
    """Find the regions enclosed by black lines, use the chosen one as work area + table calibration."""
    global table_H, table_source, work_poly
    frame, t0 = None, time.time()
    while frame is None and time.time() - t0 < 3:
        frame = scene.read()
        time.sleep(0.05)
    if frame is None:
        print("No scene image - cannot look for the black lines.")
        return False
    mask, sc = ink_mask(frame)
    found = paper_part_lines(mask, AUTO_AREA_PART)        
    if found is not None:
        quad = found[0] / sc
        regions = [{"quad": quad}]
        chosen = regions[0]
        method = f"lines: {found[1]} vertical, {found[2]} horizontal"
    else:                                                    
        regions = find_line_regions(frame)
        if not regions:
            print("Black lines not found - check that the border and the middle line are in view "
                  "(run examples/detect_area.py to see what the camera sees), or press T and click the corners.")
            return False
        chosen = pick_region(regions)
        quad = chosen["quad"]
        method = f"enclosed regions: {len(regions)}"
    table_H, _ = cv2.findHomography(quad, np.array([cm for _, cm in TABLE_POINTS_CM], np.float32))
    table_source, work_poly = "black lines", quad
    json.dump({"scene_cam_index": SCENE_CAM_INDEX, "H": table_H.tolist(), "clicked_px": quad.tolist(),
               "points_cm": [list(cm) for _, cm in TABLE_POINTS_CM], "source": "black lines"},
              open(TABLE_CALIB_FILE, "w"), indent=2)
    print(f"Work area found from black lines ({method}), using the {AUTO_AREA_PART} part, "
          f"corners {np.round(quad).astype(int).tolist()}")
    fit_view(frame.shape[1], frame.shape[0])
    surf = frame_surface(frame)
    t0 = time.time()
    while time.time() - t0 < preview_s:
        pygame.event.pump()
        screen.fill(BG)
        screen.blit(surf, (view["ox"], view["oy"]))
        for r in regions:
            pts = [img2scr(*p) for p in r["quad"]]
            pygame.draw.polygon(screen, GREEN if r is chosen else (150, 150, 150), pts, 5 if r is chosen else 2)
        text_center(f"Work area: the {AUTO_AREA_PART} region enclosed by black lines", 40, font, GREEN)
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
    """Operation buttons in a column on the right, plus Cancel."""
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
    """Drop zones in scene-image coordinates. On the calibrated table area if available,
    otherwise on the middle 70 % of the camera image. Rows A (far) .. C (near), columns 1..3."""
    cols, rows = DEST_ZONES
    if table_H is not None:
        xs = [p[0] for _, p in TABLE_POINTS_CM]
        ys = [p[1] for _, p in TABLE_POINTS_CM]
        inv = np.linalg.inv(table_H)

        def at(u, v):          # u: left -> right, v: far -> near, in cm space
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
    size = (int(view["fw"] * view["s"]), int(view["fh"] * view["s"]))
    rgb = cv2.resize(rgb, size)
    return pygame.image.frombuffer(rgb.tobytes(), size, "RGB")


if os.path.exists(CALIB_FILE):
    key = message(["Calibration found", "L = load the saved calibration (same seat, same camera position)",
                   "K = calibrate again"], accept=(pygame.K_l, pygame.K_k))
    if key == pygame.K_l:
        with open(CALIB_FILE, "rb") as f:
            gestures.loadModel(f.read(), context=CTX)
    else:
        calibrate()
else:
    calibrate()

if AUTO_AREA and table_source != "saved":
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
spot_anchor, spot_dwell = None, 0.0
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


def adapt_to(pos):
    if ADAPT and len(pre_sel) >= 5:
        seen = np.median(np.array([p[1:] for p in pre_sel]), axis=0)
        step = np.clip(ADAPT_RATE * (np.array(pos) - seen), -ADAPT_MAX_STEP, ADAPT_MAX_STEP)
        corr[1] += step[0]
        corr[3] += step[1]


def reset_flow():
    global state, chosen, operation, dest, spot_anchor, spot_dwell
    state, chosen, operation, dest = "object", None, None, None
    spot_anchor, spot_dwell = None, 0.0
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
    if obj_cm is None:
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
    if pygame.K_t in keys:
        click_table_calibration()
        reset_flow()
    if pygame.K_a in keys:
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
        fit_view(frame.shape[1], frame.shape[0])
        markers_seen = update_table(frame)
    dets, stamp = detector.get()
    if stamp != last_stamp:
        last_stamp = stamp
        dets = [d for d in dets if in_work_area(*d["img"], view["fw"], view["fh"])]
        update_tracks(dets, stamp)
    objects = [t for t in tracks if time.time() - t["last"] < TRACK_KEEP_S]

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
        targets = [(o["name"], img2scr(*o["img"]), DWELL_OBJECT_S, o) for o in objects
                   if o["name"] != chosen["name"]]
        if DEST_MODE == "zones":
            zones = dest_zones()
            for z in zones:                            # a zone with an object standing in it is occupied
                zx, zy = z["img"]
                z["occupied"] = any(o["bbox"][0] <= zx <= o["bbox"][0] + o["bbox"][2] and
                                    o["bbox"][1] <= zy <= o["bbox"][1] + o["bbox"][3] for o in objects)
            targets += [(z["name"], img2scr(*z["img"]), DWELL_SPOT_S, z) for z in zones if not z["occupied"]]
        cx, cy = CANCEL_POS
        targets.append(("cancel", (cx, cy), DWELL_BUTTON_S, {"key": "cancel"}))

    on = None
    if gaze is not None and not moving and targets and state != "done":
        dists = [gaze_distance(gaze, p, payload) for _, p, _, payload in targets]
        best = int(np.argmin(dists))                # the closest target wins...
        ids = [t[0] for t in targets]
        if last_on_id in ids:                       # ...unless the current one is nearly as close
            li = ids.index(last_on_id)
            if dists[li] - dists[best] < HYSTERESIS_PX:
                best = li
        if math.exp(-dists[best] ** 2 / (2 * SIGMA ** 2)) >= ON_INPUT:   # close enough to count at all
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

    if DEST_MODE == "free" and state == "destination" and gaze is not None and on is None and not moving:
        ix, iy = scr2img(*gaze)
        inside = 0 <= ix < view["fw"] and 0 <= iy < view["fh"]
        if inside:
            if spot_anchor is None or math.dist(gaze, spot_anchor) > SPOT_RADIUS:
                spot_anchor, spot_dwell = gaze, 0.0
            spot_dwell += dt
    elif DEST_MODE == "free" and state == "destination":
        spot_dwell = max(0.0, spot_dwell - DECAY * dt)

    if state == "done" and now >= state_until:
        reset_flow()
    if on is not None:
        tid, pos, need, payload = targets[on]
        if dwell.get(tid, 0) >= need:
            if "zone" not in payload:
                adapt_to(pos)
            dwell.clear()
            grace_until = now + STEP_GRACE_S
            if state == "object":
                chosen, state = dict(payload), "operation"
                print(f"OBJECT: {chosen['name']} at scene pixel {tuple(round(v) for v in chosen['img'])}")
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
        adapt_to(spot_anchor)
        finish_command(now)

    screen.fill(BG)
    if last_frame is not None:
        screen.blit(frame_surface(last_frame, dim=(state == "operation")), (view["ox"], view["oy"]))
    else:
        text_center(f"No image from scene camera {SCENE_CAM_INDEX} - check SCENE_CAM_INDEX", H / 2, font, RED)

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
    if show_gaze and gaze:
        pygame.draw.circle(screen, DIM if moving else ORANGE, gaze, 12, 3)

    if state == "object":
        banner, col = f"1) Look at an object to choose it ({DWELL_OBJECT_S:.0f} s)", FG
    elif state == "operation":
        banner, col = f"2) What should the arm do with the {chosen['name']}? Look at a button", ORANGE
    elif state == "destination":
        banner, col = (f"3) Look at the destination: another object, or a zone "
                        f"(A1-{chr(64 + DEST_ZONES[1])}{DEST_ZONES[0]})" if DEST_MODE == "zones"
                       else "3) Look at the destination: another object, or a free spot on the table"), ORANGE
    else:
        banner = f"Sent: {operation[1]} - {chosen['name']}" + (f" -> {dest['name']}" if dest else "")
        col = GREEN
    if ev is None:
        banner, col = "No face detected (eye camera)", RED
    text_center(banner, H - 40, big if state == "done" else font, col)
    table_txt = (f"table: {table_source}" if table_H is not None
                 else "work area: default left half (A = find black lines, T = click)")
    bg_txt = ("" if DETECTOR != "blobs" else
              "   |   empty table: " + ("saved (B)" if background["lab"] is not None else "press B"))
    status = font.render(f"{len(objects)} objects ({DETECTOR}, {detector.fps:.0f}/s)   |   {table_txt}{bg_txt}",
                         True, GREEN if table_H is not None else DIM)
    screen.blit(status, (20, 15))
    pygame.display.flip()
    clock.tick(60)
