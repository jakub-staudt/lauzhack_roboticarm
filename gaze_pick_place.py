import math
import os
import random
import sys
import time
import logging

logging.basicConfig(level=logging.INFO)

import cv2
import numpy as np
import pygame

from eyeGestures import EyeGestures_v3
from eyeGestures.utils import VideoCapture


# Constants and parameters
CAM_INDEX = 0             
ROTATE_180 = False

HERE = os.path.dirname(os.path.abspath(__file__))
CALIB_FILE = os.path.join(HERE, "eg_calibration.pkl")
HEATMAP_FILE = os.path.join(HERE, "gaze_heatmap.png")

CAL_GRID = [0.05, 0.275, 0.5, 0.725, 0.95]
CAL_SETTLE = 0.8          
CAL_COLLECT = 1.2         

SIGMA = 230              
ON_INPUT = 0.45            
DWELL_S = 3.0         
DECAY = 1.5               
THRESHOLD = DWELL_S         
PAUSE_AFTER = 0.8             
SHOW_DONE = 3.0            
STABLE_WINDOW = 0.25       
STABLE_DISPERSION = 200     

ADAPT = True
ADAPT_RATE = 0.3             
ADAPT_MAX_STEP = 120         
ADAPT_WINDOW = 0.6                     
CTX = "pickplace"

ROBOT_X_RANGE_CM = (-20.0, 20.0)
ROBOT_Y_RANGE_CM = (35.0, 15.0)

BG, FG, DIM = (30, 30, 30), (235, 235, 235), (110, 110, 110)
BLUE, ORANGE, RED, GREEN = (80, 140, 255), (255, 170, 60), (255, 90, 90), (90, 220, 120)
OBJECT_TYPES = [("cup", (230, 120, 80)), ("ball", (90, 200, 120)),
                ("box", (110, 150, 255)), ("star", (240, 210, 70))]


def screen_to_robot(x, y):
    """Linear map from screen pixels to arm coordinates in cm."""
    rx = ROBOT_X_RANGE_CM[0] + (x / W) * (ROBOT_X_RANGE_CM[1] - ROBOT_X_RANGE_CM[0])
    ry = ROBOT_Y_RANGE_CM[0] + (y / H) * (ROBOT_Y_RANGE_CM[1] - ROBOT_Y_RANGE_CM[0])
    return round(rx, 1), round(ry, 1)


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
        logging.error(f"(could not save calibration: {e})")


def layout_objects():
    margin, min_gap, center_gap = 150, 380, 260
    center = (W / 2, H / 2)
    for _ in range(3000):
        pts = [(random.randint(margin, W - margin), random.randint(margin + 60, H - margin))
               for _ in OBJECT_TYPES]
        if (all(math.dist(a, b) >= min_gap for i, a in enumerate(pts) for b in pts[i + 1:])
                and all(math.dist(p, center) >= center_gap for p in pts)):
            break
    return [{"name": n, "color": c, "pos": p} for (n, c), p in zip(OBJECT_TYPES, pts)]


def draw_object(o, level, selected):
    "As the first prototype example"
    x, y = o["pos"]
    c, s = o["color"], 55
    if selected:
        pygame.draw.circle(screen, GREEN, (x, y), 105)
    if o["name"] == "cup":
        pygame.draw.rect(screen, c, (x - s * 0.6, y - s * 0.7, s * 1.2, s * 1.4), border_radius=10)
        pygame.draw.circle(screen, c, (x + s * 0.75, y), s * 0.35, 8)
    elif o["name"] == "ball":
        pygame.draw.circle(screen, c, (x, y), s * 0.8)
    elif o["name"] == "box":
        pygame.draw.rect(screen, c, (x - s * 0.75, y - s * 0.75, s * 1.5, s * 1.5))
    else:
        pts = [(x + s * (0.9 if k % 2 == 0 else 0.4) * math.sin(k * math.pi / 5),
                y - s * (0.9 if k % 2 == 0 else 0.4) * math.cos(k * math.pi / 5)) for k in range(10)]
        pygame.draw.polygon(screen, c, pts)
    r = 95
    pygame.draw.circle(screen, (60, 60, 60), (x, y), r, 6)
    frac = min(level / THRESHOLD, 1.0)
    if frac > 0.01:
        rect = pygame.Rect(x - r, y - r, 2 * r, 2 * r)
        pygame.draw.arc(screen, BLUE, rect, math.pi / 2 - 2 * math.pi * frac, math.pi / 2, 8)
    label = font.render(o["name"], True, BG if selected else FG)
    screen.blit(label, label.get_rect(center=(x, y + s + 15)))


selections = [] 
commands = []
heat_pts = []   


def heatmap_overlay(base_rgb):
    s = 4
    heat_map= np.zeros((H // s + 1, W // s + 1), np.float32)
    for x, y in heat_pts:
        xi, yi = int(x // s), int(y // s)
        if 0 <= xi < heat_map.shape[1] and 0 <= yi < heat_map.shape[0]:
            heat_map[yi, xi] += 1
    if heat_map.max() == 0:
        return base_rgb
    heat_map = cv2.GaussianBlur(heat_map, (0, 0), sigmaX=45 / s)
    heat_map = (255 * heat_map / heat_map.max()).astype(np.uint8)
    heat_map = cv2.resize(heat_map, (W, H))
    color = cv2.cvtColor(cv2.applyColorMap(heat_map, cv2.COLORMAP_JET), cv2.COLOR_BGR2RGB)
    alpha = (heat_map.astype(np.float32) / 255 * 0.7)[..., None]
    return (base_rgb * (1 - alpha) + color * alpha).astype(np.uint8)


def screenshot_objects():
    screen.fill(BG)
    for o in globals().get("objects", []):
        draw_object(o, 0, False)
    return np.transpose(pygame.surfarray.array3d(screen), (1, 0, 2)).copy()


def save_heatmap():
    # gaze correction: x' = a*x + b,  y' = c*y + d
    if not heat_pts or "objects" not in globals():
        return
    img = heatmap_overlay(screenshot_objects())
    cv2.imwrite(HEATMAP_FILE, cv2.cvtColor(img, cv2.COLOR_RGB2BGR))
    logging.info(f"Heatmap of {len(heat_pts)} gaze points saved to {HEATMAP_FILE}")
corr = np.array([1.0, 0.0, 1.0, 0.0]) 


def correct(raw):
    return (corr[0] * raw[0] + corr[1], corr[2] * raw[1] + corr[3])


def drift_correct():
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
        logging.info("Gaze correction skipped. There is not enough tracking data")
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
    logging.info(f"Gaze correction: x' = {corr[0]:.2f}*x {corr[1]:+.0f},  y' = {corr[2]:.2f}*y {corr[3]:+.0f}")


def finish():
    try:
        save_heatmap()
    except Exception as e:
        logging.error(f"(could not save heatmap: {e})")
    pygame.quit()
    try:
        cap.close()
    except Exception:
        pass
    logging.info("\n# All pick-and-place commands this session as ((pick_x, pick_y), (place_x, place_y)):")
    logging.info(f"commands = {commands}")
    sys.exit()

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

message([f"1) Look at the object to PICK for {DWELL_S:.0f} seconds.",
         f"2) Then look at where to PLACE it for {DWELL_S:.0f} seconds.",
         "First look at the 5 crosses that appear one by one (gaze correction).",
         "U = undo pick  D = correction  H = heatmap  R = layout  G = gaze dot  ESC = quit",
         "Press SPACE to start"])

objects = layout_objects()
dwell = np.zeros(len(objects))
show_gaze = True
last_t = t_start = time.time()
state, pick_idx, place_idx = "pick", None, None
state_until = 0.0
recent, pre_sel = [], []
moving = False
show_heat, heat_surface, heat_time = False, None, 0.0
drift_correct()


def adapt_to(obj):
    if ADAPT and len(pre_sel) >= 5:
        seen = np.median(np.array([p[1:] for p in pre_sel]), axis=0)
        step = np.clip(ADAPT_RATE * (np.array(obj["pos"]) - seen), -ADAPT_MAX_STEP, ADAPT_MAX_STEP)
        corr[1] += step[0]
        corr[3] += step[1]


def reset_flow():
    global state, pick_idx, place_idx
    state, pick_idx, place_idx = "pick", None, None
    dwell[:] = 0


while True:
    keys = poll_keys()
    if pygame.K_r in keys:
        save_heatmap()
        heat_pts.clear()
        objects = layout_objects()
        dwell = np.zeros(len(objects))
        reset_flow()
    if pygame.K_u in keys or pygame.K_BACKSPACE in keys:
        reset_flow()
        logging.info("(pick undone)")
    if pygame.K_g in keys:
        show_gaze = not show_gaze
    if pygame.K_h in keys:
        show_heat, heat_surface, heat_time = not show_heat, None, 0.0
    if pygame.K_d in keys:
        drift_correct()
        dwell[:] = 0
        pre_sel.clear()
    if pygame.K_k in keys:
        calibrate()
        drift_correct()
        reset_flow()

    ev, _ = get_event(False)
    now = time.time()
    dt = min(now - last_t, 0.1)
    last_t = now

    gaze, on = None, None
    if ev is not None and not bool(getattr(ev, "blink", False)):
        gaze = correct((float(ev.point[0]), float(ev.point[1])))
        heat_pts.append(gaze)
        recent.append((now, *gaze))
        pre_sel.append((now, *gaze))
        while recent and now - recent[0][0] > STABLE_WINDOW:
            recent.pop(0)
        while pre_sel and now - pre_sel[0][0] > ADAPT_WINDOW:
            pre_sel.pop(0)
        xs, ys = [r[1] for r in recent], [r[2] for r in recent]
        moving = (max(xs) - min(xs)) + (max(ys) - min(ys)) > STABLE_DISPERSION
        pos = np.array([o["pos"] for o in objects], dtype=float)
        inputs = np.exp(-np.sum((pos - np.array(gaze)) ** 2, axis=1) / (2 * SIGMA ** 2))
        if state == "place" and pick_idx is not None:
            inputs[pick_idx] = 0.0               
        best = int(inputs.argmax())
        if inputs[best] >= ON_INPUT and not moving:
            on = best

    if state == "done" and now >= state_until:
        reset_flow()
    counting = state in ("pick", "place") and now >= state_until
    for i in range(len(objects)):
        if counting and i == on:
            dwell[i] += dt
        else:
            dwell[i] = max(0.0, dwell[i] - DECAY * dt)

    if counting and on is not None and dwell[on] >= DWELL_S:
        o = objects[on]
        selections.append((o["name"], o["pos"][0], o["pos"][1], round(now - t_start, 2)))
        adapt_to(o)
        dwell[:] = 0
        if state == "pick":
            pick_idx, state, state_until = on, "place", now + PAUSE_AFTER
            logging.info(f"PICK : {o['name']:<5} screen {o['pos']}  ->  robot {screen_to_robot(*o['pos'])} cm")
        else:
            place_idx, state, state_until = on, "done", now + SHOW_DONE
            p, q = objects[pick_idx], o
            logging.info(f"PLACE: {q['name']:<5} screen {q['pos']}  ->  robot {screen_to_robot(*q['pos'])} cm")
            command = [tuple(p["pos"]), tuple(q["pos"])]
            commands.append(tuple(command))
            logging.info(f"command = {command}")
            logging.info("ARM_ENABLED is False: not sent")

    if show_heat and now - heat_time > 0.5:
        heat_surface = pygame.surfarray.make_surface(
            np.transpose(heatmap_overlay(screenshot_objects()), (1, 0, 2)))
        heat_time = now
    screen.fill(BG)
    if show_heat and heat_surface is not None:
        screen.blit(heat_surface, (0, 0))
    if state == "done":
        a, b = objects[pick_idx]["pos"], objects[place_idx]["pos"]
        pygame.draw.line(screen, GREEN, a, b, 6)
    for i, o in enumerate(objects):
        draw_object(o, dwell[i], state == "done" and i in (pick_idx, place_idx))
        if i == pick_idx:
            tag = font.render("1  PICK", True, ORANGE)
            screen.blit(tag, tag.get_rect(center=(o["pos"][0], o["pos"][1] - 120)))
        if i == place_idx:
            tag = font.render("2  PLACE", True, GREEN)
            screen.blit(tag, tag.get_rect(center=(o["pos"][0], o["pos"][1] - 120)))
    if show_gaze and gaze:
        pygame.draw.circle(screen, DIM if moving else ORANGE, gaze, 10, 2)
    if state == "pick":
        banner, col = f"Step 1: look at the object to PICK ({DWELL_S:.0f} s)", FG
    elif state == "place":
        banner, col = (f"Step 2: look at where to PLACE the {objects[pick_idx]['name']} "
                       f"({DWELL_S:.0f} s)   U = undo"), ORANGE
    else:
        banner, col = f"Sent: {objects[pick_idx]['name']}  ->  {objects[place_idx]['name']}", GREEN
    if ev is None:
        banner, col = "No face detected", RED
    text_center(banner, H - 45, big if state == "done" else font, col)
    text_center(f"Commands: {len(commands)}   U undo   D correction   H heatmap   R layout   G gaze dot   ESC quit",
                30, font, DIM)
    pygame.display.flip()
    clock.tick(60)
