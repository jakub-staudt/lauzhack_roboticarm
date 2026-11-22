"""Find the coloured blocks in the overhead picture (plain colour segmentation, no learning).

For every block inside the locked rectangle it gives the colour, the centre of the block's top face (picture pixels and table cm), and
its rotation on the table. The blocks are about 49 mm cubes (red, blue, green, yellow). The top face is seen
from above and a little from the side, so with `lift=None` (the camera's height correction is not measured yet) the table position of the
top face can be off by a centimetre or so toward the camera: good enough to pick a block and aim, then refine with the gripper camera.
"""

from __future__ import annotations

import math

import cv2
import numpy as np

from lauzkach_eye_robot_control.projection import table_from_pixel

# HSV (OpenCV: H 0..179, S and V 0..255). Tuned on the real overhead pictures (tests/fixtures/overhead_*.jpg).
COLOURS: dict[str, list[tuple[tuple[int, int, int], tuple[int, int, int]]]] = {
    "red": [((0, 130, 60), (7, 255, 255)), ((170, 130, 60), (179, 255, 255))],
    "blue": [((100, 140, 40), (128, 255, 255))],
    "green": [((40, 80, 40), (85, 255, 255))],
    "yellow": [((17, 130, 110), (32, 255, 255))],
}                                                                          # (no brown / wooden block: it is not used)
BLOCK_CM = 4.9
MIN_AREA_PX, MAX_AREA_PX = 350, 7000


def _mask(hsv: np.ndarray, ranges) -> np.ndarray:
    m = np.zeros(hsv.shape[:2], np.uint8)
    for lo, hi in ranges:
        m |= cv2.inRange(hsv, np.array(lo, np.uint8), np.array(hi, np.uint8))
    m = cv2.morphologyEx(m, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    return cv2.morphologyEx(m, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))


def detect_blocks(bgr: np.ndarray, px_to_cm: list[list[float]], side_cm: float = 20.0, margin_cm: float = 3.0,
                  lift: list[float] | None = None, colours: tuple[str, ...] | None = None) -> list[dict]:
    """-> [{colour, px: [x, y], cm: [x, y], yaw_deg, area_px, size_px, score}], sorted by position (near to far, left to right)."""
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    out: list[dict] = []
    for name, ranges in COLOURS.items():
        if colours is not None and name not in colours:
            continue
        contours, _ = cv2.findContours(_mask(hsv, ranges), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for c in contours:
            area = cv2.contourArea(c)
            if not MIN_AREA_PX <= area <= MAX_AREA_PX:
                continue
            (cx, cy), (w, h), _angle = cv2.minAreaRect(c)
            if min(w, h) < 8 or max(w, h) / max(min(w, h), 1) > 2.2:
                continue                                                  # not a cube-like blob (an arm part, a cable, a shadow)
            fill = area / max(w * h, 1.0)
            if fill < 0.55:
                continue
            x_cm, y_cm = table_from_pixel(px_to_cm, cx, cy, 0.0, lift)
            if not (-margin_cm <= x_cm <= side_cm + margin_cm and -margin_cm <= y_cm <= side_cm + margin_cm):
                continue
            # rotation on the table: the longest edge direction mapped through the homography, folded into 0..90 degrees
            box = cv2.boxPoints(((cx, cy), (w, h), _angle))
            edge = box[1] - box[0] if np.linalg.norm(box[1] - box[0]) >= np.linalg.norm(box[2] - box[1]) else box[2] - box[1]
            p0 = np.array(table_from_pixel(px_to_cm, cx - edge[0] / 2, cy - edge[1] / 2, 0.0, lift))
            p1 = np.array(table_from_pixel(px_to_cm, cx + edge[0] / 2, cy + edge[1] / 2, 0.0, lift))
            yaw = math.degrees(math.atan2(p1[1] - p0[1], p1[0] - p0[0])) % 90.0
            out.append({"colour": name, "px": [round(float(cx), 1), round(float(cy), 1)], "box_px": [[round(float(x), 1), round(float(y), 1)] for x, y in box], "cm": [round(float(x_cm), 2), round(float(y_cm), 2)],
                        "yaw_deg": round(yaw, 1), "area_px": int(area), "size_px": [round(float(w), 1), round(float(h), 1)], "score": round(float(fill), 2)})
    out.sort(key=lambda b: (round(b["cm"][1] / 4), b["cm"][0]))
    return out
