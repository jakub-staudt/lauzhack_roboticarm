"""Draw a detection onto a frame: `python -m lauzkach_eye_robot_control.debug frame.jpg out.png`."""

from __future__ import annotations

import sys

import cv2
import numpy as np

from lauzkach_eye_robot_control.detect import detect, rectify


def annotate(bgr: np.ndarray, det: dict) -> np.ndarray:
    out = bgr.copy()
    for line in det["lines"]:  # every candidate line, faint
        cv2.line(out, tuple(int(v) for v in line["p0"]), tuple(int(v) for v in line["p1"]), (255, 200, 0) if not line["name"] else (0, 140, 255), 1 if not line["name"] else 2)
    for tag in det["tags"]:
        quad = np.array(tag["corners"], np.int32)
        cv2.polylines(out, [quad], True, (0, 255, 255), 2)
        cv2.putText(out, str(tag["id"]), tuple(int(v) for v in tag["center"]), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
    if det["polygon"]:
        cv2.polylines(out, [np.array(det["polygon"], np.int32)], True, (0, 255, 0), 3)
        for name, point in zip(("TL", "TR", "BR", "BL"), det["polygon"], strict=True):
            cv2.circle(out, (int(point[0]), int(point[1])), 5, (0, 255, 0), -1)
            cv2.putText(out, name, (int(point[0]) + 6, int(point[1]) - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
    return out


if __name__ == "__main__":
    frame = cv2.imread(sys.argv[1])
    det = detect(frame)
    print(f"tags {[t['id'] for t in det['tags']]}, lines {len(det['lines'])}, method: {det['method']}, aspect {det['aspect']}, ok {det['ok']}")
    for issue in det["issues"]:
        print("  issue:", issue)
    for line in det["lines"][:12]:
        print(f"  {line['kind']} {line['name'] or '-':6s} {line['p0']} -> {line['p1']} support {line['support']} cov {line['coverage']}")
    panel = annotate(frame, det)
    if det["polygon"]:
        crop = rectify(frame, det["polygon"], 360)
        pad = np.zeros((panel.shape[0], 360, 3), np.uint8)
        pad[:360] = crop
        panel = np.hstack([panel, pad])
    cv2.imwrite(sys.argv[2], panel)
