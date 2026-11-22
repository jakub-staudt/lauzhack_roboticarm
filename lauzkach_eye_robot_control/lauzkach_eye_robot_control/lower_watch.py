"""Watch the gripper camera while the arm lowers onto a block, and say when it has gone down far enough.

The block grows in the picture as the camera comes down. When the selected block's lower horizontal edge reaches the bottom of the
picture (touches it or crosses it) the gripper is low enough: that is a visual safety check next to the height limit. The watcher runs
on this machine (the server's Python has no image library): it reads the gripper camera, and when the edge is there it tells the agent to
stop lowering. It never moves the arm itself.
"""

from __future__ import annotations

import threading
import time
from collections.abc import Callable

import cv2
import numpy as np

from lauzkach_eye_robot_control.blocks import COLOURS, _mask

MIN_AREA_PX = 500


def bottom_edge_reached(bgr: np.ndarray, colour: str, margin_px: int = 6) -> dict:
    """-> {seen, reached, bottom_px, height_px, area_px}: is the biggest blob of `colour` touching or crossing the bottom edge?"""
    h = bgr.shape[0]
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    contours, _ = cv2.findContours(_mask(hsv, COLOURS[colour]), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    best = None
    for c in contours:
        area = cv2.contourArea(c)
        if area >= MIN_AREA_PX and (best is None or area > best[0]):
            best = (area, c)
    if best is None:
        return {"seen": False, "reached": False, "bottom_px": None, "height_px": h, "area_px": 0}
    x, y, w, hh = cv2.boundingRect(best[1])
    return {"seen": True, "reached": bool(y + hh >= h - margin_px), "bottom_px": int(y + hh), "height_px": h, "area_px": int(best[0])}


class LowerWatcher:
    """One watcher per lowering. `still_lowering()` says whether the agent is still going down; `stop()` asks it to stop."""

    def __init__(self, colour: str, fetch: Callable[[], np.ndarray | None], still_lowering: Callable[[], bool], stop: Callable[[], None],
                 on_note: Callable[[str], None] = lambda s: None, confirm_frames: int = 2, period_s: float = 0.15, max_s: float = 90.0) -> None:
        self.colour, self.fetch, self.still_lowering, self.stop_cb, self.note = colour, fetch, still_lowering, stop, on_note
        self.confirm_frames, self.period_s, self.max_s = confirm_frames, period_s, max_s
        self.status: dict = {"state": "watching", "colour": colour}
        self._thread = threading.Thread(target=self._run, daemon=True, name="lower-watch")

    def start(self) -> LowerWatcher:
        self._thread.start()
        return self

    def join(self, timeout: float = 5.0) -> None:
        self._thread.join(timeout)

    def _run(self) -> None:
        hits, end = 0, time.monotonic() + self.max_s
        last: dict = {}
        while time.monotonic() < end:
            if not self.still_lowering():
                self.status = {"state": "finished without seeing the edge", **last, "colour": self.colour}
                return
            frame = None
            try:
                frame = self.fetch()
            except Exception:                                           # noqa: BLE001 - a missed frame is not an error
                frame = None
            if frame is not None:
                last = bottom_edge_reached(frame, self.colour)
                hits = hits + 1 if last["reached"] else 0
                self.status = {"state": "watching", "colour": self.colour, **last}
                if hits >= self.confirm_frames:
                    self.stop_cb()
                    self.status = {"state": "edge reached: told the arm to stop lowering", "colour": self.colour, **last}
                    self.note(f"The {self.colour} block's edge reached the bottom of the gripper camera picture: lowered enough.")
                    return
            time.sleep(self.period_s)
        self.status = {"state": "timed out", "colour": self.colour, **last}
