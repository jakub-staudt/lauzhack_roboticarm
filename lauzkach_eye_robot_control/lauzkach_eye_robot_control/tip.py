"""Find something on the gripper in the overhead picture. Every observer returns a pixel and how sure it is, or None.

The white arm and the grey paper cannot be told apart by brightness in this camera's exposure, so a tip cannot be found "by
itself". Two ways that work:

* `ColorMarkerObserver` / `TagMarkerObserver`: a small bright sticker or an AprilTag on the gripper. The most reliable
  (needs someone to attach it once).
* `DarkBlobObserver`: EXPERIMENTAL and NOT reliable. It looks for a compact, nearly black blob near a hint (where the arm's own
  model says the feature should be). Tried on a real picture it latched onto the black cable bundle hanging off the arm
  instead of a rigid part of the gripper, so its confidence is capped below what the closed loop accepts: on its own it
  cannot drive a move.

What an observer finds is a pixel of *something rigid on the gripper*. `feature_model.py` relates that to the tip.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Protocol

import cv2
import numpy as np

from lauzkach_eye_robot_control.detect import detect_tags


@dataclass
class TipObservation:
    px: tuple[float, float]
    confidence: float
    method: str
    detail: dict = field(default_factory=dict)


class PixelObserver(Protocol):
    def observe(self, bgr: np.ndarray, hint: tuple[float, float] | None = None) -> TipObservation | None: ...


def _blobs(mask: np.ndarray, min_area: float, max_area: float) -> list[dict]:
    """Connected components of a binary mask with their centroid, area and how round/solid they are."""
    count, labels, stats, centroids = cv2.connectedComponentsWithStats(mask, connectivity=8)
    out = []
    for i in range(1, count):
        area = float(stats[i, cv2.CC_STAT_AREA])
        if not min_area <= area <= max_area:
            continue
        ys, xs = np.nonzero(labels == i)
        hull = cv2.convexHull(np.stack([xs, ys], 1).astype(np.int32))
        solidity = area / max(1.0, cv2.contourArea(hull))
        width, height = stats[i, cv2.CC_STAT_WIDTH], stats[i, cv2.CC_STAT_HEIGHT]
        out.append({"centroid": (float(centroids[i][0]), float(centroids[i][1])), "area": area, "solidity": float(solidity),
                    "aspect": float(min(width, height) / max(width, height))})
    return out


def _pick(blobs: list[dict], hint: tuple[float, float] | None, radius: float) -> tuple[dict, float] | None:
    """The blob nearest the hint (within `radius`), or simply the largest when there is no hint."""
    if not blobs:
        return None
    if hint is None:
        best = max(blobs, key=lambda b: b["area"])
        return best, 1.0
    near = [(math.dist(b["centroid"], hint), b) for b in blobs]
    near = [(d, b) for d, b in near if d <= radius]
    if not near:
        return None
    d, best = min(near, key=lambda t: t[0])
    return best, 1.0 - d / radius


class ColorMarkerObserver:
    """A brightly coloured sticker. Default colour: saturated magenta/pink (rare on this table); set `hsv_lo`/`hsv_hi` for another."""

    def __init__(self, hsv_lo=(140, 110, 110), hsv_hi=(175, 255, 255), min_area: float = 12, max_area: float = 3000, radius: float = 90) -> None:
        self.lo, self.hi = np.array(hsv_lo, np.uint8), np.array(hsv_hi, np.uint8)
        self.min_area, self.max_area, self.radius = min_area, max_area, radius

    def observe(self, bgr: np.ndarray, hint: tuple[float, float] | None = None) -> TipObservation | None:
        mask = cv2.inRange(cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV), self.lo, self.hi)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))   # odd kernel: an even one shifts the blob by half a pixel
        picked = _pick(_blobs(mask, self.min_area, self.max_area), hint, self.radius)
        if picked is None:
            return None
        blob, nearness = picked
        confidence = 0.5 * min(1.0, blob["solidity"]) + 0.5 * max(0.0, nearness)
        return TipObservation(blob["centroid"], round(confidence, 3), "colour marker", {"area": blob["area"]})


class TagMarkerObserver:
    """An AprilTag (family 36h11) on the gripper. `offset_px`: where the tip is, relative to the tag's centre, in the picture."""

    def __init__(self, tag_id: int, offset_px: tuple[float, float] = (0.0, 0.0)) -> None:
        self.tag_id, self.offset = tag_id, offset_px

    def observe(self, bgr: np.ndarray, hint: tuple[float, float] | None = None) -> TipObservation | None:
        for tag in detect_tags(cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)):
            if tag["id"] == self.tag_id:
                cx, cy = tag["center"]
                return TipObservation((cx + self.offset[0], cy + self.offset[1]), 0.95, f"tag {self.tag_id}", {"corners": tag["corners"]})
        return None


MAX_DARK_BLOB_CONFIDENCE = 0.45      # below ServoConfig.min_confidence (0.5): a dark blob is never trusted by the loop by default


class DarkBlobObserver:
    """EXPERIMENTAL: a compact, nearly black blob near where the arm's own model says a feature is (`hint` is required).
    On the real camera it picked a black cable instead of a rigid part, so its confidence is capped (see above)."""

    def __init__(self, dark_below: int = 60, min_area: float = 120, max_area: float = 6000, min_solidity: float = 0.7, radius: float = 45) -> None:
        self.dark_below, self.min_area, self.max_area, self.min_solidity, self.radius = dark_below, min_area, max_area, min_solidity, radius

    def observe(self, bgr: np.ndarray, hint: tuple[float, float] | None = None) -> TipObservation | None:
        if hint is None:
            return None                                          # without a hint any dark thing (cable, cube, tag) would do
        gray = cv2.GaussianBlur(cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY), (3, 3), 0)
        mask = (gray < self.dark_below).astype(np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))   # cables and thin lines go; compact blobs stay
        blobs = [b for b in _blobs(mask, self.min_area, self.max_area) if b["solidity"] >= self.min_solidity]
        picked = _pick(blobs, hint, self.radius)
        if picked is None:
            return None
        blob, nearness = picked
        confidence = 0.4 * blob["solidity"] + 0.6 * max(0.0, nearness)
        return TipObservation(blob["centroid"], round(min(confidence, MAX_DARK_BLOB_CONFIDENCE), 3), "dark blob", {"area": blob["area"], "solidity": round(blob["solidity"], 2)})


def observe_first(observers: list[PixelObserver], bgr: np.ndarray, hint: tuple[float, float] | None = None) -> TipObservation | None:
    """The first observer in the list that sees something (put the most reliable one first)."""
    for observer in observers:
        found = observer.observe(bgr, hint)
        if found is not None:
            return found
    return None
