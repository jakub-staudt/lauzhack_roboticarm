"""Turn recorded (picture, joint angles) pairs into samples for the feature model."""

from __future__ import annotations

import json
from collections.abc import Callable
from pathlib import Path

import cv2
import numpy as np

from lauzkach_eye_robot_control.feature_model import FeatureModel, Sample
from lauzkach_eye_robot_control.tip import PixelObserver, observe_first


def load_dataset(folder: str | Path) -> list[dict]:
    """The pairs written by tools/record_pose_samples.py: pose_000.jpg + pose_000.json (joints in degrees)."""
    entries = []
    for meta in sorted(Path(folder).glob("pose_*.json")):
        image = meta.with_suffix(".jpg")
        if image.exists():
            entries.append({"image": image, **json.loads(meta.read_text())})
    return entries


def build_samples(entries: list[dict], fk: Callable[[dict], tuple[tuple[float, float, float], np.ndarray]], template: FeatureModel,
                  observers: list[PixelObserver]) -> tuple[list[Sample], list[str]]:
    """One sample per picture in which the feature was found. `fk(joints)` -> (tip xyz [m], tool rotation 3x3) in the arm base frame.
    The rough model `template` (no offset, no lift) only provides the hint of where to look."""
    samples, skipped = [], []
    for entry in entries:
        image = cv2.imread(str(entry["image"]))
        if image is None:
            skipped.append(f"{Path(entry['image']).name}: unreadable picture")
            continue
        tip, rotation = fk({k: float(v) for k, v in entry["joints"].items()})
        seen = observe_first(observers, image, template.predict_pixel(tip, rotation))
        if seen is None or seen.confidence < 0.4:
            skipped.append(f"{Path(entry['image']).name}: the feature was not found near where the arm model expects it")
            continue
        samples.append(Sample(tuple(float(v) for v in tip), np.asarray(rotation, float).tolist(), seen.px))
    return samples, skipped
