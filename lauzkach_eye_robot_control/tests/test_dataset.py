"""Building feature-model samples from recorded pictures and joint angles."""

import json

import cv2
import numpy as np

from lauzkach_eye_robot_control.dataset import build_samples, load_dataset
from lauzkach_eye_robot_control.feature_model import FeatureModel
from lauzkach_eye_robot_control.robot_map import RobotMap
from lauzkach_eye_robot_control.tip import ColorMarkerObserver
from tests.test_robot_map import RECORDED

SRC = np.array([[0, 0], [20, 0], [20, 20], [0, 20]], np.float32)
DST = np.array([[147, 120], [332, 119], [335, 340], [127, 347]], np.float32)
M = np.linalg.inv(cv2.getPerspectiveTransform(SRC, DST)).tolist()
RMAP = RobotMap(RECORDED)
TRUE = FeatureModel(M, "up", RMAP, lift=np.array([-3.5, -8.0, -0.0015]), bias=np.array([0.005, -0.003]))


def fake_fk(joints):                       # the "joints" of this fake are just the tip position in cm
    return (joints["x"] / 100, joints["y"] / 100, joints["z"] / 100), np.eye(3)


def test_dataset_becomes_samples_and_unfindable_pictures_are_reported(tmp_path):
    for i, (x, y, z) in enumerate([(25, -3, 3), (30, -10, 1), (22, 2, 5)]):
        img = np.full((480, 640, 3), 190, np.uint8)
        px = TRUE.predict_pixel((x / 100, y / 100, z / 100), np.eye(3))
        cv2.circle(img, (round(px[0]), round(px[1])), 6, (255, 0, 255), -1)
        cv2.imwrite(str(tmp_path / f"pose_{i:03d}.jpg"), img)
        (tmp_path / f"pose_{i:03d}.json").write_text(json.dumps({"joints": {"x": x, "y": y, "z": z}}))
    cv2.imwrite(str(tmp_path / "pose_003.jpg"), np.full((480, 640, 3), 190, np.uint8))        # a picture with no marker in it
    (tmp_path / "pose_003.json").write_text(json.dumps({"joints": {"x": 28, "y": -5, "z": 2}}))
    entries = load_dataset(tmp_path)
    assert len(entries) == 4
    samples, skipped = build_samples(entries, fake_fk, FeatureModel(M, "up", RMAP), [ColorMarkerObserver(radius=120)])
    assert len(samples) == 3 and len(skipped) == 1 and "pose_003" in skipped[0]
    expected = TRUE.predict_pixel((0.25, -0.03, 0.03), np.eye(3))
    assert np.hypot(samples[0].pixel[0] - expected[0], samples[0].pixel[1] - expected[1]) < 0.7
