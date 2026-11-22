"""The feature model recovers a known set-up from noisy synthetic samples, and then locates the tip from a pixel."""

import cv2
import numpy as np
import pytest

from lauzkach_eye_robot_control.feature_model import FeatureModel, Sample
from lauzkach_eye_robot_control.robot_map import RobotMap
from tests.test_robot_map import RECORDED

SRC = np.array([[0, 0], [20, 0], [20, 20], [0, 20]], np.float32)
DST = np.array([[147, 120], [332, 119], [335, 340], [127, 347]], np.float32)
M = np.linalg.inv(cv2.getPerspectiveTransform(SRC, DST)).tolist()
RMAP = RobotMap(RECORDED)

TRUE = FeatureModel(M, "up", RMAP, offset=np.array([0.012, -0.02, 0.035]), lift=np.array([-3.5, -8.0, -0.0015]), bias=np.array([0.007, -0.005]))


def tool_rotation(yaw, pitch):
    p = np.radians(pitch)
    x = np.array([np.cos(p) * np.cos(yaw), np.cos(p) * np.sin(yaw), -np.sin(p)])
    y = np.array([-np.sin(yaw), np.cos(yaw), 0.0])
    return np.column_stack([x, y, np.cross(x, y)])


def make_samples(n, seed, noise_px=0.4):
    rng = np.random.default_rng(seed)
    out = []
    for _ in range(n):
        u, v = rng.uniform(0.1, 0.9, 2)
        x, y = RMAP.uv_to_xy(u, v)
        z = rng.uniform(-0.003, 0.06)
        R = tool_rotation(rng.uniform(-0.6, 0.6), rng.uniform(45, 90)) @ np.array(
            [[np.cos(a := rng.uniform(-1.5, 1.5)), -np.sin(a), 0], [np.sin(a), np.cos(a), 0], [0, 0, 1]])   # roll about the tool axis
        px = np.array(TRUE.predict_pixel((x, y, z), R)) + rng.normal(0, noise_px, 2)
        out.append(Sample((x, y, z), R.tolist(), (float(px[0]), float(px[1]))))
    return out


def test_fit_recovers_the_offset_the_lift_and_the_calibration_bias():
    model = FeatureModel.fit(make_samples(40, seed=1), M, "up", RMAP)
    assert model.rms_px < 0.8                                                      # as good as the 0.4 px noise allows
    assert np.allclose(model.offset, TRUE.offset, atol=0.004)                      # the feature's place on the gripper to 4 mm
    assert np.allclose(model.bias, TRUE.bias, atol=0.0015)                         # the calibration shift to 1.5 mm


def test_the_fitted_model_locates_the_tip_on_the_board_from_a_pixel():
    model = FeatureModel.fit(make_samples(40, seed=2), M, "up", RMAP)
    for s in make_samples(15, seed=99):                                            # poses it has never seen
        x, y = model.tip_xy_from_pixel(s.pixel, s.tip[2], s.rotation)
        assert np.hypot(x - s.tip[0], y - s.tip[1]) < 0.003                         # within 3 mm of the true tip


def test_a_marker_at_the_tip_needs_no_offset():
    true = FeatureModel(M, "up", RMAP, lift=np.array([-3.5, -8.0, -0.0015]), bias=np.array([0.006, 0.004]))
    rng = np.random.default_rng(5)
    samples = []
    for _ in range(25):
        x, y = RMAP.uv_to_xy(*rng.uniform(0.1, 0.9, 2)); z = rng.uniform(0, 0.05)
        px = np.array(true.predict_pixel((x, y, z), np.eye(3))) + rng.normal(0, 0.3, 2)
        samples.append(Sample((x, y, z), np.eye(3).tolist(), (float(px[0]), float(px[1]))))
    model = FeatureModel.fit(samples, M, "up", RMAP, fit_offset=False)
    assert np.allclose(model.offset, 0) and np.allclose(model.bias, true.bias, atol=0.001) and model.rms_px < 0.6


def test_too_few_samples_are_refused():
    with pytest.raises(ValueError):
        FeatureModel.fit(make_samples(5, seed=3), M, "up", RMAP)
