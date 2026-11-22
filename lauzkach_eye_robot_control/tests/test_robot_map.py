"""Calibration of the camera rectangle to the arm's own coordinates, from the points recorded on the real arm."""

import math

import pytest

from lauzkach_eye_robot_control.robot_map import CalibrationError, RobotMap

# The three corners recorded with the real arm's tip (calibration_poses.json), arm base frame in metres.
RECORDED = [
    {"name": "corner_00", "u": 0.0, "v": 0.0, "x": 0.1885, "y": 0.0375, "z": -0.0001},
    {"name": "corner_10", "u": 1.0, "v": 0.0, "x": 0.1745, "y": -0.1386, "z": -0.0011},
    {"name": "corner_11", "u": 1.0, "v": 1.0, "x": 0.4026, "y": -0.1393, "z": -0.0025},
]


def test_recorded_points_say_the_rectangle_is_not_20_by_20():
    m = RobotMap(RECORDED)
    width, depth = m.size_cm
    assert width == pytest.approx(17.7, abs=0.7) and depth == pytest.approx(22.8, abs=0.7)   # to the right / away from the robot
    s = m.summary()
    assert any("not 20 x 20" in w for w in s["warnings"])          # more than a slight stretch: the page must say so
    assert any("right angle" in w for w in s["warnings"])          # the three corners are about 4 degrees from orthogonal
    assert max(s["residual_mm"].values()) < 15                     # and still a usable compromise


def test_points_round_trip_and_the_near_left_corner_is_the_first_recorded_point():
    m = RobotMap(RECORDED)
    x, y = m.uv_to_xy(0.0, 0.0)
    assert math.hypot(x - 0.1885, y - 0.0375) < 0.01
    for u, v in ((0.3, 0.8), (1.0, 0.0), (0.5, 0.5)):
        assert m.xy_to_uv(*m.uv_to_xy(u, v)) == pytest.approx((u, v), abs=1e-9)
    names = [c["name"] for c in m.corners()]
    assert names == ["near-left", "near-right", "far-right", "far-left"]
    far_left = m.corners()[3]                                      # never recorded: predicted from the other three
    assert far_left["x"] == pytest.approx(0.4, abs=0.03)


def test_a_perfect_20_cm_square_gives_no_warnings():
    th = math.radians(10)
    pts = []
    for name, (u, v) in (("a", (0, 0)), ("b", (1, 0)), ("c", (1, 1)), ("d", (0, 1))):
        pts.append({"name": name, "u": float(u), "v": float(v), "z": 0.0,
                    "x": 0.2 + 0.2 * v * math.cos(th) + 0.2 * u * math.sin(th), "y": 0.0 + 0.2 * v * math.sin(th) - 0.2 * u * math.cos(th)})
    m = RobotMap(pts)
    assert m.size_cm == pytest.approx((20.0, 20.0), abs=0.01)
    assert m.summary()["warnings"] == [] and m.shear_deg < 0.01 and max(m.residual_mm.values()) < 0.01


def test_needs_three_points():
    with pytest.raises(CalibrationError):
        RobotMap(RECORDED[:2])
