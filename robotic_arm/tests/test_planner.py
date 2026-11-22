import numpy as np
import pytest

from soarm_lab.kinematics import RobotKinematics
from soarm_lab.robot import load_robot

from robotic_arm.calibration import POINTS, apply_homography, fit_calibration, fit_homography
from robotic_arm.config import Config
from robotic_arm.planner import ARM, Planner, PlanError


@pytest.fixture(scope="module")
def kin():
    return RobotKinematics(load_robot())


def synthetic_calibration(kin, cfg):
    """Pretend the rectangle sits 0.18..0.30 m in front of the arm and was calibrated by placing the tip."""
    rng_x, rng_y = (0.18, 0.30), (-0.06, 0.06)
    recorded, seed = [], dict(kin.spec.home)
    for name, (u, v) in POINTS.items():
        pos = np.array([rng_x[0] + u * (rng_x[1] - rng_x[0]), rng_y[0] + v * (rng_y[1] - rng_y[0]), 0.02])
        sol = kin.ik(pos, seed=seed)
        assert sol.ok, name
        seed = sol.joints
        recorded.append({"name": name, "u": u, "v": v, "joints": sol.joints})
    return fit_calibration(kin, recorded)


def test_homography_roundtrip():
    uv = np.array([[0, 0], [1, 0], [1, 1], [0, 1], [.5, .5]], float)
    H_true = np.array([[0.1, 0.02, 0.2], [-0.03, 0.12, -0.05], [0.01, 0.0, 1.0]])
    xy = apply_homography(H_true, uv)
    H = fit_homography(uv, xy)
    assert np.allclose(apply_homography(H, uv), xy, atol=1e-9)


def test_calibration_residual_small(kin):
    cal = synthetic_calibration(kin, Config())
    assert cal.residual_max_m < 2e-3
    assert abs(cal.z_table - 0.02) < 2e-3


def test_goto_plan_reaches_destination(kin):
    cfg = Config()
    cal = synthetic_calibration(kin, cfg)
    planner = Planner(kin, cal, cfg)
    start = {**kin.spec.home, "gripper": 40.0}
    plan = planner.goto(start, (0.7, 0.3))
    assert [s.name for s in plan.segments] == ["rise", "glide", "lower"]
    end = plan.rows[-1]
    tip = kin.tcp(end)[:3, 3]
    x, y = cal.xy(0.7, 0.3)
    assert np.linalg.norm(tip[:2] - [x, y]) < 0.004
    assert abs(tip[2] - (cal.z_table + cfg.goto_offset)) < 0.004
    # glide stays near travel height
    glide = plan.rows[plan.segments[1].start:plan.segments[1].end]
    zs = [kin.tcp(r)[2, 3] - cal.z_table for r in glide]
    assert max(zs) < cfg.travel_height + 0.015 and min(zs) > cfg.travel_height - 0.015
    assert all(r["gripper"] == 40.0 for r in plan.rows)


def test_speed_and_limits_respected(kin):
    cfg = Config()
    planner = Planner(kin, synthetic_calibration(kin, cfg), cfg)
    plan = planner.goto(dict(kin.spec.home), (0.2, 0.8))
    prev = dict(kin.spec.home)
    for row in plan.rows:
        for j in ARM:
            assert abs(row[j] - prev[j]) * cfg.hz <= cfg.arm_speed * 1.01
            lo, hi = kin.spec.limits[j]
            assert lo <= row[j] <= hi
        prev = row


def test_outside_rectangle_refused(kin):
    cfg = Config()
    planner = Planner(kin, synthetic_calibration(kin, cfg), cfg)
    with pytest.raises(PlanError):
        planner.goto(dict(kin.spec.home), (1.5, 0.5))
    with pytest.raises(PlanError):
        planner.goto(dict(kin.spec.home), (float("nan"), 0.5))


def test_pick_place_plan(kin):
    cfg = Config()
    planner = Planner(kin, synthetic_calibration(kin, cfg), cfg)
    plan = planner.plan({**kin.spec.home, "gripper": 35.0}, (0.2, 0.3), (0.8, 0.7))
    assert [s.name for s in plan.segments][:3] == ["open", "to_pick_hover", "descend_pick"]
    assert sum(len(c) for c in plan.chunks(cfg.hz, cfg.max_chunk_s)) == len(plan.rows)
