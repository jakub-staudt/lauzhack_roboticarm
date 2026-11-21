"""Position-only jog on the shipped SO-101 model, without connecting a backend."""
import math
from dataclasses import replace

import pytest

import soarm
from soarm.panel import PanelController, RobotDescription
from soarm.panel_cartesian import PROFILES, CartesianSolver, Workspace

from .test_panel_cartesian import Arm

pytestmark = pytest.mark.twin


@pytest.mark.parametrize("direction", ["forward", "back", "up", "down"])
@pytest.mark.parametrize("pan", [-90.0, -40.0, 0.0, 40.0, 90.0])
@pytest.mark.parametrize("profile", list(PROFILES.values()), ids=list(PROFILES))
def test_local_position_solver_preserves_pan_roll_and_gripper(direction, pan, profile):
    description = RobotDescription.load(soarm.ROBOT_DIR)
    solver = CartesianSolver(soarm.ROBOT_DIR, description.limits)
    seed = dict(description.home)
    seed["shoulder_pan"] = pan
    before = solver.position(seed)
    target = list(before)
    heading = solver.heading(seed)
    current = seed
    anchored_pitch = solver.pitch(seed)
    distance = profile["linear_m_s"] * 0.05
    for _ in range(12):
        if direction in ("up", "down"):
            target[2] += distance * (1 if direction == "up" else -1)
        else:
            for index in (0, 1):
                target[index] += distance * heading[index] * (1 if direction == "forward" else -1)
        solved = solver.solve(current, tuple(target))
        reached = solver.position(solved)
        assert math.dist(reached, target) < 0.0001
        assert abs(solver._angle(solver.pitch(solved) - anchored_pitch)) < math.radians(0.01)
        for joint in ("shoulder_pan", "wrist_roll", "gripper"):
            assert solved[joint] == seed[joint]
        assert all(description.limits[joint][0] <= value <= description.limits[joint][1]
                   for joint, value in solved.items())
        current = solved
    assert math.dist(reached, before) > 12 * distance - 0.0001


def test_pan_positive_rotates_right_in_base_frame():
    description = RobotDescription.load(soarm.ROBOT_DIR)
    solver = CartesianSolver(soarm.ROBOT_DIR, description.limits)
    before = solver.position(description.home)
    after = solver.position(dict(description.home, shoulder_pan=5.0))
    assert after[1] < before[1]


@pytest.mark.parametrize("direction", ["forward", "back", "up", "down"])
@pytest.mark.parametrize("profile", list(PROFILES))
@pytest.mark.parametrize("pan", [-90.0, 90.0])
def test_controller_tracks_model_line_with_joint_speed_limits(direction, profile, pan, monkeypatch):
    now = [100.0]
    monkeypatch.setattr("soarm.panel.time.monotonic", lambda: now[0])
    description = RobotDescription.load(soarm.ROBOT_DIR)
    arm = Arm()
    arm.pose = dict(description.home, shoulder_pan=pan)
    controller = PanelController(arm, description)
    controller.claim("simulation")
    solver = CartesianSolver(soarm.ROBOT_DIR, description.limits)
    controller._cartesian_solver = solver
    controller._cartesian_workspace = Workspace(table_z_m=-0.01)
    controller.start_cartesian(direction, profile, 0, controller._sequence)
    before = solver.position(controller.command)
    heading = solver.heading(controller.command)
    previous = dict(controller.command)
    for index in range(12):
        now[0] += 0.05
        controller.renew_cartesian(controller._cartesian_motion["motion_id"], index + 1)
        controller.tick(0.05)
        assert controller.cartesian_state()["active"], controller.clip
        step = max(abs(controller.command[joint] - previous[joint]) for joint in previous)
        assert step <= PROFILES[profile]["joint_deg_s"] * 0.05 + 1e-8
        previous = dict(controller.command)
    reached = solver.position(controller.command)
    if direction in ("forward", "back"):
        assert abs(reached[2] - before[2]) < 0.001
        assert abs((reached[0] - before[0]) * heading[1] -
                   (reached[1] - before[1]) * heading[0]) < 0.001
    else:
        assert math.dist(reached[:2], before[:2]) < 0.001
    assert math.dist(reached, before) > 0.005


SCREENSHOT_POSE = dict(shoulder_pan=0.4, shoulder_lift=-95.6, elbow_flex=77.4,
                       wrist_flex=-9.1, wrist_roll=-13.3, gripper=99.0)
LIVE_POSE = dict(shoulder_pan=-1.2305, shoulder_lift=-10.9863, elbow_flex=34.7168,
                 wrist_flex=-14.5898, wrist_roll=-13.0078, gripper=98.9766)
# Captured software/EEPROM intersection. These are not raw EEPROM bounds.
LIVE_EFFECTIVE_LIMITS = {"shoulder_pan": (-105.0, 64.599609375), "shoulder_lift": (-95.0, 95.0),
                         "elbow_flex": (-92.0, 87.1875), "wrist_flex": (-95.0, 95.0),
                         "wrist_roll": (-113.818359375, 95.185546875),
                         "gripper": (7.748538011695906, 100.0)}


def test_screenshot_outside_soft_limit_is_never_clipped_as_ik_seed():
    description = RobotDescription.load(soarm.ROBOT_DIR)
    solver = CartesianSolver(soarm.ROBOT_DIR, description.limits)
    point = solver.position(SCREENSHOT_POSE)
    for vector in ((0.0005, 0, 0), (-0.0005, 0, 0), (0, 0, 0.0005), (0, 0, -0.0005)):
        target = tuple(point[index] + vector[index] for index in range(3))
        with pytest.raises(ValueError, match="recovery is required"):
            solver.solve(SCREENSHOT_POSE, target)


@pytest.mark.parametrize("direction", ["forward", "back", "up", "down"])
def test_reported_live_pose_keeps_full_gripper_rotation(direction):
    import numpy as np

    solver = CartesianSolver(soarm.ROBOT_DIR, LIVE_EFFECTIVE_LIMITS)
    before = solver.kin.tcp(LIVE_POSE).copy()
    heading = solver.heading(LIVE_POSE)
    vector = ((heading[0], heading[1], 0.0) if direction == "forward" else
              (-heading[0], -heading[1], 0.0) if direction == "back" else
              (0.0, 0.0, 1 if direction == "up" else -1))
    seed = dict(LIVE_POSE)
    pitch = solver.pitch(seed)
    for index in range(1, 21):
        target = tuple(before[axis, 3] + index * 0.0005 * vector[axis] for axis in range(3))
        seed = solver.solve(seed, target, pitch=pitch)
        reached = solver.kin.tcp(seed).copy()
        assert np.linalg.norm(reached[:3, 3] - target) < 0.00002
        assert np.linalg.norm(reached[:3, :3] - before[:3, :3]) < 0.0003


@pytest.mark.parametrize("direction", ["forward", "back", "up", "down"])
def test_quantized_goals_follow_original_line_and_tilt(direction, monkeypatch):
    now = [100.0]
    monkeypatch.setattr("soarm.panel.time.monotonic", lambda: now[0])
    description = replace(RobotDescription.load(soarm.ROBOT_DIR), limits=LIVE_EFFECTIVE_LIMITS)
    arm = Arm()
    quantum = 360.0 / 4096
    arm.pose = {joint: (round(value / quantum) * quantum if joint != "gripper" else value)
                for joint, value in LIVE_POSE.items()}

    def quantized_send(goals):
        arm.pose = {joint: (round(value / quantum) * quantum if joint != "gripper" else value)
                    for joint, value in goals.items()}
        return dict(arm.pose)

    arm.send_joints = quantized_send
    controller = PanelController(arm, description)
    controller.claim("quantized simulation")
    solver = CartesianSolver(soarm.ROBOT_DIR, description.limits)
    controller._cartesian_solver = solver
    controller._cartesian_workspace = Workspace(table_z_m=-0.01)
    controller.start_cartesian(direction, "slow", 0, controller._sequence)
    motion = dict(controller._cartesian_motion)
    for index in range(1, 31):
        now[0] += 0.05
        controller.renew_cartesian(motion["motion_id"], index)
        controller.tick(0.05)
        assert controller.cartesian_state()["active"], controller.clip
        position = solver.position(controller.command)
        delta = [position[axis] - motion["anchor_position"][axis] for axis in range(3)]
        along = sum(delta[axis] * motion["vector"][axis] for axis in range(3))
        transverse = math.sqrt(sum((delta[axis] - along * motion["vector"][axis]) ** 2 for axis in range(3)))
        assert transverse < 0.001
        pitch_error = solver._angle(solver.pitch(controller.command) - motion["anchor_pitch"])
        assert abs(pitch_error) < math.radians(.25)
    assert along > 0.01
