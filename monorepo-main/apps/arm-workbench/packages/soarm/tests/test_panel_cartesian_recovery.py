"""Explicit bounded recovery and continuous quantized trajectories; no hardware."""
from __future__ import annotations

import math

import pytest

import soarm
from soarm.panel import PanelController, PanelError, RobotDescription
from soarm.panel_cartesian import PROFILES, CartesianSolver, Workspace
from soarm.panel_cartesian_recovery import MAX_PITCH_CHANGE_DEG, MAX_TIP_DRIFT_M, ingress_plan

from .test_panel_cartesian import Arm
from .test_panel_cartesian_twin import LIVE_POSE, SCREENSHOT_POSE

pytestmark = pytest.mark.twin

# Deliberately synthetic EEPROM bounds. They are not actual robot calibration.
HARDWARE = {"shoulder_pan": (-110.0, 110.0), "shoulder_lift": (-100.0, 100.0),
            "elbow_flex": (-95.0, 95.0), "wrist_flex": (-95.0, 95.0),
            "wrist_roll": (-150.0, 150.0), "gripper": (0.0, 100.0)}


def make_controller(monkeypatch, pose=SCREENSHOT_POSE, hardware=HARDWARE, arm=None):
    clock = [100.0]
    monkeypatch.setattr("soarm.panel.time.monotonic", lambda: clock[0])
    description = RobotDescription.load(soarm.ROBOT_DIR)
    arm = Arm() if arm is None else arm
    arm.pose = dict(pose)
    arm.joint_limits = lambda: dict(hardware)
    controller = PanelController(arm, description)
    token = controller.claim("model recovery test")["token"]
    controller._cartesian_workspace = Workspace(table_z_m=-0.0074)
    controller._cartesian_solver = CartesianSolver(soarm.ROBOT_DIR, controller.limits)
    return controller, token, clock


def tick(controller, token, clock, sequence):
    clock[0] += 0.05
    controller.heartbeat(token)
    controller.renew_cartesian(controller._cartesian_motion["motion_id"], sequence)
    controller.tick(0.05)


@pytest.mark.parametrize("profile", ["slow", "medium", "fast"])
def test_explicit_ingress_preserves_tip_and_repairs_limit_at_selected_speed(monkeypatch, profile):
    controller, token, clock = make_controller(monkeypatch)
    solver = controller._cartesian_solver
    initial = dict(controller.measured)
    anchor = solver.position(initial)
    pitch = solver.pitch(initial)
    controller.start_cartesian("up", profile, 0, controller._sequence, "recovery")
    assert not controller.arm.writes
    assert controller.cartesian_state()["phase"] == "ingress"
    previous = dict(initial)
    reached_tracking = False
    peak_step = 0.0
    for sequence in range(1, 50):
        tick(controller, token, clock, sequence)
        assert controller.cartesian_state()["active"], controller.clip
        sent = controller.command
        peak_step = max(peak_step, max(abs(sent[joint] - previous[joint]) for joint in sent))
        assert sent["shoulder_lift"] >= previous["shoulder_lift"] - 1e-9
        assert (max(abs(sent[joint] - previous[joint]) for joint in sent)
                <= PROFILES[profile]["joint_deg_s"] * .05)
        assert all(HARDWARE[joint][0] <= value <= HARDWARE[joint][1] for joint, value in sent.items())
        assert all(controller.limits[joint][0] <= value <= controller.limits[joint][1]
                   for joint, value in sent.items() if joint != "shoulder_lift")
        assert math.dist(solver.position(sent), anchor) <= MAX_TIP_DRIFT_M
        assert abs(solver._angle(solver.pitch(sent) - pitch)) <= math.radians(MAX_PITCH_CHANGE_DEG)
        assert all(sent[joint] == initial[joint] for joint in ("shoulder_pan", "wrist_roll", "gripper"))
        previous = dict(sent)
        if controller.cartesian_state()["phase"] == "tracking":
            reached_tracking = True
            break
    assert reached_tracking
    if profile != "slow":
        # After the same 200ms startup ramp, these profiles really exceed Slow.
        assert peak_step > .4
    assert controller.measured["shoulder_lift"] >= -95


def test_normal_mode_does_not_silently_enter_recovery(monkeypatch):
    controller, _, _ = make_controller(monkeypatch)
    with pytest.raises(PanelError, match="recovery is required"):
        controller.start_cartesian("up", "fast", 0, controller._sequence)
    assert not controller.arm.writes


@pytest.mark.parametrize("outside", ["measured", "command"])
def test_normal_mode_checks_feedback_and_command_limits_separately(monkeypatch, outside):
    controller, _, _ = make_controller(monkeypatch)
    controller.measured["shoulder_lift"] = -94.9
    controller.command["shoulder_lift"] = -94.9
    getattr(controller, outside)["shoulder_lift"] = -95.6
    controller.target = dict(controller.command)
    with pytest.raises(PanelError, match="recovery is required"):
        controller.start_cartesian("up", "fast", 0, controller._sequence)
    assert not controller.cartesian_state()["active"]
    assert not controller.arm.writes


@pytest.mark.parametrize("kind", ["unknown_hardware", "too_far", "second_violation", "physical_limit"])
def test_ingress_rejects_unverified_or_unsafe_pose_before_writes(monkeypatch, kind):
    pose = dict(SCREENSHOT_POSE)
    hardware = dict(HARDWARE)
    if kind == "unknown_hardware":
        hardware.pop("shoulder_lift")
    elif kind == "too_far":
        pose["shoulder_lift"] = -96.1
    elif kind == "second_violation":
        pose["elbow_flex"] = 92.1
    else:
        hardware["shoulder_lift"] = (-95.0, 100.0)
    controller, _, _ = make_controller(monkeypatch, pose, hardware)
    with pytest.raises(PanelError):
        controller.start_cartesian("up", "fast", 0, controller._sequence, "recovery")
    assert not controller.arm.writes


def test_entire_ingress_is_prevalidated(monkeypatch):
    controller, _, _ = make_controller(monkeypatch)
    solver = controller._cartesian_solver
    original = solver.solve_position
    calls = [0]

    def later_failure(*args, **kwargs):
        calls[0] += 1
        if calls[0] == 10:
            raise ValueError("Later ingress waypoint exceeds tilt budget.")
        return original(*args, **kwargs)

    monkeypatch.setattr(solver, "solve_position", later_failure)
    with pytest.raises(PanelError, match="Later ingress"):
        controller.start_cartesian("up", "fast", 0, controller._sequence, "recovery")
    assert not controller.arm.writes
    assert not controller.cartesian_state()["active"]


def test_in_bounds_recovery_free_tilt_stays_within_original_total_budget(monkeypatch):
    pose = dict(SCREENSHOT_POSE, shoulder_lift=-95.0)
    controller, token, clock = make_controller(monkeypatch, pose)
    solver = controller._cartesian_solver
    initial_pitch = solver.pitch(pose)
    controller.start_cartesian("back", "fast", 0, controller._sequence, "recovery")
    moved = False
    for sequence in range(1, 30):
        tick(controller, token, clock, sequence)
        if not controller.cartesian_state()["active"]:
            break
        moved = moved or controller._cartesian_motion["travel_m"] > 0.0001
        assert abs(solver._angle(solver.pitch(controller.command) - initial_pitch)) <= math.radians(3)
    assert moved
    assert not controller.cartesian_state()["active"]
    assert "tilt" in controller.clip.lower()


class QuantizedArm(Arm):
    def _to_ticks(self, joint, value):
        return round(value * 10) if joint == "gripper" else round(2048 + value * 4096 / 360)

    def _to_units(self, joint, value):
        return value / 10 if joint == "gripper" else (value - 2048) * 360 / 4096

    def send_joints(self, goals):
        self.pose = {joint: self._to_units(joint, self._to_ticks(joint, value))
                     for joint, value in goals.items()}
        self.writes.append(dict(self.pose))
        return dict(self.pose)


@pytest.mark.parametrize("pan", [0.0, 40.0])
def test_subquantum_continuous_progress_accumulates_without_backtracking(monkeypatch, pan):
    arm = QuantizedArm()
    pose = {joint: arm._to_units(joint, arm._to_ticks(joint, value))
            for joint, value in dict(LIVE_POSE, shoulder_pan=pan).items()}
    controller, token, clock = make_controller(monkeypatch, pose, arm=arm)
    monkeypatch.setitem(PROFILES["slow"], "linear_m_s", 0.001)
    controller.start_cartesian("forward", "slow", 0, controller._sequence)
    motion = controller._cartesian_motion
    solver = controller._cartesian_solver
    achieved = []
    desired = []
    previous = dict(controller.command)
    minimum_tick_changes = []
    for sequence in range(1, 61):
        tick(controller, token, clock, sequence)
        assert controller.cartesian_state()["active"], controller.clip
        point = solver.position(controller.command)
        progress = sum((point[index] - motion["anchor_position"][index]) * motion["vector"][index]
                       for index in range(3))
        achieved.append(progress)
        desired.append(motion["travel_m"])
        changes = [abs(controller.command[joint] - previous[joint]) for joint in previous
                   if abs(controller.command[joint] - previous[joint]) > 1e-9 and joint != "gripper"]
        minimum_tick_changes.extend(changes)
        previous = dict(controller.command)
    assert desired[-1] > .0028
    assert achieved[-1] > .0025
    assert all(after >= before for before, after in zip(desired, desired[1:], strict=False))
    # Quantized goals may hold still but must not oscillate the TCP backward.
    assert min(after - before for before, after in zip(achieved, achieved[1:], strict=False)) >= -1e-9
    assert min(minimum_tick_changes) == pytest.approx(360 / 4096)
    assert max(abs(ideal - actual) for ideal, actual in zip(desired, achieved, strict=True)) < .0005


def test_quantized_ingress_plan_is_monotonic_and_fully_inside_at_end():
    arm = QuantizedArm()
    pose = {joint: arm._to_units(joint, arm._to_ticks(joint, value))
            for joint, value in SCREENSHOT_POSE.items()}
    description = RobotDescription.load(soarm.ROBOT_DIR)
    solver = CartesianSolver(soarm.ROBOT_DIR, description.limits)
    def quantize(goals):
        return {joint: arm._to_units(joint, arm._to_ticks(joint, value)) for joint, value in goals.items()}
    plan = ingress_plan(solver, pose, description.limits, HARDWARE, quantize)
    lifts = [pose["shoulder_lift"], *(item["shoulder_lift"] for item in plan)]
    assert all(after >= before for before, after in zip(lifts, lifts[1:], strict=False))
    assert plan[-1]["shoulder_lift"] >= -94.3


@pytest.mark.parametrize("bias", [0.3, 0.6])
def test_measured_offset_never_leaves_ingress_waiting_forever(monkeypatch, bias):
    class BiasedArm(Arm):
        def send_joints(self, goals):
            super().send_joints(goals)
            self.pose["shoulder_lift"] -= bias
            return dict(goals)

    controller, token, clock = make_controller(monkeypatch, arm=BiasedArm())
    controller.start_cartesian("up", "fast", 0, controller._sequence, "recovery")
    for sequence in range(1, 50):
        tick(controller, token, clock, sequence)
        if not controller.cartesian_state()["active"] or controller.cartesian_state()["phase"] == "tracking":
            break
    assert not controller.cartesian_state()["active"] or controller.cartesian_state()["phase"] == "tracking"
    if controller.cartesian_state()["active"]:
        assert controller.measured["shoulder_lift"] >= -95
    else:
        assert "budget" in controller.clip.lower() or "locally" in controller.clip.lower()
