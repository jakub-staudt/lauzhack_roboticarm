"""Direct wrist tilt and roll on fake hardware plus the shipped model."""
from __future__ import annotations

import math

import numpy as np
import pytest

import soarm
from soarm.feetech import FeetechArm, FeetechSettings
from soarm.panel import PanelError, RobotDescription, run_command
from soarm.panel_cartesian import PROFILES, WATCHDOG_S, Workspace

from .test_panel_cartesian import Arm
from .test_panel_cartesian_recovery import make_controller, tick
from .test_panel_cartesian_twin import LIVE_POSE

pytestmark = pytest.mark.twin
DESCRIPTION = RobotDescription.load(soarm.ROBOT_DIR)
CURRENT_POSE = dict(shoulder_pan=-12.83203125, shoulder_lift=-29.35546875, elbow_flex=11.77734375,
                    wrist_flex=51.767578125, wrist_roll=-13.0078125, gripper=99.12280701754386)
ACTUAL_HARDWARE = {"shoulder_pan": (-110.21484375, 64.599609375),
                   "shoulder_lift": (-101.162109375, 109.51171875),
                   "elbow_flex": (-105.556640625, 87.1875), "wrist_flex": (-108.28125, 97.822265625),
                   "wrist_roll": (-113.818359375, 95.185546875),
                   "gripper": (7.748538011695906, 218.71345029239765)}


class EncoderArm(Arm):
    """Exact deployed default encoder conversions; no bus is constructed."""
    def __init__(self):
        super().__init__()
        self.settings = FeetechSettings()
        self.gripper = "gripper"

    def _to_ticks(self, joint, value):
        return FeetechArm._to_ticks(self, joint, value)

    def _to_units(self, joint, value):
        return FeetechArm._to_units(self, joint, value)

    def quantize(self, goals):
        return {joint: self._to_units(joint, self._to_ticks(joint, value)) for joint, value in goals.items()}

    def send_joints(self, goals):
        self.pose = self.quantize(goals)
        self.writes.append(dict(self.pose))
        return dict(self.pose)


POSES = [DESCRIPTION.home, dict(DESCRIPTION.home, shoulder_pan=40.0, wrist_roll=30.0), CURRENT_POSE]


@pytest.mark.parametrize("pose", POSES, ids=["home", "pan40_roll30", "current"])
@pytest.mark.parametrize("profile", list(PROFILES))
@pytest.mark.parametrize("direction", ["tilt_up", "tilt_down"])
def test_quantized_tilt_only_moves_wrist_flex_and_allows_tip_arc(monkeypatch, pose, profile, direction):
    arm = EncoderArm()
    pose = arm.quantize(pose)
    controller, token, clock = make_controller(monkeypatch, pose, ACTUAL_HARDWARE, arm)
    solver = controller._cartesian_solver
    controller.start_cartesian(direction, profile, 0, controller._sequence, "recovery")
    assert controller.cartesian_state()["mode"] == "normal"
    assert not controller.arm.writes
    anchor = solver.position(pose)
    previous = dict(pose)
    previous_angle = 0.0
    completed = 0
    for sequence in range(1, 17):
        tick(controller, token, clock, sequence)
        if not controller.cartesian_state()["active"]:
            assert "boundary" in controller.clip or "limit" in controller.clip
            break
        completed += 1
        actual = controller.command
        assert (max(abs(actual[joint] - previous[joint]) for joint in actual)
                <= PROFILES[profile]["joint_deg_s"]*.05)
        assert all(controller.limits[joint][0] <= value <= controller.limits[joint][1]
                   for joint, value in actual.items())
        assert all(actual[joint] == pose[joint] for joint in actual if joint != "wrist_flex")
        angle = controller._cartesian_motion["achieved_angle_deg"]
        assert angle >= previous_angle - 1e-8
        previous, previous_angle = dict(actual), angle
    assert completed >= 4
    assert previous_angle > .25
    assert math.dist(solver.position(controller.command), anchor) > .001
    capabilities = controller.cartesian_state()["orientation_controls"]
    assert capabilities["tilt_tip_compensated"] is False
    assert "tilt_tip_tolerance_m" not in capabilities


@pytest.mark.parametrize("pose", [LIVE_POSE, CURRENT_POSE], ids=["previous_live", "current"])
@pytest.mark.parametrize("profile", list(PROFILES))
@pytest.mark.parametrize("direction,sign", [("roll_left", 1), ("roll_right", -1)])
def test_roll_only_moves_wrist_about_its_axis(monkeypatch, pose, profile, direction, sign):
    arm = EncoderArm()
    pose = arm.quantize(pose)
    controller, token, clock = make_controller(monkeypatch, pose, ACTUAL_HARDWARE, arm)
    controller.start_cartesian(direction, profile, 0, controller._sequence, "recovery")
    assert controller.cartesian_state()["mode"] == "normal"
    assert not controller.arm.writes
    previous = pose["wrist_roll"]
    for sequence in range(1, 17):
        tick(controller, token, clock, sequence)
        assert controller.cartesian_state()["active"], controller.clip
        actual = controller.command
        assert all(actual[joint] == pose[joint] for joint in actual if joint != "wrist_roll")
        assert sign * (actual["wrist_roll"] - previous) >= 0
        assert abs(actual["wrist_roll"] - previous) <= PROFILES[profile]["joint_deg_s"]*.05
        previous = actual["wrist_roll"]
    assert sign * (previous - pose["wrist_roll"]) > .5
    assert controller._cartesian_motion["tip_drift_m"] > .00001  # Arc is deliberately uncompensated.


def test_roll_positive_is_left_in_wrist_to_tip_view(monkeypatch):
    controller, _, _ = make_controller(monkeypatch, DESCRIPTION.home)
    solver = controller._cartesian_solver
    before = solver.kin.tcp(DESCRIPTION.home).copy()
    after = solver.kin.tcp(dict(DESCRIPTION.home, wrist_roll=5.0)).copy()
    forward = before[:3, :3] @ np.asarray(solver.kin.spec.tcp_approach)
    up = np.array([0., 0., 1.])
    up -= forward * float(up @ forward)
    up /= np.linalg.norm(up)
    right = np.cross(forward, up)
    closing_before = before[:3, :3] @ np.asarray(solver.kin.spec.tcp_closing)
    closing_after = after[:3, :3] @ np.asarray(solver.kin.spec.tcp_closing)
    a = math.atan2(float(closing_before @ right), float(closing_before @ up))
    b = math.atan2(float(closing_after @ right), float(closing_after @ up))
    assert math.degrees(solver._angle(b - a)) == pytest.approx(-5.0)
    assert controller.cartesian_state()["orientation_controls"]["roll_left_joint_sign"] == 1


def test_tilt_progress_unwraps_across_pi(monkeypatch):
    pose = dict(DESCRIPTION.home, shoulder_lift=-80.0, elbow_flex=-50.0, wrist_flex=-49.8)
    controller, token, clock = make_controller(monkeypatch, pose)
    controller._cartesian_workspace = Workspace(table_z_m=-1.0)
    controller.start_cartesian("tilt_up", "slow", 0, controller._sequence)
    assert controller._cartesian_solver.pitch(pose) > math.radians(179)
    for sequence in range(1, 13):
        tick(controller, token, clock, sequence)
        assert controller.cartesian_state()["active"], controller.clip
    assert controller._cartesian_solver.pitch(controller.command) < 0
    assert controller._cartesian_motion["achieved_angle_deg"] > .5


def test_direct_tilt_never_calls_compensated_position_ik(monkeypatch):
    controller, token, clock = make_controller(monkeypatch, DESCRIPTION.home)

    def forbidden(*_args, **_kwargs):
        raise AssertionError("Direct wrist tilt must not solve compensated IK.")

    monkeypatch.setattr(controller._cartesian_solver, "solve", forbidden)
    controller.start_cartesian("tilt_up", "fast", 0, controller._sequence)
    tick(controller, token, clock, 1)
    assert controller.cartesian_state()["active"]
    assert controller.command["wrist_flex"] < DESCRIPTION.home["wrist_flex"]
    assert all(controller.command[joint] == DESCRIPTION.home[joint] for joint in controller.command
               if joint != "wrist_flex")


@pytest.mark.parametrize("direction", ["tilt_up", "tilt_down", "roll_left", "roll_right"])
def test_orientation_subquantum_fractions_accumulate_without_backtracking(monkeypatch, direction):
    arm = EncoderArm()
    pose = arm.quantize(CURRENT_POSE)
    controller, token, clock = make_controller(monkeypatch, pose, ACTUAL_HARDWARE, arm)
    monkeypatch.setitem(PROFILES["slow"], "angular_deg_s", .1)
    controller.start_cartesian(direction, "slow", 0, controller._sequence)
    previous = 0.0
    for sequence in range(1, 61):
        tick(controller, token, clock, sequence)
        assert controller.cartesian_state()["active"], controller.clip
        motion = controller._cartesian_motion
        assert motion["achieved_angle_deg"] >= previous - 1e-8
        previous = motion["achieved_angle_deg"]
    assert motion["angle_deg"] > .28
    assert motion["achieved_angle_deg"] > .17


def test_roll_floor_checked_on_goal_before_write(monkeypatch):
    controller, token, clock = make_controller(monkeypatch, CURRENT_POSE, ACTUAL_HARDWARE)
    solver = controller._cartesian_solver
    z = solver.position(controller.measured)[2]
    left_z = solver.position(dict(controller.measured, wrist_roll=controller.measured["wrist_roll"] + .25))[2]
    direction = "roll_left" if left_z < z else "roll_right"
    controller._cartesian_workspace = Workspace(table_z_m=z - .0200001)
    controller.start_cartesian(direction, "fast", 0, controller._sequence)
    tick(controller, token, clock, 1)
    assert not controller.cartesian_state()["active"]
    assert "floor" in controller.clip.lower()
    assert all(goal == controller.measured for goal in controller.arm.writes)


@pytest.mark.parametrize("direction", ["tilt_up", "tilt_down", "roll_left", "roll_right"])
def test_orientation_old_queued_starts_and_expired_renewals_cannot_revive(monkeypatch, direction):
    controller, token, clock = make_controller(monkeypatch, CURRENT_POSE, ACTUAL_HARDWARE)
    run_command(controller, "cartesian/stop", {"token": token, "sequence": 2})
    body = {"token": token, "direction": direction, "sequence": 1,
            "observation_sequence": controller._sequence}
    with pytest.raises(PanelError, match="sequence"):
        run_command(controller, "cartesian/start", body)
    motion = run_command(controller, "cartesian/start", dict(body, sequence=3))["cartesian"]
    clock[0] += WATCHDOG_S + .01
    with pytest.raises(PanelError, match="ended"):
        run_command(controller, "cartesian/renew", {"token": token, "sequence": 4,
                                                   "motion_id": motion["motion_id"]})
    assert not controller.cartesian_state()["active"]


def test_orientation_invalid_start_and_latched_stop(monkeypatch):
    pose = dict(CURRENT_POSE, wrist_flex=-95.6)
    controller, token, _ = make_controller(monkeypatch, pose, ACTUAL_HARDWARE)
    with pytest.raises(PanelError, match="inside its limits|inside their limits"):
        controller.start_cartesian("tilt_up", "fast", 0, controller._sequence, "recovery")
    assert not controller.arm.writes
    controller.emergency_stop()
    with pytest.raises(PanelError, match="latched"):
        run_command(controller, "cartesian/start", {"token": token, "direction": "roll_left"})


BELOW_FLOOR_POSE = dict(shoulder_pan=38.583984375, shoulder_lift=-1.23046875, elbow_flex=42.802734375,
                       wrist_flex=19.072265625, wrist_roll=-24.169921875, gripper=99.1228070175)


@pytest.mark.parametrize("profile", list(PROFILES))
def test_reported_below_floor_pose_reenters_only_upward_and_relatches(monkeypatch, profile):
    arm = EncoderArm()
    pose = arm.quantize(BELOW_FLOOR_POSE)
    controller, token, clock = make_controller(monkeypatch, pose, ACTUAL_HARDWARE, arm)
    solver = controller._cartesian_solver
    floor = controller._cartesian_workspace.snapshot()["min_tcp_z_m"]
    assert solver.position(pose)[2] == pytest.approx(.010534258769351643)
    assert solver.position(pose)[2] < floor
    controller.start_cartesian("tilt_up", profile, 0, controller._sequence)
    assert controller.cartesian_state()["floor_reentry_active"]
    assert not arm.writes
    previous_z = solver.position(pose)[2]
    for sequence in range(1, 41):
        tick(controller, token, clock, sequence)
        assert controller.cartesian_state()["active"], controller.clip
        z = solver.position(controller.command)[2]
        assert z >= previous_z - 1e-9
        assert all(controller.command[joint] == pose[joint] for joint in pose if joint != "wrist_flex")
        previous_z = z
        if not controller.cartesian_state()["floor_reentry_active"]:
            break
    assert not controller.cartesian_state()["floor_reentry_active"]
    assert solver.position(controller.measured)[2] >= floor
    assert solver.position(controller.command)[2] >= floor
    # A later measured drop below the floor must use the restored normal guard,
    # not silently reopen the initial re-entry exemption.
    arm.pose = dict(pose)
    tick(controller, token, clock, sequence + 1)
    assert not controller.cartesian_state()["active"]
    assert "floor" in controller.clip.lower()


@pytest.mark.parametrize("direction", ["tilt_down", "roll_left", "roll_right", "left", "right"])
def test_below_floor_downward_and_unrelated_starts_are_refused(monkeypatch, direction):
    controller, _, _ = make_controller(monkeypatch, BELOW_FLOOR_POSE, ACTUAL_HARDWARE)
    with pytest.raises(PanelError, match="floor"):
        controller.start_cartesian(direction, "fast", 0, controller._sequence)
    assert not controller.arm.writes


def test_floor_reentry_noise_is_bounded_against_best_not_each_step(monkeypatch):
    arm = EncoderArm()
    pose = arm.quantize(BELOW_FLOOR_POSE)
    controller, token, clock = make_controller(monkeypatch, pose, ACTUAL_HARDWARE, arm)
    solver = controller._cartesian_solver
    controller.start_cartesian("tilt_up", "slow", 0, controller._sequence)
    tick(controller, token, clock, 1)
    tick(controller, token, clock, 2)
    state = controller._cartesian_motion["floor_reentry"]
    assert state is not None
    best_z = state["best_measured_z"]
    # One encoder tick of feedback rollback is ~0.134 mm here and fits the
    # fixed 0.2 mm noise band. Two rollback ticks must not gain another band.
    one_tick = 360 / 4096
    best_pose = dict(controller.measured)
    arm.pose = dict(best_pose, wrist_flex=best_pose["wrist_flex"] + one_tick)
    assert 0 < best_z - solver.position(arm.pose)[2] < .0002
    tick(controller, token, clock, 3)
    assert controller.cartesian_state()["active"]
    arm.pose = dict(best_pose, wrist_flex=best_pose["wrist_flex"] + 2 * one_tick)
    assert best_z - solver.position(arm.pose)[2] > .0002
    tick(controller, token, clock, 4)
    assert not controller.cartesian_state()["active"]
    assert "worsened" in controller.clip


def test_floor_reentry_without_measured_progress_times_out(monkeypatch):
    controller, token, clock = make_controller(monkeypatch, BELOW_FLOOR_POSE, ACTUAL_HARDWARE)
    controller.start_cartesian("tilt_up", "slow", 0, controller._sequence)
    controller._cartesian_motion["floor_reentry"]["last_progress_at"] -= 1.01
    tick(controller, token, clock, 1)
    assert not controller.cartesian_state()["active"]
    assert "no measured upward progress" in controller.clip


def test_renewal_preserves_actual_floor_stop_reason(monkeypatch):
    controller, token, clock = make_controller(monkeypatch, BELOW_FLOOR_POSE, ACTUAL_HARDWARE)
    controller.start_cartesian("tilt_up", "slow", 0, controller._sequence)
    motion_id = controller._cartesian_motion["motion_id"]
    controller._cartesian_motion["floor_reentry"]["last_progress_at"] -= 1.01
    tick(controller, token, clock, 1)
    reason = controller.cartesian_state()["stop_reason"]
    assert "no measured upward progress" in reason
    assert controller.cartesian_state()["stopped_motion_id"] == motion_id
    with pytest.raises(PanelError, match="no measured upward progress"):
        controller.renew_cartesian(motion_id, 2)
    controller.start_cartesian("tilt_up", "slow", 3, controller._sequence)
    assert controller.cartesian_state()["stop_reason"] is None
    assert controller.cartesian_state()["stopped_motion_id"] is None
