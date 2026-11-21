"""Hold-jog safety on a deterministic fake arm: never connects to hardware."""
from __future__ import annotations

import math

import pytest

import soarm
from soarm.panel import PanelController, PanelError, RobotDescription, run_command
from soarm.panel_cartesian import WATCHDOG_S, Workspace
from soarm.panel_preview import PreviewController

DESCRIPTION = RobotDescription.load(soarm.ROBOT_DIR)


class Arm:
    port = "FAKE"
    is_connected = True
    _stale = []

    def __init__(self):
        self.pose = {name: 0.0 for name in DESCRIPTION.joints}
        self.pose["gripper"] = 50.0
        self.writes = []

    def joint_limits(self):
        return {}

    def read_joints(self):
        return dict(self.pose)

    def faults(self):
        return []

    def set_joint_limits(self, _limits):
        pass

    def send_joints(self, goals):
        self.writes.append(dict(goals))
        self.pose = dict(goals)
        return dict(goals)


class Solver:
    """Locally invertible test geometry, no table or collision assumptions."""
    def position(self, joints):
        return (0.2 + joints["shoulder_lift"] / 100, 0.0, 0.2 + joints["elbow_flex"] / 100)

    def heading(self, joints):
        return (1.0, 0.0)

    def pitch(self, joints):
        return 0.0

    def _angle(self, value):
        return value

    def solve(self, joints, position, *, pitch=None):
        return dict(joints, shoulder_lift=(position[0] - 0.2) * 100,
                    elbow_flex=(position[2] - 0.2) * 100)


@pytest.fixture
def rig(monkeypatch):
    clock = [100.0]
    monkeypatch.setattr("soarm.panel.time.monotonic", lambda: clock[0])
    panel = PanelController(Arm(), DESCRIPTION)
    panel._cartesian_workspace = Workspace(table_z_m=-0.01)
    panel._cartesian_solver = Solver()
    claim = panel.claim("test")
    return panel, claim["token"], clock


def start(rig, direction="forward", sequence=0, **extra):
    panel, token, _ = rig
    return run_command(panel, "cartesian/start", {"token": token, "direction": direction,
                       "sequence": sequence, "observation_sequence": panel._sequence, **extra})["cartesian"]


@pytest.mark.parametrize("direction,joint,sign", [("forward", "shoulder_lift", 1),
                          ("back", "shoulder_lift", -1), ("up", "elbow_flex", 1),
                          ("down", "elbow_flex", -1), ("left", "shoulder_pan", -1),
                          ("right", "shoulder_pan", 1), ("open", "gripper", 1),
                          ("close", "gripper", -1)])
def test_single_direction_increment(rig, direction, joint, sign):
    panel, _, _ = rig
    before = dict(panel.command)
    start(rig, direction)
    assert panel.arm.writes == []  # Starting is an intent, not an immediate servo write.
    panel.tick(0.05)
    assert (panel.command[joint] - before[joint]) * sign > 0
    assert all(panel.command[name] == value for name, value in before.items() if name != joint)


def test_release_tombstones_pending_start_and_ended_motion(rig):
    panel, token, _ = rig
    run_command(panel, "cartesian/stop", {"token": token, "sequence": 2})
    with pytest.raises(PanelError, match="sequence"):
        start(rig, sequence=1)
    motion = start(rig, sequence=3)
    run_command(panel, "cartesian/stop", {"token": token, "motion_id": motion["motion_id"], "sequence": 5})
    with pytest.raises(PanelError):
        run_command(panel, "cartesian/renew",
                    {"token": token, "motion_id": motion["motion_id"], "sequence": 4})
    assert not panel.cartesian_state()["active"]
    assert panel.command == panel.measured == panel.target


def test_watchdog_is_independent_of_owner_and_terminal_before_tick(rig):
    panel, token, clock = rig
    motion = start(rig)
    clock[0] += WATCHDOG_S + 0.01
    panel.heartbeat(token)
    with pytest.raises(PanelError, match="ended"):
        run_command(panel, "cartesian/renew",
                    {"token": token, "motion_id": motion["motion_id"], "sequence": 1})
    assert not panel.cartesian_state()["active"]
    assert panel._token == token


def test_tick_watchdog_stops_without_accumulating_target(rig):
    panel, _, clock = rig
    start(rig)
    panel.tick(0.05)
    stopped = dict(panel.command)
    clock[0] += WATCHDOG_S + 0.01
    panel.tick(0.05)
    assert not panel.cartesian_state()["active"]
    assert panel.command == panel.target == stopped
    panel.tick(0.05)
    assert panel.command == panel.target == stopped


def test_renew_rejects_reordering_and_foreign_motion(rig):
    panel, token, _ = rig
    motion = start(rig)
    body = {"token": token, "motion_id": motion["motion_id"], "sequence": 2}
    run_command(panel, "cartesian/renew", body)
    with pytest.raises(PanelError, match="sequence"):
        run_command(panel, "cartesian/renew", dict(body, sequence=1))
    with pytest.raises(PanelError, match="ended"):
        run_command(panel, "cartesian/renew", dict(body, motion_id="wrong", sequence=3))


def test_start_rejects_old_observation(rig):
    panel, _, clock = rig
    clock[0] += 0.76
    with pytest.raises(PanelError, match="fresh observation"):
        start(rig)
    assert not panel.arm.writes


def test_only_one_direction_and_manual_model_exclusivity(rig):
    panel, token, _ = rig
    start(rig)
    with pytest.raises(PanelError, match="Release"):
        start(rig, "up", sequence=1)
    with pytest.raises(PanelError, match="Release"):
        run_command(panel, "jog", {"token": token, "joint": "elbow_flex", "delta": 1})
    with pytest.raises(PanelError, match="Release"):
        panel.arm_model(1, 1)


def test_workspace_required_for_spatial_but_gripper_available(rig):
    panel, _, _ = rig
    panel._cartesian_workspace = Workspace()
    assert panel.cartesian_state()["available"]
    assert not panel.cartesian_state()["spatial_available"]
    with pytest.raises(PanelError, match="Configure"):
        start(rig)
    start(rig, "open", sequence=1)
    panel.tick(0.05)
    assert panel.command["gripper"] > 50


@pytest.mark.parametrize("environment", [{"RAMION_CARTESIAN_TABLE_Z_M": "NaN"},
                         {"RAMION_CARTESIAN_TABLE_Z_M": "bad"},
                         {"RAMION_CARTESIAN_TCP_MARGIN_M": "-1"}])
def test_invalid_workspace(environment):
    assert Workspace.from_environment(environment).snapshot()["error"]
    assert not Workspace.from_environment(environment).snapshot()["configured"]


def test_floor_and_joint_limit_stop(rig):
    panel, _, _ = rig
    panel._cartesian_workspace = Workspace(table_z_m=0.1799)
    start(rig, "down")
    panel.tick(0.05)
    assert not panel.cartesian_state()["active"]
    assert panel.command["elbow_flex"] == 0
    panel._cartesian_workspace = Workspace(table_z_m=-0.01)
    panel.limits["shoulder_pan"] = (-1, 0.1)
    start(rig, "right", sequence=1)
    panel.tick(0.05)
    assert not panel.cartesian_state()["active"]
    assert panel.command["shoulder_pan"] == 0


@pytest.mark.parametrize("event", ["disconnect", "stale", "stop", "emergency", "release"])
def test_external_stop_conditions(rig, event):
    panel, token, _ = rig
    start(rig)
    if event == "disconnect":
        panel.arm.is_connected = False
        panel.tick(0.05)
    elif event == "stale":
        panel.arm._stale = ["shoulder_pan"]
        panel.tick(0.05)
    elif event == "stop":
        panel.stop()
    elif event == "emergency":
        panel.emergency_stop()
    else:
        panel.release(token)
    assert not panel.cartesian_state()["active"]
    assert panel._token is None
    assert panel.command == panel.measured


def test_owner_and_latched_stop_enforced(rig):
    panel, token, _ = rig
    with pytest.raises(PanelError):
        run_command(panel, "cartesian/start", {"token": "wrong"})
    panel.emergency_stop()
    with pytest.raises(PanelError, match="latched"):
        run_command(panel, "cartesian/start", {"token": token})


def test_profile_stays_bounded_after_large_dt(rig):
    panel, _, _ = rig
    start(rig, "open", profile="fast")
    panel.tick(100)
    assert math.isclose(panel.command["gripper"], 53.5)


def test_cold_solver_load_cannot_start_delayed_press(rig, monkeypatch):
    panel, _, clock = rig
    panel._cartesian_solver = None

    def slow_constructor(*_args):
        clock[0] += 0.8
        return Solver()

    monkeypatch.setattr("soarm.panel.CartesianSolver", slow_constructor)
    with pytest.raises(PanelError, match="refresh feedback"):
        start(rig)
    assert not panel.cartesian_state()["active"]
    assert not panel.arm.writes


@pytest.mark.parametrize("kind", ["partial", "clip"])
def test_partial_or_clipped_write_ends_motion(rig, monkeypatch, kind):
    panel, _, _ = rig
    start(rig)

    def refused(goals):
        return {} if kind == "partial" else dict(goals, shoulder_lift=10.0)

    monkeypatch.setattr(panel, "_send", refused)
    panel.tick(0.05)
    assert not panel.cartesian_state()["active"]
    assert panel.command == panel.measured


def test_tracking_error_ends_hold_even_with_fresh_feedback(rig):
    panel, _, _ = rig
    start(rig)
    panel.arm.pose["shoulder_lift"] = -5.1
    panel.tick(0.05)
    assert not panel.cartesian_state()["active"]
    assert panel._token is None


@pytest.mark.parametrize("profile", [[], {}, 12, None])
def test_malformed_profile_is_refused(rig, profile):
    with pytest.raises(PanelError):
        start(rig, profile=profile)


@pytest.mark.parametrize("direction", ["open", "close", "left", "right"])
def test_unchanged_verified_soft_overshoot_does_not_trap_other_controls(rig, direction):
    panel, _, _ = rig
    panel.hardware_limits["shoulder_lift"] = (-100.0, 100.0)
    panel.arm.pose["shoulder_lift"] = -95.6
    panel.measured = panel.arm.read_joints()
    panel._hold()
    start(rig, direction)
    panel.tick(0.05)
    assert panel.cartesian_state()["active"]
    assert panel.command["shoulder_lift"] == -95.6


def test_measured_eeprom_violation_never_starts_hold_jog(rig):
    panel, _, _ = rig
    panel.hardware_limits["shoulder_lift"] = (-95.0, 95.0)
    panel.arm.pose["shoulder_lift"] = -95.6
    panel.measured = panel.arm.read_joints()
    panel._hold()
    with pytest.raises(PanelError, match="EEPROM"):
        start(rig, "open")
    assert not panel.arm.writes


def test_slow_planning_never_sends_late_motion(rig, monkeypatch):
    panel, _, clock = rig
    start(rig)
    original = panel._cartesian_solver.solve

    def delayed(*args, **kwargs):
        clock[0] += WATCHDOG_S + 0.01
        return original(*args, **kwargs)

    monkeypatch.setattr(panel._cartesian_solver, "solve", delayed)
    panel.tick(0.05)
    assert not panel.cartesian_state()["active"]
    assert all(goals == panel.measured for goals in panel.arm.writes)


def test_preview_reports_configured_floor_without_enabling_motion(monkeypatch):
    monkeypatch.setenv("RAMION_CARTESIAN_TABLE_Z_M", "-0.0074")
    monkeypatch.setenv("RAMION_CARTESIAN_TCP_MARGIN_M", "0.02")
    preview = PreviewController(DESCRIPTION)
    cartesian = preview.state()["cartesian"]
    assert cartesian["workspace"]["configured"]
    assert cartesian["workspace"]["table_z_m"] == -0.0074
    assert cartesian["workspace"]["min_tcp_z_m"] == pytest.approx(0.0126)
    assert not cartesian["available"] and not cartesian["spatial_available"]
    assert preview.state()["measured"] == {}
    with pytest.raises(PanelError, match="unavailable"):
        run_command(preview, "cartesian/start", {"token": "test", "direction": "up",
                                                "profile": "slow", "sequence": 1,
                                                "observation_sequence": 0})


@pytest.fixture
def biased_rig(rig, monkeypatch):
    """Persistent servo tracking residual, distinct from the accepted bus command."""
    panel, _, _ = rig
    biases = {"shoulder_pan": -0.2, "shoulder_lift": 0.3, "elbow_flex": 0.4,
              "wrist_flex": -0.3, "wrist_roll": 0.2, "gripper": 0.5}
    panel.arm.pose["elbow_flex"] = 49.3
    panel.command = dict(panel.arm.pose)
    panel.target = dict(panel.command)
    monkeypatch.setattr(panel.arm, "read_joints", lambda: {
        joint: value + biases[joint] for joint, value in panel.arm.pose.items()})
    panel.measured = panel.arm.read_joints()
    return rig


@pytest.mark.parametrize("direction,moving_joint", [("open", "gripper"), ("close", "gripper"),
                                                    ("left", "shoulder_pan"), ("right", "shoulder_pan"),
                                                    ("tilt_up", "wrist_flex"), ("tilt_down", "wrist_flex"),
                                                    ("roll_left", "wrist_roll"),
                                                    ("roll_right", "wrist_roll")])
def test_repeated_unrelated_gestures_do_not_ratchet_feedback_bias(
        biased_rig, direction, moving_joint):
    panel, token, _ = biased_rig
    baseline = dict(panel.command)
    for sequence in range(0, 10, 2):
        motion = start(biased_rig, direction, sequence=sequence)
        panel.tick(0.05)
        accepted = dict(panel.command)
        assert panel.cartesian_state()["active"]
        assert accepted[moving_joint] != baseline[moving_joint]
        run_command(panel, "cartesian/stop", {"token": token, "motion_id": motion["motion_id"],
                                              "sequence": sequence + 1})
        assert panel.command == panel.target == accepted
        panel.tick(0.05)
        assert panel.command == panel.target == accepted
    assert panel.measured["elbow_flex"] == pytest.approx(49.7)
    assert all(write[joint] == value for write in panel.arm.writes
               for joint, value in baseline.items() if joint != moving_joint)


def test_press_release_without_tick_never_adopts_feedback(biased_rig):
    panel, token, _ = biased_rig
    baseline = dict(panel.command)
    for sequence in range(0, 10, 2):
        motion = start(biased_rig, "open", sequence=sequence)
        assert panel.command == panel.target == baseline
        run_command(panel, "cartesian/stop", {"token": token, "motion_id": motion["motion_id"],
                                              "sequence": sequence + 1})
        assert panel.command == panel.target == baseline
    assert all(write == baseline for write in panel.arm.writes)


def test_translation_anchors_command_and_discards_pending_manual_target(biased_rig):
    panel, token, _ = biased_rig
    baseline = dict(panel.command)
    run_command(panel, "target", {"token": token, "joint": "elbow_flex", "value": 60.0})
    assert panel.target["elbow_flex"] == 60.0
    start(biased_rig, "forward")
    assert panel.command == panel.target == baseline
    assert panel._cartesian_motion["anchor_position"] == panel._cartesian_solver.position(baseline)
    assert panel._cartesian_motion["anchor_position"] != panel._cartesian_solver.position(panel.measured)
    panel.tick(0.05)
    assert panel.cartesian_state()["active"]
    assert panel.command["shoulder_lift"] > baseline["shoulder_lift"]
    assert panel.command["elbow_flex"] == pytest.approx(baseline["elbow_flex"])


@pytest.mark.parametrize("ending", ["watchdog", "boundary"])
def test_ordinary_end_freezes_accepted_pose_despite_feedback_bias(biased_rig, ending):
    panel, _, clock = biased_rig
    start(biased_rig, "right")
    panel.tick(0.05)
    accepted = dict(panel.command)
    if ending == "watchdog":
        clock[0] += WATCHDOG_S + 0.01
    else:
        panel.limits["shoulder_pan"] = (-100.0, accepted["shoulder_pan"] + 0.01)
    panel.tick(0.05)
    assert not panel.cartesian_state()["active"]
    assert panel.command == panel.target == accepted
    assert panel.command != panel.measured
    panel.tick(0.05)
    assert panel.command == panel.target == accepted
    assert panel.arm.writes[-1] == accepted


def test_large_tracking_error_still_revokes_and_holds_measured(biased_rig):
    panel, _, _ = biased_rig
    start(biased_rig, "open")
    panel.arm.pose["elbow_flex"] += 5.1
    panel.tick(0.05)
    assert not panel.cartesian_state()["active"]
    assert panel._token is None
    assert panel.command == panel.target == panel.measured
    assert panel.arm.writes[-1] == panel.measured


@pytest.mark.parametrize("failure", ["partial", "clip", "nonfinite", "geometry"])
def test_invalid_physical_write_revokes_and_immediately_holds_feedback(biased_rig, monkeypatch, failure):
    panel, _, _ = biased_rig
    if failure == "geometry":
        original_position = panel._cartesian_solver.position

        def position_with_roll_offset(joints):
            x, y, z = original_position(joints)
            return x, y + joints["wrist_roll"] * 0.02, z

        monkeypatch.setattr(panel._cartesian_solver, "position", position_with_roll_offset)
    start(biased_rig, "forward" if failure == "geometry" else "open")
    expected_hold = panel.arm.read_joints()
    accepted_before_write = dict(panel.command)
    send = panel.arm.send_joints
    first = True

    def faulty_first_write(goals):
        nonlocal first
        if not first:
            return send(goals)
        first = False
        physical = dict(goals)
        if failure == "clip":
            physical["elbow_flex"] += 1.0
        elif failure == "geometry":
            # Within the per-joint backend tolerance, beyond the total path budget.
            physical["wrist_roll"] += 0.09
        sent = send(physical)
        if failure == "partial":
            sent.pop("elbow_flex")
        elif failure == "nonfinite":
            sent["elbow_flex"] = math.nan
        return sent

    monkeypatch.setattr(panel.arm, "send_joints", faulty_first_write)
    panel.tick(0.05)
    assert not panel.cartesian_state()["active"]
    assert panel._token is None
    assert panel.command == panel.target == panel.measured == expected_hold
    assert expected_hold != accepted_before_write
    assert len(panel.arm.writes) == 2
    assert panel.arm.writes[0] != expected_hold
    assert panel.arm.writes[1] == expected_hold
