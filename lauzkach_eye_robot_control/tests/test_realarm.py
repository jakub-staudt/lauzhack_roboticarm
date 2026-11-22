"""The real-arm adapter against a FAKE dashboard. Nothing here touches the real arm. Needs the robotic_arm environment (kinematics)."""

import json
import threading
import time
from pathlib import Path

import numpy as np
import pytest

pytest.importorskip("soarm_lab")
from robotic_arm.clickmove import MoveError  # noqa: E402
from soarm_lab.kinematics import RobotKinematics  # noqa: E402
from soarm_lab.robot import load_robot  # noqa: E402

from lauzkach_eye_robot_control.realarm import RealArmClient, RealSettings  # noqa: E402

KIN = RobotKinematics(load_robot())
POINTS = json.loads((Path(__file__).resolve().parents[2] / "robotic_arm" / "calibration_poses.json").read_text()) \
    if (Path(__file__).resolve().parents[2] / "robotic_arm" / "calibration_poses.json").exists() else None
pytestmark = pytest.mark.skipif(POINTS is None, reason="needs the recorded calibration points")
CAL = [(p["xyz_m"][0], p["xyz_m"][1]) for p in (POINTS or [])]
START = {k: float(v) for k, v in (POINTS[1]["joints"] if POINTS else {}).items()}   # resting at corner_01 (table level), the way the arm is left after recording it
LIMITS = {"shoulder_pan": [-105, 65], "shoulder_lift": [-95, 95], "elbow_flex": [-92, 87], "wrist_flex": [-95, 95], "wrist_roll": [-114, 95]}


class FakeDashboard:
    """The few dashboard routes the adapter uses. Targets are reached instantly; every call is recorded."""

    def __init__(self, joints=None, owner=None, gripper=10.0, limits=None, estop=False, sag=None, gripper_floor=7.7):
        self.m = {**(joints or START), "gripper": gripper}
        self.cmd = dict(self.m)                      # what was commanded; `m` is what is measured (command + sag)
        self.gripper_floor = gripper_floor           # the jaws cannot close below this (49.4 with a block between them)
        self.sag = sag or {}                         # degrees of gravity sag per joint, added to the measurement
        self.owner, self.token, self.estop, self.limits = owner, None, estop, limits or LIMITS
        self.targets, self.calls, self.beats, self.stops = [], [], 0, 0

    def __call__(self, method, path, body=None):
        self.calls.append((method, path))
        if path == "/api/state":
            return {"connected": True, "torque": True, "emergency_stop": {"latched": self.estop, "generation": 3},
                    "control": {"owner_id": self.owner, "label": "someone"}, "cartesian": {"active": False, "workspace": {"min_tcp_z_m": -0.0024},
                    "joint_limits": {"effective": self.limits}}, "measured": dict(self.m), "command": dict(self.cmd)}
        if path == "/api/control/claim":
            if self.owner is not None:
                raise MoveError("Control is held by someone else.")
            self.owner, self.token = "me", "tok"
            return {"token": "tok", "owner_id": "me"}
        if path == "/api/control/heartbeat":
            self.beats += 1
            return {}
        if path == "/api/control/release":
            self.owner = self.token = None
            return {}
        if path == "/api/target":
            assert body["token"] == "tok" and self.owner == "me"
            self.targets.append((body["joint"], body["value"]))
            self.cmd[body["joint"]] = body["value"]
            self.m[body["joint"]] = body["value"] + self.sag.get(body["joint"], 0.0)
            if body["joint"] == "gripper":
                self.m["gripper"] = max(body["value"], self.gripper_floor)
            return {}
        if path == "/api/stop":
            self.stops += 1
            self.owner = self.token = None
            return {}
        raise AssertionError(path)


def client(dash, **settings):
    return RealArmClient(dash, KIN, CAL, z_table=-0.003, settings=RealSettings(pause_s=0.0, **settings))


def finish(c, timeout=60.0):
    end = time.time() + timeout
    while c.mover.busy() and time.time() < end:
        time.sleep(0.02)
    assert not c.mover.busy()


def corner(i):
    p = POINTS[i]["xyz_m"]
    return p[0], p[1]


def tip(joints):
    return KIN.tcp({k: float(v) for k, v in joints.items()})[:3, 3]


def test_a_hover_move_ends_3_cm_above_the_corner_travels_high_and_releases_control():
    dash = FakeDashboard()
    c = client(dash)
    x, y = corner(0)
    c.go_to_base(x, y, 3.0)
    finish(c)
    assert c.mover.status()["state"] == "done" and dash.owner is None                   # control released after the move
    end = tip(dash.m)
    assert np.hypot(end[0] - x, end[1] - y) < 0.004 and abs(end[2] - (-0.003 + 0.03)) < 0.004
    waypoints = {}
    for i, (joint, value) in enumerate(dash.targets):                                    # rebuild each commanded waypoint
        waypoints.setdefault(i // 5, {})[joint] = value
    start = tip(START)
    positions = [tip({**START, **w}) for w in waypoints.values()]
    travelling = [p for p in positions if np.hypot(p[0] - start[0], p[1] - start[1]) > 0.02 and np.hypot(p[0] - x, p[1] - y) > 0.02]
    assert travelling and min(p[2] for p in travelling) > -0.003 + 0.05 - 0.016         # all horizontal travel is at the 5 cm glide height
    assert min(p[2] for p in positions[len(positions) // 2:]) > -0.003 + 0.03 - 0.004   # and the second half never goes below 3 cm


def test_no_waypoint_moves_a_joint_by_more_than_3_degrees():
    dash = FakeDashboard()
    c = client(dash)
    c.go_to_base(*corner(1), 3.0)
    finish(c)
    waypoints, current = {}, []
    for i, (joint, value) in enumerate(dash.targets):
        waypoints.setdefault(i // 5, {})[joint] = value
    previous = {k: START[k] for k in START if k != "gripper"}
    for w in waypoints.values():
        assert max(abs(w[j] - previous[j]) for j in w) <= 3.0 + 1e-6
        previous = w


@pytest.mark.parametrize("height", [1.0, 2.9])
def test_the_real_arm_never_goes_below_3_cm(height):
    dash = FakeDashboard()
    with pytest.raises(MoveError, match="at least 3 cm"):
        client(dash).go_to_base(*corner(0), height)
    assert dash.targets == [] and ("POST", "/api/control/claim") not in dash.calls


def test_a_point_outside_the_calibrated_area_is_refused():
    dash = FakeDashboard()
    with pytest.raises(MoveError, match="outside the area"):
        client(dash).go_to_base(0.40, 0.30, 3.0)
    assert dash.targets == []


def test_an_open_gripper_someone_elses_control_or_a_latched_estop_stop_the_move_before_it_starts():
    for dash, message in ((FakeDashboard(gripper=80.0), "gripper"), (FakeDashboard(owner="x"), "holds control"), (FakeDashboard(estop=True), "Emergency")):
        with pytest.raises(MoveError, match=message):
            client(dash).go_to_base(*corner(0), 3.0)
        assert dash.targets == []                                                         # nothing was commanded
        if dash.owner == "me":
            pytest.fail("the control lease was left claimed")


def test_a_path_that_would_leave_the_controllers_joint_limits_is_refused_and_control_released():
    limits = {**LIMITS, "shoulder_lift": [-95, 20]}                                       # the arm rests at about 60 degrees: far outside this
    dash = FakeDashboard(limits=limits)
    with pytest.raises(MoveError, match="shoulder_lift"):
        client(dash).go_to_base(*corner(0), 3.0)
    assert dash.targets == [] and dash.owner is None


def test_stop_cancels_the_move_holds_the_arm_and_releases_control():
    dash = FakeDashboard()
    c = RealArmClient(dash, KIN, CAL, z_table=-0.003, settings=RealSettings(pause_s=0.2))   # a slower pace so there is time to stop
    c.go_to_base(*corner(2), 3.0)
    time.sleep(0.5)
    c.stop()
    finish(c)
    assert dash.stops >= 1 and dash.owner is None
    assert c.state()["task"]["state"] == "stopped"
    n = len(dash.targets)
    time.sleep(0.3)
    assert len(dash.targets) == n                                                          # nothing more is sent after the Stop


def test_state_reports_the_tip_in_the_arms_frame():
    dash = FakeDashboard()
    st = client(dash).state()
    assert st["connected"] and st["task"]["state"] == "idle" and st["emergency_stop"] is False
    assert np.allclose([st["tip_base"][k] for k in "xyz"], tip(START), atol=1e-9)


def test_a_move_can_start_from_the_table_where_the_corners_were_recorded():
    """After recording a corner the arm rests at table level; the rise must be gradual, not one big jump."""
    rest = {k: float(v) for k, v in POINTS[0]["joints"].items()}                       # the pose recorded at corner_00
    dash = FakeDashboard(joints=rest)
    c = client(dash)
    c.go_to_base(*corner(3), 3.0)
    finish(c)
    assert c.mover.status()["state"] == "done"
    end = tip(dash.m)
    assert np.hypot(end[0] - corner(3)[0], end[1] - corner(3)[1]) < 0.004 and abs(end[2] - 0.027) < 0.004
    first = {j: v for j, v in dash.targets[:5]}
    assert tip({**rest, **first})[2] - tip(rest)[2] < 0.012                              # the first step up is small (about 1 cm or less)


def test_control_is_claimed_only_after_planning_and_a_heartbeat_follows_at_once():
    """A lease lasts 1.5 s without a heartbeat and planning takes seconds: claiming first made the real arm stop after one step."""
    dash = FakeDashboard()
    c = client(dash)
    c.go_to_base(*corner(0), 3.0)
    finish(c)
    paths = [p for _, p in dash.calls]
    i = paths.index("/api/control/claim")
    assert paths[i + 1] == "/api/control/heartbeat" and "/api/target" not in paths[:i]


def test_the_fast_planner_reaches_the_same_pose_as_the_slow_one_in_a_fraction_of_the_time_and_never_dips():
    from robotic_arm.clickmove import ClickSettings

    from lauzkach_eye_robot_control.realarm import plan_hover, plan_hover_fast
    cs = ClickSettings(above_m=0.05, step_m=0.01)
    start = {k: float(v) for k, v in POINTS[0]["joints"].items()}
    x, y = corner(2)
    t0 = time.time(); slow = plan_hover(KIN, cs, start, (x, y), -0.003, -0.0024, 0.03, 5.0); t_slow = time.time() - t0
    t0 = time.time(); fast = plan_hover_fast(KIN, cs, start, (x, y), -0.003, -0.0024, 0.03, 5.0); t_fast = time.time() - t0
    assert np.allclose(tip({**start, **fast[-1].joints}), tip({**start, **slow[-1].joints}), atol=0.002)
    assert t_fast < t_slow / 3
    previous = start
    for w in fast:
        assert max(abs(w.joints[j] - previous[j]) for j in w.joints) <= 5.0 + 1e-6
        previous = w.joints
    assert min(tip({**start, **w.joints})[2] for w in fast) > tip(start)[2] - 0.011     # never dips below where it started


def test_the_fast_planner_refuses_a_path_that_would_dip_and_the_client_falls_back_to_the_slow_one(monkeypatch):
    import lauzkach_eye_robot_control.realarm as ra
    monkeypatch.setattr(ra, "plan_hover_fast", lambda *a, **k: (_ for _ in ()).throw(MoveError("forced")))
    dash = FakeDashboard()
    c = client(dash)
    c.go_to_base(*corner(0), 3.0)
    finish(c)
    assert c.mover.status()["state"] == "done"


def test_sag_is_compensated_so_the_measured_tip_lands_near_the_target():
    """The arm rests lower than commanded under gravity: the last step looks at the measured tip and shifts the command."""
    sag = {"shoulder_lift": 4.0}                                                     # about 9 mm of tip error
    dash = FakeDashboard(sag=sag)
    c = client(dash)
    x, y = corner(0)
    c.go_to_base(x, y, 3.0)
    finish(c)
    assert c.mover.status()["state"] == "done", c.mover.status()
    end = tip(dash.m)
    assert abs(end[2] - (-0.003 + 0.03)) < 0.006 and np.hypot(end[0] - x, end[1] - y) < 0.006
    uncompensated = FakeDashboard(sag=sag)
    c2 = client(uncompensated)
    c2.mover.trim = False
    c2.go_to_base(x, y, 3.0)
    finish(c2)
    assert abs(tip(uncompensated.m)[2] - 0.027) > abs(end[2] - 0.027)               # compensation made it better


def test_a_move_is_stopped_if_the_measured_tip_sags_below_the_height_limit():
    dash = FakeDashboard(sag={"shoulder_lift": 14.0})                                # absurd sag: the tip ends far too low
    c = client(dash)
    c.settings.max_step_deg = 3.0
    c.mover.settings.max_sag_deg = 30.0                                              # do not let the "blocked joint" check catch it first
    c.go_to_base(*corner(0), 3.0)
    finish(c)
    st = c.mover.status()
    assert st["state"] == "error" and "sags" in st["message"] and dash.stops >= 1 and dash.owner is None


# ------------------------------------------------------------------------------------------------ the staged real grasp
def grasp_rig(**kw):
    rest = {k: float(v) for k, v in POINTS[0]["joints"].items()}                    # resting at corner_00: the gripper points nearly straight down
    dash = FakeDashboard(joints=rest, gripper=10.0, **kw)
    c = client(dash)
    return dash, c, rest


def run_stage(c, action, **kw):
    c.grasp_action(action, **kw)
    finish(c, 90.0)
    return c.mover.status()


def block_xy(rest, dx=0.03, dy=-0.02):
    t = tip(rest)
    return float(t[0] + dx), float(t[1] + dy)


def test_stages_must_follow_each_other():
    dash, c, rest = grasp_rig()
    with pytest.raises(MoveError, match="Not now"):
        c.grasp_action("lower")
    assert c.state()["grasp"]["next"] == ["open"] and dash.targets == []


def test_open_hover_lower_grip_lift_putback_for_a_block_that_stalls_the_jaws_at_49():
    dash, c, rest = grasp_rig(gripper_floor=49.4)
    x, y = block_xy(rest)
    assert run_stage(c, "open")["state"] == "done" and dash.m["gripper"] == pytest.approx(80.0, abs=1.5)
    assert c.state()["grasp"]["stage"] == "open"
    assert run_stage(c, "hover", x=x, y=y)["state"] == "done"
    g = c.state()["grasp"]
    assert g["stage"] == "hover" and np.hypot(*(np.array(g["block"]) - [x, y])) < 1e-9
    t = tip(dash.m)
    assert abs(t[2] - (-0.003 + 0.062)) < 0.006                                     # hovering about 6 cm above the table
    aim_offset = np.hypot(t[0] - x, t[1] - y)
    assert 0.004 < aim_offset < 0.014                                               # the tool point is aimed a few mm off the block's centre
    assert run_stage(c, "lower")["state"] == "done"
    assert abs(tip(dash.m)[2] - (-0.003 + 0.0185)) < 0.006                         # tool point a little below the block's mid-height
    st = run_stage(c, "close")
    assert st["state"] == "done" and "Holding a block" in st["message"], st
    g = c.state()["grasp"]
    assert g["stage"] == "closed" and g["hold_value"] == pytest.approx(49.4, abs=1.5)
    assert dash.cmd["gripper"] == pytest.approx(g["hold_value"] - 1.5, abs=1.0)    # squeezed a little
    z0 = tip(dash.m)[2]
    assert run_stage(c, "lift")["state"] == "done" and tip(dash.m)[2] - z0 > 0.05
    assert c.state()["grasp"]["stage"] == "lifted" and c.state()["grasp"]["next"] == ["putback"]
    assert run_stage(c, "putback")["state"] == "done"
    assert dash.m["gripper"] == pytest.approx(80.0, abs=1.5) and c.state()["grasp"]["stage"] == "hover"
    assert dash.owner is None


def test_closing_on_nothing_is_a_miss_and_the_gripper_opens_again():
    dash, c, rest = grasp_rig(gripper_floor=10.7)
    x, y = block_xy(rest)
    run_stage(c, "open")
    run_stage(c, "hover", x=x, y=y)
    run_stage(c, "lower")
    st = run_stage(c, "close")
    assert st["state"] == "error" and "Missed" in st["message"]
    assert dash.m["gripper"] == pytest.approx(80.0, abs=1.5)
    g = c.state()["grasp"]
    assert g["stage"] == "miss" and g["next"] == ["open"]


def test_hover_needs_an_open_gripper_a_downward_pointing_tool_and_a_point_inside_the_calibrated_area():
    dash, c, rest = grasp_rig()
    x, y = block_xy(rest)
    run_stage(c, "open")
    with pytest.raises(MoveError, match="outside the area"):
        c.grasp_action("hover", x=0.9, y=0.9)
    with pytest.raises(MoveError, match="Click the block"):
        c.grasp_action("hover")
    assert dash.targets and dash.owner is None                                      # (nothing else was refused above: the earlier cases never moved)


def test_hover_with_the_gripper_closed_is_refused():
    dash, c, rest = grasp_rig()
    c.grasp = {"stage": "open", "pending": None}                                    # pretend the open stage was done, the jaws are still closed
    with pytest.raises(MoveError, match="Open the gripper"):
        c.grasp_action("hover", x=block_xy(rest)[0], y=block_xy(rest)[1])


def test_the_lowering_stops_if_a_joint_cannot_follow_its_command():
    dash, c, rest = grasp_rig(gripper_floor=49.4)
    x, y = block_xy(rest)
    run_stage(c, "open")
    run_stage(c, "hover", x=x, y=y)
    dash.sag = {"shoulder_lift": 5.5}                                               # something resists: the joint rests 5.5 degrees behind
    st = run_stage(c, "lower")
    assert st["state"] == "error" and c.state()["grasp"]["stage"] == "hover" and dash.owner is None


def test_stop_during_a_grasp_stage_halts_and_releases():
    dash, c, rest = grasp_rig(gripper_floor=49.4)
    x, y = block_xy(rest)
    run_stage(c, "open")
    c.settings.pause_s = c.mover.pause_s = 0.3
    c.grasp_action("hover", x=x, y=y)
    time.sleep(0.6)
    c.stop()
    finish(c, 30.0)
    assert dash.stops >= 1 and dash.owner is None and c.state()["grasp"]["stage"] == "open"


def test_a_refused_plain_move_does_not_reset_the_grasp_stage_but_a_started_one_does():
    dash, c, rest = grasp_rig()
    run_stage(c, "open")
    assert c.state()["grasp"]["stage"] == "open"
    with pytest.raises(MoveError, match="at least 3 cm"):
        c.go_to_base(*block_xy(rest), 1.0)                                          # refused before anything starts
    assert c.state()["grasp"]["stage"] == "open"
    dash.m["gripper"] = 10.0                                                         # a plain move needs the gripper closed
    dash.cmd["gripper"] = 10.0
    c.go_to_base(*block_xy(rest), 3.0)
    finish(c, 90.0)
    assert c.state()["grasp"]["stage"] == "idle"


def test_a_block_that_is_visibly_held_counts_as_gripped_even_if_the_close_step_ended_in_an_error():
    """The controller dropped control during a close once ("Arm fault; holding the arm") although the jaws were holding the block."""
    dash, c, rest = grasp_rig(gripper_floor=46.8)
    x, y = block_xy(rest)
    run_stage(c, "open")
    run_stage(c, "hover", x=x, y=y)
    run_stage(c, "lower")
    assert c.state()["grasp"]["stage"] == "lower"
    dash.cmd["gripper"] = 44.0
    dash.m["gripper"] = 46.8                                                        # the jaws stalled on the block, the command is ahead
    g = c.state()["grasp"]
    assert g["stage"] == "closed" and g["hold_value"] == pytest.approx(46.8) and g["next"] == ["lift", "open"]
    assert run_stage(c, "lift")["state"] == "done" and c.state()["grasp"]["stage"] == "lifted"


def test_an_open_gripper_is_never_taken_for_a_held_block():
    dash, c, rest = grasp_rig(gripper_floor=7.7)
    c.grasp = {"stage": "lower", "pending": None}
    dash.m["gripper"], dash.cmd["gripper"] = 79.0, 80.0
    assert c.state()["grasp"]["stage"] == "lower"
    dash.m["gripper"], dash.cmd["gripper"] = 10.7, 10.7                              # closed on nothing
    assert c.state()["grasp"]["stage"] == "lower"
    dash.m["gripper"], dash.cmd["gripper"] = 50.0, 50.0                              # a rest at a middle value with no push: not a grip
    assert c.state()["grasp"]["stage"] == "lower"


def test_a_block_held_when_the_agent_starts_is_recognised_so_lift_is_offered():
    dash, c, rest = grasp_rig(gripper_floor=46.8)
    assert c.state()["grasp"]["stage"] == "idle"
    dash.cmd["gripper"], dash.m["gripper"] = 44.0, 46.8                              # holding a block from before the agent was restarted
    g = c.state()["grasp"]
    assert g["stage"] == "closed" and g["next"] == ["lift", "open"]


def test_a_lift_that_does_not_raise_the_tip_is_an_error_and_a_sagging_lift_is_corrected():
    sag = {"shoulder_lift": 4.0}
    dash, c, rest = grasp_rig(gripper_floor=49.4, sag=sag)
    x, y = block_xy(rest)
    for action in ("open",):
        run_stage(c, action)
    run_stage(c, "hover", x=x, y=y)
    run_stage(c, "lower")
    run_stage(c, "close")
    z0 = tip(dash.m)[2]
    st = run_stage(c, "lift")
    assert st["state"] == "done", st
    assert tip(dash.m)[2] - z0 > 0.05                                               # the sag was compensated: it really rose about 3 cm
    # a joint that cannot follow (stuck) leaves the tip where it is: the stage must fail, not say "lifted"
    dash2, c2, rest2 = grasp_rig(gripper_floor=49.4)
    x2, y2 = block_xy(rest2)
    for action in ("open",):
        run_stage(c2, action)
    run_stage(c2, "hover", x=x2, y=y2)
    run_stage(c2, "lower")
    run_stage(c2, "close")
    orig = dash2.__call__.__func__

    def stuck(self_, method, path, body=None):
        if path == "/api/target" and body["joint"] == "shoulder_lift" and body["value"] < dash2.cmd["shoulder_lift"]:
            self_.cmd[body["joint"]] = body["value"]                                  # the command moves, the joint does not
            return {}
        return orig(self_, method, path, body)
    import types
    dash2.__call__ = types.MethodType(stuck, dash2)
    c2.mover.http = lambda m, p, b=None: stuck(dash2, m, p, b)
    c2.http = c2.mover.http
    st2 = run_stage(c2, "lift")
    assert st2["state"] == "error" and ("did not lift" in st2["message"] or "blocked" in st2["message"]), st2
    assert c2.state()["grasp"]["stage"] == "closed"


def test_hover_chooses_its_own_tilt_so_a_steep_or_shallow_gripper_does_not_have_to_be_tilted_by_hand():
    for dwf in (+12.0, -14.0):                                                      # bent too far / too shallow compared with the rest pose
        rest0 = {k: float(v) for k, v in POINTS[0]["joints"].items()}
        steep = {**rest0, "wrist_flex": rest0["wrist_flex"] + dwf}
        dash = FakeDashboard(joints=steep, gripper=10.0)
        c = client(dash)
        x, y = block_xy(steep, dx=0.02, dy=-0.02)
        run_stage(c, "open")
        st = run_stage(c, "hover", x=x, y=y)
        assert st["state"] == "done", (dwf, st)
        from soarm_lab.kinematics import RobotKinematics  # noqa: F401
        approach, _ = KIN.tool_axes({k: float(v) for k, v in dash.m.items()})
        pitch = float(np.degrees(np.arcsin(-approach[2])))
        assert 64.0 < pitch < 79.0, pitch
        assert dash.owner is None and c.state()["grasp"]["stage"] == "hover"



def test_the_saved_grasp_profiles_load_and_the_elite_one_aims_23_mm_behind_the_block_with_a_wide_opening():
    from lauzkach_eye_robot_control.realarm import GraspSettings
    path = str(Path(__file__).resolve().parents[1] / "grasp_profiles.json")
    cur, elite = GraspSettings.from_profile(path, "current"), GraspSettings.from_profile(path, "sim_elite")
    assert cur.pick_offset_m == pytest.approx(0.0084, abs=1e-4) and cur.open_value == 80.0 and cur.grasp_tip_m == pytest.approx(0.0185)
    assert elite.pick_offset_m == pytest.approx(0.02303) and elite.open_value == 92.0 and elite.grasp_tip_m == pytest.approx(0.02867)
    assert elite.profile == "sim_elite"
    with pytest.raises(ValueError, match="no grasp profile"):
        GraspSettings.from_profile(path, "nope")


def test_the_elite_profile_runs_the_whole_staged_grasp_against_the_fake_controller():
    from lauzkach_eye_robot_control.realarm import GraspSettings
    path = str(Path(__file__).resolve().parents[1] / "grasp_profiles.json")
    rest = {k: float(v) for k, v in POINTS[0]["joints"].items()}
    dash = FakeDashboard(joints=rest, gripper=10.0, gripper_floor=49.4)
    c = RealArmClient(dash, KIN, CAL, z_table=-0.003, settings=RealSettings(pause_s=0.0), grasp_settings=GraspSettings.from_profile(path, "sim_elite"))
    x, y = block_xy(rest)
    assert run_stage(c, "open")["state"] == "done" and dash.m["gripper"] == pytest.approx(92.0, abs=1.5)
    assert run_stage(c, "hover", x=x, y=y)["state"] == "done"
    t = tip(dash.m)
    assert 0.018 < np.hypot(t[0] - x, t[1] - y) < 0.03                              # aimed about 23 mm off the block's centre
    assert run_stage(c, "lower")["state"] == "done" and abs(tip(dash.m)[2] - (-0.003 + 0.02867)) < 0.006
    assert run_stage(c, "close")["state"] == "done" and c.state()["grasp"]["profile"] == "sim_elite"
