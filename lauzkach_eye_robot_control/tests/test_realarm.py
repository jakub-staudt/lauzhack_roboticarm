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

    def __init__(self, joints=None, owner=None, gripper=10.0, limits=None, estop=False, sag=None):
        self.m = {**(joints or START), "gripper": gripper}
        self.cmd = dict(self.m)                      # what was commanded; `m` is what is measured (command + sag)
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
