import numpy as np
import pytest

from soarm_lab.kinematics import RobotKinematics
from soarm_lab.robot import load_robot

from robotic_arm.clickmove import ClickSettings, ImagePlane, MoveError, Mover, plan_click
from robotic_arm.planner import ARM


@pytest.fixture(scope="module")
def kin():
    return RobotKinematics(load_robot())


def make_plane(kin, tmp_path, n=5):
    """Pretend the user placed the tip on spots of a table patch and clicked them (image 640x480 normalised)."""
    plane = ImagePlane(tmp_path / "plane.json")
    seed = dict(kin.spec.home)
    spots = [(0.2, 0.2), (0.8, 0.2), (0.8, 0.8), (0.2, 0.8), (0.5, 0.5)][:n]
    for px, py in spots:
        pos = np.array([0.17 + 0.16 * py, -0.07 + 0.14 * px, 0.0])  # image x -> table y, image y -> table x
        sol = kin.ik(pos, seed=seed)
        assert sol.ok
        seed = sol.joints
        plane.add(px, py, list(kin.tcp(sol.joints)[:3, 3]), sol.joints)
    return plane


class FakeController:
    """Joint targets are reached instantly; records every commanded target."""

    def __init__(self, joints, owner="abc", gripper=10.0, sag=None):
        self.m = {**joints, "gripper": gripper}
        self.sag = sag or {}  # steady offset of the measurement from the command, like gravity load
        self.owner, self.targets, self.beats, self.stopped = owner, [], 0, False

    def __call__(self, method, path, body=None):
        if path == "/api/state":
            return {"connected": True, "torque": True, "emergency_stop": {"latched": False, "generation": 1},
                    "control": {"owner_id": self.owner}, "cartesian": {"active": False, "workspace": {"min_tcp_z_m": -0.0024}},
                    "measured": {j: v + self.sag.get(j, 0.0) for j, v in self.m.items()},
                    "command": dict(self.m), "sequence": 1}
        if path == "/api/target":
            self.targets.append((body["joint"], body["value"])); self.m[body["joint"]] = body["value"]; return {}
        if path == "/api/control/heartbeat":
            self.beats += 1; return {}
        if path == "/api/stop":
            self.stopped = True; return {}
        raise AssertionError(path)


def wait(mover):
    mover._thread.join(10)
    return mover.status()


def test_plane_xy_matches_spots(kin, tmp_path):
    plane = make_plane(kin, tmp_path)
    for p in plane.pairs:
        assert np.linalg.norm(np.array(plane.xy(p["px"], p["py"])) - p["xyz"][:2]) < 2e-3
    assert plane.summary()["checked"] and plane.summary()["max_mm"] < 3


def test_four_spots_are_ready_but_unchecked(kin, tmp_path):
    plane = make_plane(kin, tmp_path, n=4)
    assert plane.ready and not plane.summary()["checked"]


def test_click_sequence_goes_above_down_up(kin, tmp_path):
    plane = make_plane(kin, tmp_path)
    s = ClickSettings()
    ctl = FakeController(kin.spec.home)
    mover = Mover(ctl, kin, plane, s)
    mover.start_click("tok", 0.4, 0.6)
    st = wait(mover)
    assert st["state"] == "done", st
    # reconstruct tip heights from the commanded arm joints, waypoint by waypoint
    per_wp = [ctl.targets[i:i + 5] for i in range(0, len(ctl.targets), 5)]
    zs = []
    for group in per_wp:
        joints = {**ctl.m, **{j: v for j, v in group}}
        zs.append(kin.tcp(joints)[2, 3] - plane.z_table)
    assert abs(zs[-1] - s.above_m) < 0.004               # ends 25 mm above the table
    assert abs(min(zs) - s.down_offset_m) < 0.004        # lowest point is just above the table
    assert min(zs[:-2]) > s.above_m - 0.004             # everything before the final down/up stays at hover height or higher
    assert ctl.beats > 0
    assert all(j in ARM for j, _ in ctl.targets)          # the gripper is never commanded
    xy_end = kin.tcp({**ctl.m})[:2, 3]
    assert np.linalg.norm(xy_end - np.array(plane.xy(0.4, 0.6))) < 0.005


def test_refuses_open_gripper_and_outside_area(kin, tmp_path):
    plane = make_plane(kin, tmp_path)
    mover = Mover(FakeController(kin.spec.home, gripper=80.0), kin, plane)
    with pytest.raises(MoveError, match="Close the gripper"):
        mover.start_click("tok", 0.4, 0.6)
    mover = Mover(FakeController(kin.spec.home), kin, plane)
    with pytest.raises(MoveError, match="outside"):
        mover.start_click("tok", 1.5, 0.5)
    far = (0.0, 0.0)  # image corner, well outside the patch the spots cover
    assert not plane.region_contains(*plane.xy(*far), 0.03)
    with pytest.raises(MoveError, match="outside the calibrated area"):
        mover.start_click("tok", *far)


def test_needs_calibration_and_control(kin, tmp_path):
    empty = ImagePlane(tmp_path / "empty.json")
    with pytest.raises(MoveError, match="Calibrate first"):
        Mover(FakeController(kin.spec.home), kin, empty).start_click("t", 0.5, 0.5)
    plane = make_plane(kin, tmp_path)
    with pytest.raises(MoveError, match="Take control"):
        Mover(FakeController(kin.spec.home, owner=None), kin, plane).start_click("t", 0.5, 0.5)


def test_waypoints_respect_floor_and_limits(kin, tmp_path):
    plane = make_plane(kin, tmp_path)
    wps = plan_click(kin, ClickSettings(), dict(kin.spec.home), plane.xy(0.5, 0.5), plane.z_table, -0.0024)
    assert [w.name for w in wps][-2:] == ["down", "up"]
    assert min(w.xyz[2] for w in wps) >= -0.0024 + 0.001
    for w in wps:
        for j in ARM:
            lo, hi = kin.spec.limits[j]
            assert lo <= w.joints[j] <= hi


def test_cancel_stops_and_reports(kin, tmp_path):
    plane = make_plane(kin, tmp_path)
    ctl = FakeController(kin.spec.home)
    mover = Mover(ctl, kin, plane)
    mover.start_click("tok", 0.4, 0.6)
    mover.cancel()
    mover._thread.join(10)
    assert mover.status()["state"] in ("error", "done")


def test_nudge_up_and_down(kin, tmp_path):
    plane = make_plane(kin, tmp_path)
    start = dict(kin.spec.home)
    tip0 = kin.tcp(start)[:3, 3]
    ctl = FakeController(start, gripper=80.0)           # nudges work whatever the gripper is doing
    mover = Mover(ctl, kin, plane)
    mover.start_nudge("tok", 0.010)
    assert wait(mover)["state"] == "done"
    tip1 = kin.tcp(ctl.m)[:3, 3]
    assert abs(tip1[2] - tip0[2] - 0.010) < 0.002 and np.linalg.norm(tip1[:2] - tip0[:2]) < 0.002
    mover.start_nudge("tok", -0.010)
    assert wait(mover)["state"] == "done"
    assert abs(kin.tcp(ctl.m)[2, 3] - tip0[2]) < 0.002
    assert all(j in ARM for j, _ in ctl.targets)


def test_nudge_refuses_below_floor_and_bad_steps(kin, tmp_path):
    plane = make_plane(kin, tmp_path)
    low = kin.ik(np.array([0.25, 0.0, -0.001]), seed=dict(kin.spec.home))
    assert low.ok
    mover = Mover(FakeController(low.joints), kin, plane)
    with pytest.raises(MoveError, match="Too low"):
        mover.start_nudge("tok", -0.010)
    with pytest.raises(MoveError, match="between 1 and 30"):
        mover.start_nudge("tok", 0.2)
    with pytest.raises(MoveError, match="between 1 and 30"):
        mover.start_nudge("tok", float("nan"))


def test_loaded_joint_with_steady_sag_still_arrives(kin, tmp_path):
    plane = make_plane(kin, tmp_path)
    ctl = FakeController(kin.spec.home, sag={"shoulder_lift": 1.8})   # the real arm rests 1.8 deg off its command
    mover = Mover(ctl, kin, plane)
    mover.start_nudge("tok", 0.010)
    assert wait(mover)["state"] == "done"


def test_blocked_joint_is_reported(kin, tmp_path):
    plane = make_plane(kin, tmp_path)
    ctl = FakeController(kin.spec.home, sag={"elbow_flex": 9.0})      # something holds the elbow 9 deg from its command
    mover = Mover(ctl, kin, plane)
    mover.start_nudge("tok", 0.010)
    st = wait(mover)
    assert st["state"] == "error" and "elbow_flex is blocked" in st["message"]
