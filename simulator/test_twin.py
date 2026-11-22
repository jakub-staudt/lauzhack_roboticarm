import pytest

from simulator.twin import JOINTS, Twin

REAL = {"measured": {"shoulder_pan": 10.0, "shoulder_lift": -20.0, "elbow_flex": 40.0, "wrist_flex": 30.0, "wrist_roll": -90.0, "gripper": 12.0},
        "connected": True, "torque": True, "emergency_stop": {"latched": False}, "control": {"label": "Camera drive"}}


class Clock:
    t = 0.0

    def __call__(self):
        return self.t


def test_every_read_of_the_real_arm_drives_the_simulated_one():
    seen, clock = [], Clock()
    twin = Twin(lambda: REAL, seen.append, clock=clock)
    assert twin.poll_once()
    assert seen == [REAL["measured"]] and set(seen[0]) == set(JOINTS)
    st = twin.status()
    assert st["ok"] and st["real"]["owner"] == "Camera drive" and st["real"]["emergency_stop"] is False


def test_when_the_real_arm_stops_answering_the_twin_keeps_its_pose_and_reports_it_stale():
    seen, clock = [], Clock()
    answers = [REAL]

    def fetch():
        if not answers:
            raise OSError("link down")
        return answers.pop()

    twin = Twin(fetch, seen.append, clock=clock)
    assert twin.poll_once() and len(seen) == 1
    clock.t = 5.0
    assert not twin.poll_once()
    st = twin.status()
    assert len(seen) == 1                                    # nothing new was commanded: the last pose is held
    assert not st["ok"] and st["age_s"] == 5.0 and "OSError" in st["error"]


def test_a_malformed_reply_is_not_applied():
    seen = []
    twin = Twin(lambda: {"measured": {"shoulder_pan": 1.0}}, seen.append)
    assert not twin.poll_once() and seen == []


def test_the_simulated_arm_ends_up_where_the_real_one_is():
    """End to end with the real MuJoCo scene: after a few simulated seconds the sim joints match the real measured joints."""
    pytest.importorskip("mujoco")
    from simulator.web.server import Station

    station = Station("mujoco")
    station.start_twin("http://127.0.0.1:9")                 # never contacted: the fetch below is replaced
    station.twin._stopping.set()
    station.twin._fetch = lambda: REAL
    assert station.twin.poll_once()
    station.sim.advance(4.0)
    sim = station.planner.to_app(station.sim.joints())
    for joint, value in REAL["measured"].items():
        assert sim[joint] == pytest.approx(value, abs=1.5), joint
    from simulator.web.server import Busy
    with pytest.raises(Busy, match="digital twin"):
        station.go_home()                                    # no separate movement while it mirrors the real arm
    assert station.twin_status()["max_diff_deg"] < 1.5
