import time
"""The drive service with a fake simulator: clicks and corners become moves, and lag cannot make a move run late, twice or after Stop."""

import numpy as np
import pytest

from lauzkach_eye_robot_control.drive import CommandGuard, SimError
from lauzkach_eye_robot_control.robot_map import RobotMap
from lauzkach_eye_robot_control.service import ApiError, DriveService
from tests.test_robot_map import RECORDED

SQUARE = [[0.1, 0.0, 0.0], [0.0, 0.1, 0.0], [0.0, 0.0, 1.0]]          # the rectangle is a 200 x 200 px square at the origin


class FakeSim:
    def __init__(self):
        self.moves, self.stops, self.busy, self.down = [], 0, False, False

    def _check(self):
        if self.down:
            raise SimError("the simulator is not reachable", 503)

    def state(self):
        self._check()
        return {"task": {"state": "running" if self.busy else "idle", "message": ""}, "tip_base": {"x": 0.2, "y": 0.0, "z": 0.03}}

    def go_to_base(self, x, y, height_cm):
        self._check()
        self.moves.append((x, y, height_cm))
        self.busy = True
        return {"state": "planning"}

    def stop(self):
        self._check()
        self.stops += 1
        self.busy = False
        return {"state": "stopped"}


class Clock:
    def __init__(self): self.t = 5000.0
    def __call__(self): return self.t


@pytest.fixture()
def rig():
    clock, sim = Clock(), FakeSim()
    service = DriveService(lambda: SQUARE, RobotMap(RECORDED), sim, guard=CommandGuard(clock=clock))
    return service, sim, clock


def env(service, clock, cid, **kw):
    return {"cid": cid, "sent_ms": clock() * 1000, "gen": service.guard.generation, **kw}


def test_a_click_becomes_a_move_to_the_robot_calibrated_position(rig):
    service, sim, clock = rig
    out = service.go_click(env(service, clock, "a"), px=0, py=200, height_cm=3.0)      # the picture's bottom-left = near-left corner
    near_left = RobotMap(RECORDED).uv_to_xy(0, 0)
    assert sim.moves == [pytest.approx((*near_left, 3.0))] and out["target"]["u"] == pytest.approx(0, abs=0.01)


def test_corners_are_the_four_recorded_rectangle_corners_in_order(rig):
    service, sim, clock = rig
    for i in range(4):
        service.go_corner(env(service, clock, f"c{i}"), i, 1.0)
    expected = [(c["x"], c["y"]) for c in RobotMap(RECORDED).corners()]
    assert [(m[0], m[1]) for m in sim.moves] == [pytest.approx(e) for e in expected] and {m[2] for m in sim.moves} == {1.0}


def test_clicks_outside_the_rectangle_or_before_it_is_locked_move_nothing(rig):
    service, sim, clock = rig
    with pytest.raises(ApiError) as outside:
        service.go_click(env(service, clock, "o"), px=400, py=100, height_cm=3.0)
    assert outside.value.code == "outside" and sim.moves == []
    service._matrix_source = lambda: None
    with pytest.raises(ApiError) as unlocked:
        service.go_click(env(service, clock, "u"), px=100, py=100, height_cm=3.0)
    assert unlocked.value.code == "not_locked" and sim.moves == []


def test_a_lagging_command_is_refused_and_the_arm_does_not_move(rig):
    service, sim, clock = rig
    request = env(service, clock, "late")
    clock.t += 4                                                                       # the request sat in a lagging link for 4 s
    with pytest.raises(ApiError) as err:
        service.go_corner(request, 0, 3.0)
    assert err.value.code == "stale" and sim.moves == []


def test_a_retried_copy_does_not_move_the_arm_twice(rig):
    service, sim, clock = rig
    request = env(service, clock, "once")
    first = service.go_corner(request, 1, 3.0)
    second = service.go_corner(request, 1, 3.0)                                        # the page retried after a lost reply
    assert len(sim.moves) == 1 and second["duplicate"] and second["target"] == first["target"]


def test_stop_is_always_accepted_and_voids_moves_still_on_their_way(rig):
    service, sim, clock = rig
    queued = env(service, clock, "queued")                                             # sent, then Stop pressed, then it arrives
    clock.t += 3                                                                       # (a late stop is still honoured: no age check)
    result = service.stop()
    assert result == {"generation": 1, "held": True} and sim.stops == 1
    with pytest.raises(ApiError) as err:
        service.go_corner(queued, 0, 3.0)
    assert err.value.code == "old_generation" and sim.moves == []
    service.go_corner(env(service, clock, "after"), 0, 3.0)                            # nothing latched: the next move is fine
    assert len(sim.moves) == 1


def test_silent_page_during_a_move_holds_the_arm(rig):
    service, sim, clock = rig
    service.go_corner(env(service, clock, "m"), 0, 3.0)                                # a move is running
    service.deadman._clock = lambda: service.deadman._last + 10                        # ...and the page has been silent for 10 s
    assert service.deadman.check() and sim.stops == 1 and not sim.busy
    assert service.guard.generation == 1 and any("silent" in line for line in service.log)


def test_a_stopped_simulator_is_reported_not_crashed(rig):
    service, sim, clock = rig
    sim.down = True
    assert service.overview()["sim"]["connected"] is False
    assert service.stop()["held"] is False


# ---------------------------------------------------------------------------------------------- closed loop through the service
import threading  # noqa: E402
import time  # noqa: E402

from lauzkach_eye_robot_control.servo import ServoConfig  # noqa: E402
from tests.test_servo import BiasedArm, Camera  # noqa: E402


def closed_rig(arm=None):
    clock, sim = Clock(), FakeSim()
    arm = arm or BiasedArm()
    service = DriveService(lambda: SQUARE, RobotMap(RECORDED), sim, guard=CommandGuard(clock=clock),
                           closed_loop=(arm, Camera(arm)), servo_config=ServoConfig(settle_s=0.0))
    return service, arm, clock


def wait_servo(service, timeout=5.0):
    end = time.time() + timeout
    while time.time() < end:
        if not service.overview()["servo"]["running"]:
            return service.overview()["servo"]["last"]
        time.sleep(0.02)
    raise AssertionError("the closed loop never finished")


def test_a_closed_loop_click_corrects_the_arms_miss_in_the_background():
    service, arm, clock = closed_rig()
    answer = service.go_click(env(service, clock, "cl"), px=100, py=100, height_cm=3.0, closed_loop=True)
    assert answer["closed_loop"] is True                                    # returns at once; the loop runs behind it
    last = wait_servo(service)
    target = RobotMap(RECORDED).uv_to_xy(answer["target"]["u"], answer["target"]["v"])
    assert last["ok"] and np.hypot(arm.tip[0] - target[0], arm.tip[1] - target[1]) < 0.003
    assert last["steps"][0]["error_mm"] > 10                                # the first look found the miss


def test_closed_loop_without_an_observer_says_so_and_moves_nothing(rig):
    service, sim, clock = rig
    with pytest.raises(ApiError) as err:
        service.go_corner(env(service, clock, "x"), 0, 3.0, closed_loop=True)
    assert err.value.code == "no_observer" and sim.moves == []
    assert service.overview()["servo"]["available"] is False


def test_a_second_closed_loop_move_is_refused_while_one_runs_and_stop_ends_it():
    release = threading.Event()

    class SlowArm(BiasedArm):
        def wait_idle(self, timeout_s):
            release.wait(5)
            return True

    service, arm, clock = closed_rig(SlowArm())
    service.go_corner(env(service, clock, "one"), 0, 3.0, closed_loop=True)
    time.sleep(0.1)
    with pytest.raises(ApiError) as err:
        service.go_corner(env(service, clock, "two"), 1, 3.0, closed_loop=True)
    assert err.value.code == "busy"
    service.stop()                                                          # the user presses Stop; the arm finishes its step
    release.set()
    last = wait_servo(service)
    assert last["reason"] == "stopped" and len(arm.moves) == 1              # no further move after the Stop
    service.go_corner(env(service, clock, "three"), 1, 3.0, closed_loop=True)   # and the next one is accepted: nothing latched
    release.set()
    wait_servo(service)


# ---------------------------------------------------------------------------------------------- the REAL arm gating
class FakeReal:
    """Stands in for RealArmClient: records what it is asked and can refuse like the real one."""

    def __init__(self):
        self.moves, self.stops, self.refuse, self.guard, self.grasps = [], 0, None, True, []

    def state(self):
        return {"task": {"state": "idle", "message": ""}, "tip_base": {"x": 0.2, "y": 0.0, "z": 0.03}, "connected": True, "owner": None,
                "emergency_stop": False, "gripper": 10.0, "height_guard": self.guard}

    def go_to_base(self, x, y, height_cm):
        if self.refuse:
            raise RuntimeError(self.refuse)
        self.moves.append((x, y, height_cm))
        return {"state": "planning"}

    def grasp_action(self, action, x=None, y=None):
        if self.refuse:
            raise RuntimeError(self.refuse)
        self.grasps.append((action, x, y))
        return {"state": "planning"}

    def stop(self):
        self.stops += 1
        return {}


def real_rig(real=None):
    clock, sim = Clock(), FakeSim()
    return DriveService(lambda: SQUARE, RobotMap(RECORDED), sim, guard=CommandGuard(clock=clock), real=real), sim, clock


def test_the_real_arm_is_off_unless_the_server_enabled_it():
    service, sim, clock = real_rig(real=None)
    with pytest.raises(ApiError) as err:
        service.go_corner(env(service, clock, "r"), 0, 3.0, target="real", confirm_real=True)
    assert err.value.code == "no_real" and sim.moves == []
    assert service.overview()["real"] == {"available": False}


def test_every_real_move_needs_an_explicit_confirmation_and_never_runs_closed_loop_or_free_clicks():
    real = FakeReal()
    service, sim, clock = real_rig(real)
    with pytest.raises(ApiError) as err:
        service.go_corner(env(service, clock, "a"), 1, 3.0, target="real", confirm_real=False)
    assert err.value.code == "confirm" and real.moves == []
    with pytest.raises(ApiError) as err:
        service.go_corner(env(service, clock, "b"), 1, 3.0, closed_loop=True, target="real", confirm_real=True)
    assert err.value.code == "bad_request" and real.moves == []
    with pytest.raises(ApiError) as err:
        service.go_click(env(service, clock, "c"), 100, 100, 3.0, target="real", confirm_real=False)      # a free click needs it too
    assert err.value.code == "confirm" and real.moves == []


def test_a_confirmed_click_on_the_picture_moves_the_real_arm_to_that_point_at_3_cm_and_clicks_outside_are_refused():
    real = FakeReal()
    service, sim, clock = real_rig(real)
    px, py = 100, 100                                                          # the middle of the 200 x 200 px square
    answer = service.go_click(env(service, clock, "k1"), px, py, 1.0, target="real", confirm_real=True)       # asks for 1 cm: the real arm uses 3
    assert real.moves and real.moves[0][2] == 3.0 and sim.moves == [] and answer["real"] is True
    with pytest.raises(ApiError) as err:
        service.go_click(env(service, clock, "k2"), -500, -500, 3.0, target="real", confirm_real=True)
    assert err.value.code == "outside" and len(real.moves) == 1


def test_a_confirmed_real_corner_move_goes_to_the_real_arm_only():
    real = FakeReal()
    service, sim, clock = real_rig(real)
    answer = service.go_corner(env(service, clock, "ok"), 2, 3.0, target="real", confirm_real=True)
    expected = RobotMap(RECORDED).corners()[2]
    assert real.moves == [pytest.approx((expected["x"], expected["y"], 3.0))] and sim.moves == [] and answer["real"] is True


def test_the_real_arms_refusal_reaches_the_page_as_a_clear_reason():
    real = FakeReal()
    real.refuse = "Close the gripper first."
    service, sim, clock = real_rig(real)
    with pytest.raises(ApiError) as err:
        service.go_corner(env(service, clock, "no"), 0, 3.0, target="real", confirm_real=True)
    assert err.value.code == "real_refused" and "gripper" in str(err.value)
    assert any("real arm refused" in line for line in service.log)


def test_stop_halts_both_arms_and_a_silent_page_holds_the_real_one_too():
    real = FakeReal()
    service, sim, clock = real_rig(real)
    service.stop()
    assert sim.stops == 1 and real.stops == 1
    service._hold_after_silence()
    assert sim.stops == 2 and real.stops == 2
    assert service.overview()["real"]["available"] and service.overview()["real"]["connected"]


def test_free_clicks_on_the_real_arm_are_refused_while_the_agent_lacks_the_height_guard_but_corners_still_work():
    real = FakeReal()
    real.guard = False
    service, sim, clock = real_rig(real)
    with pytest.raises(ApiError) as err:
        service.go_click(env(service, clock, "g1"), 100, 100, 3.0, target="real", confirm_real=True)
    assert err.value.code == "agent_old" and real.moves == []
    service.go_corner(env(service, clock, "g2"), 0, 3.0, target="real", confirm_real=True)
    assert len(real.moves) == 1


def test_the_simulators_digital_twin_status_reaches_the_page():
    service, sim, clock = real_rig(FakeReal())
    original = sim.state
    sim.state = lambda: {**original(), "twin": {"available": True, "enabled": True, "ok": True}}
    assert service.overview()["sim"]["twin"]["enabled"] is True


def test_a_grasp_step_needs_the_real_arm_a_confirmation_and_for_hover_a_click_inside_the_rectangle():
    real = FakeReal()
    service, sim, clock = real_rig(real)
    with pytest.raises(ApiError) as err:
        service.grasp(env(service, clock, "s1"), "open", confirm_real=False)
    assert err.value.code == "confirm" and real.grasps == []
    service.grasp(env(service, clock, "s2"), "open", confirm_real=True)
    assert real.grasps == [("open", None, None)]
    with pytest.raises(ApiError, match="Click the block"):
        service.grasp(env(service, clock, "s3"), "hover", confirm_real=True)
    with pytest.raises(ApiError) as err:
        service.grasp(env(service, clock, "s4"), "hover", -500, -500, confirm_real=True)
    assert err.value.code == "outside" and len(real.grasps) == 1
    service.grasp(env(service, clock, "s5"), "hover", 100, 100, confirm_real=True)
    action, x, y = real.grasps[-1]
    assert action == "hover" and x is not None and y is not None


def test_the_agents_refusal_of_a_grasp_step_reaches_the_page_as_a_reason():
    real = FakeReal()
    real.refuse = "Not now: after 'idle' the next steps are open."
    service, sim, clock = real_rig(real)
    with pytest.raises(ApiError) as err:
        service.grasp(env(service, clock, "s6"), "lower", confirm_real=True)
    assert err.value.code == "real_refused" and "Not now" in str(err.value)


# ---------------------------------------------------------------------------------------------- pick and place (the whole sequence)
class ScriptedReal(FakeReal):
    """Behaves like the agent: every grasp action runs for a moment and then finishes; the stage advances."""

    NEXT = {"open": "open", "hover": "hover", "lower": "lower", "close": "closed", "lift": "lifted", "carry": "carried", "place": "hover"}

    def __init__(self, fail_at=None, miss=False):
        super().__init__()
        self.calls, self.fail_at, self.miss, self.stage, self.task, self.busy_until = [], fail_at, miss, "idle", "idle", 0.0

    def grasp_action(self, action, x=None, y=None, yaw=None, park_after=False, obstacles=None):
        if action == "lower_stop":
            return {"state": "stopping"}
        self.calls.append((action, x, y, yaw, park_after, obstacles))
        self.busy_until = time.time() + 0.05
        self.pending = action
        return {"state": "planning"}

    def state(self):
        st = super().state()
        if self.calls and time.time() >= self.busy_until and getattr(self, "pending", None):
            action, self.pending = self.pending, None
            if action == self.fail_at:
                self.task = "error"
            else:
                self.task = "done"
                self.stage = "miss" if (action == "close" and self.miss) else self.NEXT[action]
        elif getattr(self, "pending", None):
            self.task = "moving"
        st["task"] = {"state": self.task, "message": f"{self.calls[-1][0]} done" if self.calls else ""}
        st["grasp"] = {"stage": self.stage}
        return st


def wait_for(runner, secs=10.0):
    end = time.time() + secs
    while runner.busy() and time.time() < end:
        time.sleep(0.02)
    assert not runner.busy()


def pp_rig(real):
    service, sim, clock = real_rig(real)
    service.wrist_fetch = None
    return service, clock


PICK = {"px": 60, "py": 150, "colour": "blue", "box": [[50, 140], [70, 140], [70, 160], [50, 160]]}
DEST = {"px": 150, "py": 60}


def test_pick_and_place_runs_all_the_steps_in_order_with_the_right_places_and_finishes():
    real = ScriptedReal()
    service, clock = pp_rig(real)
    service.pick_place(env(service, clock, "p1"), PICK, DEST, others=[[100, 190]], confirm_real=True, park_after=True)
    from lauzkach_eye_robot_control.pickplace import STEPS
    service.runner.timeout_s, service.runner.poll_s = 5.0, 0.02
    wait_for(service.runner)
    assert [c[0] for c in real.calls] == [s[0] for s in STEPS]
    assert service.runner.status["state"] == "done"
    hover, carry, place = real.calls[1], real.calls[5], real.calls[6]
    assert hover[1] is not None and hover[3] is not None                              # the block's place and rotation went to the hover
    assert carry[1] is not None and not carry[5]                                       # the destination only: no block-distance checks once the block is held
    assert place[4] is True


def test_pick_and_place_needs_confirmation_and_a_free_destination_and_the_real_arm():
    real = ScriptedReal()
    service, clock = pp_rig(real)
    with pytest.raises(ApiError) as err:
        service.pick_place(env(service, clock, "p2"), PICK, DEST, confirm_real=False)
    assert err.value.code == "confirm" and real.calls == []
    with pytest.raises(ApiError, match="on top of another block"):
        service.pick_place(env(service, clock, "p3"), PICK, DEST, others=[[152, 62]], confirm_real=True)
    with pytest.raises(ApiError, match="too close to the starting block"):
        service.pick_place(env(service, clock, "p4"), PICK, {"px": 62, "py": 150}, confirm_real=True)
    with pytest.raises(ApiError) as err:
        service.pick_place(env(service, clock, "p5"), PICK, {"px": -900, "py": 0}, confirm_real=True)
    assert err.value.code == "outside" and real.calls == []
    service2, _sim2, clock2 = real_rig(None)
    with pytest.raises(ApiError) as err:
        service2.pick_place(env(service2, clock2, "p6"), PICK, DEST, confirm_real=True)
    assert err.value.code == "no_real"


def test_a_failed_or_missed_step_stops_the_sequence_and_says_which_one_and_why():
    for kw, step_name in (({"fail_at": "lower"}, "lower"), ({"miss": True}, "close")):
        real = ScriptedReal(**kw)
        service, clock = pp_rig(real)
        service.pick_place(env(service, clock, "f" + step_name), PICK, DEST, confirm_real=True)
        service.runner.timeout_s, service.runner.poll_s = 5.0, 0.02
        wait_for(service.runner)
        st = service.runner.status
        assert st["state"] == "error" and "arm holds" in st["message"], st
        assert real.calls[-1][0] == step_name                                            # nothing after the failed step was started


def test_stop_cancels_the_sequence_before_the_next_step():
    real = ScriptedReal()
    service, clock = pp_rig(real)
    service.pick_place(env(service, clock, "s1"), PICK, DEST, confirm_real=True)
    service.runner.timeout_s, service.runner.poll_s = 5.0, 0.02
    time.sleep(0.1)
    service.stop()
    wait_for(service.runner)
    assert service.runner.status["state"] == "stopped" and len(real.calls) < 7


def test_a_second_try_while_the_arm_holds_a_block_carries_on_from_the_carry_step_without_a_starting_block():
    real = ScriptedReal()
    real.stage = "lifted"
    service, clock = pp_rig(real)
    service.pick_place(env(service, clock, "r1"), None, DEST, others=[[100, 190]], confirm_real=True)
    service.runner.timeout_s, service.runner.poll_s = 5.0, 0.02
    wait_for(service.runner)
    assert [c[0] for c in real.calls] == ["carry", "place"]
    assert service.runner.status["state"] == "done"


def test_without_a_held_block_a_starting_block_is_required():
    real = ScriptedReal()
    service, clock = pp_rig(real)
    with pytest.raises(ApiError, match="Choose the starting block"):
        service.pick_place(env(service, clock, "r2"), None, DEST, confirm_real=True)


def test_when_the_arm_holds_a_block_no_other_block_can_stop_the_carry():
    real = ScriptedReal()
    real.stage = "lifted"
    service, clock = pp_rig(real)
    service.pick_place(env(service, clock, "r9"), None, DEST, others=[[150, 62], [100, 100]], confirm_real=True)       # even a block at the destination
    service.runner.timeout_s, service.runner.poll_s = 5.0, 0.02
    wait_for(service.runner)
    assert [c[0] for c in real.calls] == ["carry", "place"] and not real.calls[0][5]


def test_the_picked_block_itself_is_never_taken_for_another_block():
    real = ScriptedReal()
    service, clock = pp_rig(real)
    service.pick_place(env(service, clock, "q1"), PICK, DEST, others=[[61, 151]], confirm_real=True)                  # its own centre, a pixel off
    service.runner.timeout_s, service.runner.poll_s = 5.0, 0.02
    wait_for(service.runner)
    assert service.runner.status["state"] == "done"
