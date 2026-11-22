"""Picture <-> (u, v) conversions and the protections against lag."""

import pytest

from lauzkach_eye_robot_control.drive import FACES, CommandGuard, DeadMan, image_unit_to_uv, px_to_uv, uv_to_image_unit, uv_to_px

SQUARE = [[0.1, 0.0, 0.0], [0.0, 0.1, 0.0], [0.0, 0.0, 1.0]]       # a picture where the rectangle is a 200 x 200 px square at the origin


@pytest.mark.parametrize("faces", FACES)
def test_orientation_mapping_round_trips(faces):
    for u, v in ((0.0, 0.0), (1.0, 0.0), (1.0, 1.0), (0.25, 0.7)):
        assert image_unit_to_uv(*uv_to_image_unit(u, v, faces), faces) == pytest.approx((u, v))


def test_robot_facing_up_the_near_left_corner_is_the_bottom_left_of_the_picture():
    assert px_to_uv(SQUARE, 0, 200, "up") == pytest.approx((0, 0))     # bottom-left  = near-left
    assert px_to_uv(SQUARE, 200, 200, "up") == pytest.approx((1, 0))   # bottom-right = near-right
    assert px_to_uv(SQUARE, 200, 0, "up") == pytest.approx((1, 1))     # top-right    = far-right
    assert uv_to_px(SQUARE, 0, 1, "up") == pytest.approx((0, 0))       # far-left     = top-left


def test_robot_facing_right_the_far_edge_is_the_right_side():
    assert px_to_uv(SQUARE, 200, 100, "right") == pytest.approx((0.5, 1.0))   # right edge = far; down = the robot's right
    assert px_to_uv(SQUARE, 0, 200, "right") == pytest.approx((1.0, 0.0))


class Clock:
    def __init__(self): self.t = 1000.0
    def __call__(self): return self.t


def make(**kw):
    clock = Clock()
    return CommandGuard(max_age_s=2.0, clock=clock, **kw), clock


def test_fresh_command_runs_and_a_retried_copy_does_not_move_the_arm_twice():
    guard, clock = make()
    first = guard.admit("c1", clock() * 1000, guard.generation)
    assert first.ok
    guard.record("c1", {"state": "planning"})
    again = guard.admit("c1", clock() * 1000, guard.generation)
    assert again.status == "duplicate" and again.cached == {"state": "planning"}      # same answer, no second move


def test_a_command_delayed_by_lag_is_refused():
    guard, clock = make()
    sent = clock() * 1000
    clock.t += 3.5                                                    # the link stalled for 3.5 s
    late = guard.admit("c2", sent, guard.generation)
    assert late.status == "stale" and "lagging" in late.detail
    assert guard.admit("c3", sent, guard.generation, max_age_ms=5000).ok   # a command can allow a longer delay explicitly


def test_a_move_sent_before_stop_but_arriving_after_it_is_void():
    guard, clock = make()
    before = guard.generation
    queued = ("c4", clock() * 1000, before)                           # the page sent it, then the user pressed Stop
    assert guard.stop() == before + 1
    assert guard.admit(*queued).status == "old_generation"
    assert guard.admit("c5", clock() * 1000, guard.generation).ok     # a command made after the stop is fine


def test_incomplete_and_future_commands_are_refused():
    guard, clock = make()
    assert guard.admit(None, None, None).status == "invalid"
    assert guard.admit("c6", clock() * 1000 + 60_000, guard.generation).status == "invalid"     # the page's clock is a minute ahead


def test_dead_man_holds_the_arm_only_when_a_move_is_running_and_the_page_is_silent():
    state = {"active": False, "fired": 0}
    t = Clock()
    man = DeadMan(3.0, lambda: state["active"], lambda: state.__setitem__("fired", state["fired"] + 1), clock=t)
    t.t += 10
    assert not man.check() and state["fired"] == 0                    # silent page but nothing moving: nothing to do
    state["active"] = True
    man.touch()
    t.t += 2.9
    assert not man.check()                                            # still within the limit
    t.t += 0.3
    assert man.check() and state["fired"] == 1                        # silent for more than 3 s during a move: hold
    t.t += 5
    assert not man.check() and state["fired"] == 1                    # fires once, not repeatedly
    man.touch()
    t.t += 4
    assert man.check() and state["fired"] == 2                        # a new silence is a new event
