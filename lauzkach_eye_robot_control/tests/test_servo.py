"""The closed loop, against a simulated arm that does not land where it is told, and observers that lie or fail."""

import math
import random

import pytest

from lauzkach_eye_robot_control.servo import ServoConfig, run

TARGET = (0.30, -0.05)


class BiasedArm:
    """Lands `bias` metres away from every commanded point (a calibration error), plus a little noise."""

    def __init__(self, bias=(0.012, -0.008), noise_mm=0.4, seed=1):
        self.bias, self.noise, self.rng = bias, noise_mm / 1000, random.Random(seed)
        self.tip = (0.0, 0.0)
        self.moves: list[tuple[float, float, float]] = []
        self.finishes = True

    def move_xy(self, x, y, height_cm):
        self.moves.append((x, y, height_cm))
        self.tip = (x + self.bias[0], y + self.bias[1])

    def wait_idle(self, timeout_s):
        return self.finishes


class Camera:
    """Reports the arm's true tip with noise; can be told to lie, to lose the tip, or to report low confidence."""

    def __init__(self, arm, noise_mm=0.5, seed=2):
        self.arm, self.noise, self.rng = arm, noise_mm / 1000, random.Random(seed)
        self.mode, self.confidence, self.calls = "true", 0.9, 0

    def observe_xy(self):
        self.calls += 1
        if self.mode == "blind":
            return None
        if self.mode == "lie":                                                # claims the tip is 10 cm from where it is, and is sure of it
            return (self.arm.tip[0] + 0.10, self.arm.tip[1], 0.95)
        return (self.arm.tip[0] + self.rng.gauss(0, self.noise), self.arm.tip[1] + self.rng.gauss(0, self.noise), self.confidence)


FAST = ServoConfig(settle_s=0.0)
NO_SLEEP = lambda s: None  # noqa: E731


def test_a_one_centimetre_calibration_error_is_corrected_to_a_few_millimetres():
    arm = BiasedArm()
    result = run(TARGET, 3.0, arm, Camera(arm), FAST, sleep=NO_SLEEP)
    final_error = math.hypot(arm.tip[0] - TARGET[0], arm.tip[1] - TARGET[1]) * 1000
    assert result.ok and final_error < 3.0
    assert len(arm.moves) <= 4                                              # the plain move plus at most three corrections
    assert result.steps[0].error_mm > 10                                    # the first look found the ~14 mm miss
    assert all(m[2] == 3.0 for m in arm.moves)                              # the height never changes


def test_an_arm_that_is_already_accurate_is_not_touched():
    arm = BiasedArm(bias=(0.0005, 0.0))
    result = run(TARGET, 3.0, arm, Camera(arm), FAST, sleep=NO_SLEEP)
    assert result.ok and len(arm.moves) == 1                                # one move, one look, done


def test_when_the_tip_cannot_be_seen_the_open_loop_move_is_kept():
    arm = BiasedArm()
    camera = Camera(arm)
    camera.mode = "blind"
    result = run(TARGET, 3.0, arm, camera, FAST, sleep=NO_SLEEP)
    assert not result.ok and "could not be seen" in result.reason and len(arm.moves) == 1 and camera.calls == FAST.observe_tries


def test_low_confidence_observations_are_ignored_not_trusted():
    arm = BiasedArm()
    camera = Camera(arm)
    camera.confidence = 0.2
    result = run(TARGET, 3.0, arm, camera, FAST, sleep=NO_SLEEP)
    assert not result.ok and len(arm.moves) == 1


def test_an_observer_that_lies_cannot_make_the_arm_run_away():
    arm = BiasedArm()
    camera = Camera(arm)
    camera.mode = "lie"
    result = run(TARGET, 3.0, arm, camera, FAST, sleep=NO_SLEEP)
    drift = max(math.hypot(x - TARGET[0], y - TARGET[1]) for x, y, _ in arm.moves) * 1000
    assert not result.ok and drift <= FAST.max_total_mm + 1e-6              # never farther than 30 mm from what was tapped
    steps = [math.hypot(b[0] - a[0], b[1] - a[1]) * 1000 for a, b in zip(arm.moves, arm.moves[1:], strict=False)]
    assert all(s <= FAST.max_step_mm + 1e-6 for s in steps)                  # and no single correction above 15 mm


def test_stop_ends_the_loop_before_the_next_move():
    arm = BiasedArm()
    stop = {"now": False}
    original = arm.move_xy

    def move_then_stop(x, y, h):
        original(x, y, h)
        stop["now"] = True                                                   # the user presses Stop during the first move

    arm.move_xy = move_then_stop
    result = run(TARGET, 3.0, arm, Camera(arm), FAST, should_stop=lambda: stop["now"], sleep=NO_SLEEP)
    assert result.reason == "stopped" and len(arm.moves) == 1


def test_a_move_that_never_finishes_ends_the_loop():
    arm = BiasedArm()
    arm.finishes = False
    result = run(TARGET, 3.0, arm, Camera(arm), FAST, sleep=NO_SLEEP)
    assert not result.ok and "in time" in result.reason and len(arm.moves) == 1


def test_noise_alone_does_not_keep_the_arm_moving():
    arm = BiasedArm(bias=(0.0, 0.0), noise_mm=0.8)
    result = run(TARGET, 3.0, arm, Camera(arm, noise_mm=1.0), FAST, sleep=NO_SLEEP)
    assert result.ok and len(arm.moves) <= 2


@pytest.mark.parametrize("bias", [(0.02, 0.0), (-0.015, 0.015), (0.0, -0.025)])
def test_converges_for_larger_calibration_errors_too(bias):
    arm = BiasedArm(bias=bias)
    result = run(TARGET, 3.0, arm, Camera(arm), ServoConfig(settle_s=0.0, max_iters=6), sleep=NO_SLEEP)
    assert result.ok and result.final_error_mm < 3.0
