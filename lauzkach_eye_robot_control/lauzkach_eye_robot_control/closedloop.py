"""The parts that plug the closed loop (servo.py) into real things: a mover, and observers of the tip.

* `SimMover`          moves the simulated arm through its web server and waits until it has stopped;
* `SimTruthObserver`  a stand-in "camera" for the simulator: it reports the simulated tip, shifted by a made-up calibration
                      error, so the loop can be shown correcting a miss (a demo and a test: it is NOT the real camera);
* `CameraTipObserver` the real thing: a picture + the arm's pose -> where the tip is on the board.
"""

from __future__ import annotations

import time
from collections.abc import Callable

import numpy as np

from lauzkach_eye_robot_control.drive import SimClient, SimError
from lauzkach_eye_robot_control.feature_model import FeatureModel
from lauzkach_eye_robot_control.tip import PixelObserver, TipObservation, observe_first

DONE_STATES = ("done", "idle")


class SimMover:
    def __init__(self, sim: SimClient, poll_s: float = 0.25) -> None:
        self.sim, self.poll_s = sim, poll_s
        self.last_state = ""

    def move_xy(self, x: float, y: float, height_cm: float) -> None:
        self.sim.go_to_base(x, y, height_cm)

    def wait_idle(self, timeout_s: float) -> bool:
        end = time.monotonic() + timeout_s
        time.sleep(self.poll_s)                                  # let the task start before looking for its end
        while time.monotonic() < end:
            try:
                state = self.sim.state()["task"]["state"]
            except (SimError, KeyError):
                return False
            self.last_state = state
            if state in DONE_STATES:
                return True
            if state in ("error", "stopped"):
                return False
            time.sleep(self.poll_s)
        return False


class SimTruthObserver:
    """Reports the simulated tip as the board would see it: the arm's coordinates minus a calibration error (metres)."""

    def __init__(self, sim: SimClient, calibration_error: tuple[float, float] = (0.0, 0.0), noise_mm: float = 0.0) -> None:
        self.sim, self.error, self.noise = sim, calibration_error, noise_mm / 1000.0
        self._rng = np.random.default_rng(0)

    def observe_xy(self) -> tuple[float, float, float] | None:
        try:
            tip = self.sim.state()["tip_base"]
        except (SimError, KeyError):
            return None
        jitter = self._rng.normal(0, self.noise, 2) if self.noise else (0.0, 0.0)
        return tip["x"] - self.error[0] + jitter[0], tip["y"] - self.error[1] + jitter[1], 0.95


class CameraTipObserver:
    """A picture and the arm's own pose -> the tip's position on the board [m, arm base frame].

    `pose_source()` returns {"tip": (x, y, z), "rotation": 3x3} from the arm's joint angles, `frame_source()` the latest picture.
    The arm's own model says where the feature should appear (the hint); the observer looks near it, and a result that is
    implausibly far from where the arm believes its tip is (`max_disagreement_m`) is refused instead of chased."""

    def __init__(self, frame_source: Callable[[], np.ndarray | None], pose_source: Callable[[], dict | None],
                 observers: list[PixelObserver], model: FeatureModel, max_disagreement_m: float = 0.04) -> None:
        self.frame_source, self.pose_source, self.observers = frame_source, pose_source, observers
        self.model, self.max_disagreement = model, max_disagreement_m
        self.last: TipObservation | None = None

    def observe_xy(self) -> tuple[float, float, float] | None:
        frame, pose = self.frame_source(), self.pose_source()
        if frame is None or pose is None:
            return None
        tip, rotation = pose["tip"], pose["rotation"]
        seen = observe_first(self.observers, frame, self.model.predict_pixel(tip, rotation))
        self.last = seen
        if seen is None:
            return None
        x, y = self.model.tip_xy_from_pixel(seen.px, tip[2], rotation)
        if np.hypot(x - tip[0], y - tip[1]) > self.max_disagreement:
            return None
        return x, y, seen.confidence
