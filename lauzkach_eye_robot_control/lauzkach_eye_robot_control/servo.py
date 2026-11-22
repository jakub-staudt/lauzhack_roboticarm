"""Closed-loop correction of a move: command the arm, look at where the tip really is, correct, repeat.

The open-loop move uses the arm's own calibration (accurate to about a centimetre). The camera sees the board to about a
millimetre, so once something on the gripper can be tracked, the picture is the better judge of where the tip is. This module is
the loop and its safety limits; it knows nothing about cameras or arms:

* a `Mover` commands the tip over a point (arm base frame, metres) and says when it has stopped;
* an `Observer` reports where the tip actually is (same frame), or None if it cannot tell.

Safety limits, because a wrong observation must never make the arm run away: each correction is capped, the commanded point
never strays more than `max_total_mm` from the target the user tapped, low-confidence observations are not used, a Stop
ends the loop before the next move, and the loop gives up after a few iterations instead of chasing noise.
"""

from __future__ import annotations

import math
import time
from collections.abc import Callable
from dataclasses import dataclass, field
from typing import Protocol


class Mover(Protocol):
    def move_xy(self, x: float, y: float, height_cm: float) -> None: ...      # start a move over (x, y) [m]
    def wait_idle(self, timeout_s: float) -> bool: ...                         # True when the arm has finished and stopped


class Observer(Protocol):
    def observe_xy(self) -> tuple[float, float, float] | None: ...             # (x, y [m], confidence 0..1) of the tip, or None


@dataclass
class ServoConfig:
    tol_mm: float = 3.0              # done when the tip is this close to the target
    max_iters: int = 4               # corrections after the first (open-loop) move
    gain: float = 0.8                # fraction of the observed error corrected each time (below 1: noise does not overshoot)
    max_step_mm: float = 15.0        # largest single correction
    max_total_mm: float = 30.0       # the commanded point never strays farther than this from the tapped target
    min_confidence: float = 0.5      # observations below this are ignored
    settle_s: float = 0.8            # wait after the arm stops, so it is still when the picture is taken
    observe_tries: int = 3           # tries per iteration before giving up on seeing the tip
    move_timeout_s: float = 30.0


@dataclass
class ServoStep:
    iteration: int
    commanded: tuple[float, float]
    observed: tuple[float, float] | None
    error_mm: float | None
    note: str = ""


@dataclass
class ServoResult:
    ok: bool
    reason: str
    steps: list[ServoStep] = field(default_factory=list)
    commanded: tuple[float, float] = (0.0, 0.0)
    final_error_mm: float | None = None

    def summary(self) -> dict:
        return {"ok": self.ok, "reason": self.reason, "final_error_mm": None if self.final_error_mm is None else round(self.final_error_mm, 1),
                "steps": [{"iteration": s.iteration, "commanded_mm": [round(s.commanded[0] * 1000, 1), round(s.commanded[1] * 1000, 1)],
                           "error_mm": None if s.error_mm is None else round(s.error_mm, 1), "note": s.note} for s in self.steps]}


def run(target: tuple[float, float], height_cm: float, mover: Mover, observer: Observer, config: ServoConfig | None = None,
        should_stop: Callable[[], bool] = lambda: False, sleep: Callable[[float], None] = time.sleep) -> ServoResult:
    """Move over `target` [m, arm base frame] and refine the position from observations of the tip."""
    cfg = config or ServoConfig()
    tx, ty = target
    cx, cy = tx, ty
    result = ServoResult(False, "", commanded=(cx, cy))
    for iteration in range(cfg.max_iters + 1):                 # iteration 0 is the plain move; the rest are corrections
        if should_stop():
            result.reason = "stopped"
            return result
        mover.move_xy(cx, cy, height_cm)
        if not mover.wait_idle(cfg.move_timeout_s):
            result.reason = "the arm did not finish the move in time"
            return result
        if should_stop():
            result.reason = "stopped"
            return result
        sleep(cfg.settle_s)
        seen = None
        for _ in range(cfg.observe_tries):
            seen = observer.observe_xy()
            if seen is not None and seen[2] >= cfg.min_confidence:
                break
            seen = None
        result.commanded = (cx, cy)
        if seen is None:
            result.steps.append(ServoStep(iteration, (cx, cy), None, None, "the tip could not be seen"))
            result.reason = "the tip could not be seen: the open-loop position was kept" if iteration == 0 else "lost sight of the tip"
            return result
        ox, oy, _confidence = seen
        ex, ey = tx - ox, ty - oy
        error_mm = math.hypot(ex, ey) * 1000
        result.final_error_mm = error_mm
        if error_mm <= cfg.tol_mm:
            result.steps.append(ServoStep(iteration, (cx, cy), (ox, oy), error_mm, "within tolerance"))
            result.ok, result.reason = True, f"within {cfg.tol_mm:g} mm"
            return result
        if iteration == cfg.max_iters:
            result.steps.append(ServoStep(iteration, (cx, cy), (ox, oy), error_mm, "out of iterations"))
            result.reason = f"did not converge: still {error_mm:.1f} mm away after {cfg.max_iters} corrections"
            return result
        step = (ex * cfg.gain, ey * cfg.gain)
        norm = math.hypot(*step) * 1000
        if norm > cfg.max_step_mm:                              # cap one correction
            scale = cfg.max_step_mm / norm
            step = (step[0] * scale, step[1] * scale)
        nx, ny = cx + step[0], cy + step[1]
        drift = math.hypot(nx - tx, ny - ty) * 1000
        if drift > cfg.max_total_mm:                            # never wander far from what the user tapped
            scale = cfg.max_total_mm / drift
            nx, ny = tx + (nx - tx) * scale, ty + (ny - ty) * scale
        result.steps.append(ServoStep(iteration, (cx, cy), (ox, oy), error_mm, f"correcting by {math.hypot(nx - cx, ny - cy) * 1000:.1f} mm"))
        cx, cy = nx, ny
    return result
