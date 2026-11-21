"""Advisory local fixed-tilt travel, computed only by a private worker.

These sampled estimates are not global reach or collision guarantees. They never
authorize motion; the controller validates each actual command independently.
"""
from __future__ import annotations

import copy
import math
import threading
import time
from collections.abc import Callable, Mapping
from typing import Any

from .panel_cartesian import OBSERVATION_MAX_AGE_S, Workspace

DIRECTIONS = ("forward", "back", "up", "down")
RESOLUTION_M = 0.0005
HORIZON_M = 0.4


def _unknown(reason: str) -> dict:
    return {"allowed": False, "distance_m": None, "limiting_reason": reason,
            "limiting_joint": None, "at_least": False}


def estimate_ranges(solver: Any, joints: Mapping[str, float], limits: Mapping,
                    workspace: Workspace, *, budget_s: float = 2.0,
                    clock: Callable[[], float] = time.monotonic) -> dict:
    """March each connected local line from its measured seed, then bisect failure.

    The deadline is checked between bounded solver calls; a single call cannot be
    preempted. A budget failure yields unknown, never a guessed distance.
    """
    result = {"status": "unavailable", "orientation": "fixed_tilt",
              "resolution_m": RESOLUTION_M, "horizon_m": HORIZON_M,
              "directions": {name: _unknown("Range estimate unavailable.") for name in DIRECTIONS}}
    deadline = clock() + budget_s
    floor = workspace.snapshot()["min_tcp_z_m"]
    try:
        if workspace.error or floor is None:
            raise ValueError(workspace.error or "Configure the TCP floor to estimate travel.")
        for joint, value in joints.items():
            if joint not in limits or not math.isfinite(value):
                raise ValueError("Measured joint pose is incomplete or invalid.")
            lo, hi = limits[joint]
            if value < lo - 1e-8 or value > hi + 1e-8:
                raise ValueError(f"{joint} is outside its configured limit; recovery is required.")
        if any(joint not in joints for joint in limits):
            raise ValueError("Measured joint pose is incomplete.")
        origin = solver.position(joints)
        heading = solver.heading(joints)
        pitch = solver.pitch(joints)
        if not all(math.isfinite(value) for value in (*origin, *heading, pitch)):
            raise ValueError("Measured TCP pose is invalid.")
        if origin[2] < floor - 1e-8:
            raise ValueError("Measured TCP is below the configured floor; recovery is required.")
        solver.solve(joints, origin, pitch=pitch)
        result["origin"] = {"x_m": origin[0], "y_m": origin[1], "z_m": origin[2],
                            "radial_m": origin[0] * heading[0] + origin[1] * heading[1],
                            "pitch_deg": math.degrees(pitch)}
    except Exception as exc:
        result["directions"] = {name: _unknown(str(exc)) for name in DIRECTIONS}
        return result

    for name in DIRECTIONS:
        vector = ((heading[0], heading[1], 0.0) if name == "forward" else
                  (-heading[0], -heading[1], 0.0) if name == "back" else
                  (0.0, 0.0, 1.0 if name == "up" else -1.0))
        current = dict(joints)
        low, high = 0.0, 0.0
        directional_horizon = min(HORIZON_M, max(0.0, origin[2] - floor)) if name == "down" else HORIZON_M
        reason = "Local solver / position-orientation boundary."
        try:
            while low < directional_horizon - 1e-9:
                if clock() >= deadline:
                    raise TimeoutError("Range estimation time budget reached.")
                high = min(directional_horizon, low + 0.01)
                target = tuple(origin[i] + high * vector[i] for i in range(3))
                if name == "down" and high == directional_horizon:
                    target = (target[0], target[1], max(floor, target[2]))
                if target[2] < floor:
                    reason = "Configured TCP floor."
                    break
                try:
                    candidate = solver.solve(current, target, pitch=pitch)
                except ValueError:
                    break
                current, low = candidate, high
            while high - low > RESOLUTION_M:
                if clock() >= deadline:
                    raise TimeoutError("Range estimation time budget reached.")
                mid = (low + high) / 2
                target = tuple(origin[i] + mid * vector[i] for i in range(3))
                if target[2] < floor:
                    reason = "Configured TCP floor."
                    high = mid
                    continue
                try:
                    candidate = solver.solve(current, target, pitch=pitch)
                except ValueError:
                    high = mid
                else:
                    current, low = candidate, mid
            at_least = low >= HORIZON_M - 1e-9
            if not at_least and name == "down" and low >= directional_horizon - 1e-9:
                reason = "Configured TCP floor."
            # Attribute only a joint actually at a known bound. Otherwise the
            # bounded local solver cannot distinguish a limit from singularity.
            limiting_joint = next((joint for joint in ("shoulder_lift", "elbow_flex", "wrist_flex")
                                   if joint in current and joint in limits
                                   and min(abs(current[joint] - bound) for bound in limits[joint]) < 0.02),
                                  None)
            result["directions"][name] = {
                "allowed": low >= RESOLUTION_M, "distance_m": low,
                "limiting_reason": None if at_least else reason,
                "limiting_joint": (limiting_joint
                                   if not at_least and reason != "Configured TCP floor." else None),
                "at_least": at_least}
        except Exception as exc:
            result["directions"][name] = _unknown(str(exc))
    result["status"] = ("ready" if all(item["distance_m"] is not None
                                       for item in result["directions"].values()) else "unavailable")
    return result


class CartesianRangeEstimator:
    """Latest-observation mailbox; all geometry stays outside the control lock.

    At most one computation per second. A changed pose invalidates an in-flight
    result. Exact pose comparison is intentional: noisy feedback may show stale
    ranges, but cannot make an estimate from another pose appear fresh.
    """

    def __init__(self, solver_factory: Callable[[], Any], limits: Mapping,
                 workspace: Workspace, *, interval_s: float = 1.0, budget_s: float = 2.0) -> None:
        self._factory = solver_factory
        self._limits = dict(limits)
        self._workspace = workspace
        self._interval = interval_s
        self._budget = budget_s
        self._condition = threading.Condition()
        self._latest: tuple[dict, int, float] | None = None
        self._computed_pose: dict | None = None
        self._result: dict | None = None
        self._result_observed_at = 0.0
        self._computed_at: float | None = None
        self._disabled_reason = (workspace.error or
                                 ("Configure the TCP floor to estimate travel."
                                  if workspace.table_z_m is None else None))
        self._stopped = False
        self._thread: threading.Thread | None = None

    def observe(self, joints: Mapping[str, float], observation_sequence: int, observed_at: float) -> None:
        with self._condition:
            if self._stopped:
                return
            self._latest = (dict(joints), observation_sequence, observed_at)
            if self._disabled_reason:
                self._result = {"status": "unavailable", "orientation": "fixed_tilt",
                                "resolution_m": RESOLUTION_M, "horizon_m": HORIZON_M,
                                "directions": {name: _unknown(self._disabled_reason) for name in DIRECTIONS}}
                self._computed_pose = dict(joints)
                return
            if self._thread is None:
                self._thread = threading.Thread(target=self._run, name="cartesian-ranges", daemon=True)
                self._thread.start()
            self._condition.notify()

    def snapshot(self) -> dict:
        now = time.monotonic()
        with self._condition:
            result = copy.deepcopy(self._result) if self._result is not None else {
                "status": "pending", "orientation": "fixed_tilt", "resolution_m": RESOLUTION_M,
                "horizon_m": HORIZON_M, "directions": {name: _unknown("Computing travel.")
                                                        for name in DIRECTIONS}}
            latest = self._latest
            result.update(source_pose=copy.deepcopy(self._computed_pose),
                          computed_age_s=(None if self._computed_at is None
                                          else max(0.0, now - self._computed_at)),
                          latest_pose_delta_deg=None, stale_reason=None)
            if latest is None:
                result.update(observation_sequence=None, age_s=None)
            else:
                matching = latest[0] == self._computed_pose
                if self._computed_pose is not None:
                    result["latest_pose_delta_deg"] = max(
                        (abs(value - self._computed_pose[joint]) for joint, value in latest[0].items()
                         if joint != "gripper" and joint in self._computed_pose), default=0.0)
                if matching or self._result is None:
                    result.update(observation_sequence=latest[1], age_s=max(0.0, now - latest[2]))
                else:
                    result["age_s"] = max(0.0, now - self._result_observed_at)
                if now - latest[2] > OBSERVATION_MAX_AGE_S:
                    result["status"] = "stale"
                    result["stale_reason"] = "stale_feedback"
                elif self._result is not None and not matching:
                    result["status"] = "stale"
                    result["stale_reason"] = "stale_pose"
            return result

    def close(self) -> None:
        with self._condition:
            self._stopped = True
            self._condition.notify()
        if self._thread is not None:
            self._thread.join(timeout=0.05)

    def _run(self) -> None:
        solver = None
        next_at = 0.0
        while True:
            observation = None
            with self._condition:
                while not self._stopped:
                    now = time.monotonic()
                    if (self._latest is not None and now >= next_at
                            and (self._result is None or self._latest[0] != self._computed_pose)):
                        observation = self._latest
                        break
                    self._condition.wait(timeout=max(0.01, next_at - now) if now < next_at else None)
                if self._stopped:
                    return
            if observation is None:
                continue
            joints, sequence, observed_at = observation
            try:
                if solver is None:
                    solver = self._factory()
                result = estimate_ranges(solver, joints, self._limits, self._workspace, budget_s=self._budget)
            except Exception as exc:
                result = {"status": "unavailable", "orientation": "fixed_tilt",
                          "resolution_m": RESOLUTION_M, "horizon_m": HORIZON_M,
                          "directions": {name: _unknown(f"Cartesian kinematics unavailable: {exc}")
                                         for name in DIRECTIONS}}
            with self._condition:
                # Publish the source identity too; snapshot marks mismatched poses
                # stale and never presents these as fresh current-pose estimates.
                self._computed_pose = joints
                self._result_observed_at = observed_at
                self._computed_at = time.monotonic()
                result.update(observation_sequence=sequence, source_observation_sequence=sequence,
                              age_s=max(0.0, self._computed_at - observed_at))
                self._result = result
                next_at = time.monotonic() + self._interval
