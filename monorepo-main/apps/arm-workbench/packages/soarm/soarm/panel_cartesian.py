"""Local position tracking on the optional twin model; no hardware or renderer.

The configured floor limits the TCP only. It does not model a table, camera,
links, fixtures or collisions. All positions use the robot base frame.
"""
from __future__ import annotations

import math
from collections.abc import Mapping
from dataclasses import dataclass
from itertools import product
from pathlib import Path

WATCHDOG_S = 0.35
OBSERVATION_MAX_AGE_S = 0.75
TILT_FLOOR_NOISE_M = 0.0002
TILT_FLOOR_PROGRESS_TIMEOUT_S = 1.0
PROFILES = {
    "slow": {"linear_m_s": 0.01, "angular_deg_s": 5.0, "joint_deg_s": 8.0, "gripper_units_s": 10.0},
    "medium": {"linear_m_s": 0.025, "angular_deg_s": 10.0, "joint_deg_s": 15.0, "gripper_units_s": 20.0},
    "fast": {"linear_m_s": 0.05, "angular_deg_s": 20.0, "joint_deg_s": 25.0, "gripper_units_s": 35.0},
}
TRANSLATION_DIRECTIONS = frozenset(("forward", "back", "up", "down"))
TILT_DIRECTIONS = frozenset(("tilt_up", "tilt_down"))
ROLL_DIRECTIONS = frozenset(("roll_left", "roll_right"))
ORIENTATION_DIRECTIONS = TILT_DIRECTIONS | ROLL_DIRECTIONS
DIRECTIONS = (TRANSLATION_DIRECTIONS | ORIENTATION_DIRECTIONS
              | frozenset(("left", "right", "open", "close")))


@dataclass(frozen=True)
class Workspace:
    table_z_m: float | None = None
    tcp_margin_m: float = 0.02
    error: str | None = None

    @classmethod
    def from_environment(cls, environment: Mapping[str, str]) -> Workspace:
        try:
            raw = environment.get("RAMION_CARTESIAN_TABLE_Z_M")
            table = float(raw) if raw is not None else None
            margin = float(environment.get("RAMION_CARTESIAN_TCP_MARGIN_M", "0.02"))
            if (table is not None and not math.isfinite(table)) or not math.isfinite(margin) or margin < 0:
                raise ValueError("nonfinite table height or invalid margin")
            return cls(table, margin)
        except ValueError:
            return cls(error="Invalid Cartesian table height or TCP margin configuration.")

    def snapshot(self) -> dict:
        return {"configured": self.table_z_m is not None and self.error is None,
                "table_z_m": self.table_z_m, "tcp_margin_m": self.tcp_margin_m,
                "min_tcp_z_m": None if self.table_z_m is None else self.table_z_m + self.tcp_margin_m,
                "error": self.error}


class CartesianSolver:
    """Lazy optional twin adapter. Single-seed IK never restarts on another branch."""

    def __init__(self, robot_dir: Path, limits: Mapping[str, tuple[float, float]]) -> None:
        import numpy as np
        from machinekind_twin.kinematics import RobotKinematics
        from machinekind_twin.robots import load_robot

        self.kin = RobotKinematics(load_robot(robot_dir))
        self.kin.arm = np.array([name in ("shoulder_lift", "elbow_flex", "wrist_flex")
                                 for name in self.kin.spec.joints])
        for index, joint in enumerate(self.kin.spec.joints):
            if joint != self.kin.spec.gripper:
                self.kin.lo[index] = max(self.kin.lo[index], math.radians(limits[joint][0]))
                self.kin.hi[index] = min(self.kin.hi[index], math.radians(limits[joint][1]))
        self._pitch_joints = ("shoulder_lift", "elbow_flex", "wrist_flex")
        self._limits = {joint: (max(limits[joint][0], math.degrees(self.kin.lo[index])),
                                min(limits[joint][1], math.degrees(self.kin.hi[index])))
                        for index, joint in enumerate(self.kin.spec.joints) if joint in self._pitch_joints}

    def position(self, joints: Mapping[str, float]) -> tuple[float, float, float]:
        position = self.kin.tcp(joints)[:3, 3]
        return float(position[0]), float(position[1]), float(position[2])

    def heading(self, joints: Mapping[str, float]) -> tuple[float, float]:
        # SO-101 shoulder local -X points forward; its hinge axis points down.
        # Using the actual model transform preserves the pitch plane's small lateral
        # TCP offset, unlike normalizing TCP XY about the base frame origin.
        shoulder = self.kin.body("shoulder", joints)
        x, y = -float(shoulder[0, 0]), -float(shoulder[1, 0])
        length = math.hypot(x, y)
        if length < 0.001:
            raise ValueError("Shoulder heading is undefined.")
        return x / length, y / length

    def pitch(self, joints: Mapping[str, float]) -> float:
        """Tool approach elevation in its shoulder plane, in radians."""
        import numpy as np

        transform = self.kin.tcp(joints)
        approach = transform[:3, :3] @ np.asarray(self.kin.spec.tcp_approach, float)
        heading = self.heading(joints)
        horizontal = float(approach[0]) * heading[0] + float(approach[1]) * heading[1]
        return math.atan2(float(approach[2]), horizontal)

    @staticmethod
    def _angle(value: float) -> float:
        return math.atan2(math.sin(value), math.cos(value))

    @staticmethod
    def _bounded_step(jacobian, error, lower, upper):
        """Exact tiny box least squares by enumerating three-variable active sets."""
        import numpy as np

        # Interior Cartesian steps have a square, full-rank system. Avoid active
        # set enumeration when the exact unconstrained step already fits its box.
        try:
            direct = np.linalg.solve(jacobian, error)
        except np.linalg.LinAlgError:
            direct = np.linalg.lstsq(jacobian, error, rcond=1e-10)[0]
        if np.all(direct >= lower - 1e-9) and np.all(direct <= upper + 1e-9):
            return direct
        best = None
        count = jacobian.shape[1]
        for status in product((-1, 0, 1), repeat=count):
            free = np.array([index for index, item in enumerate(status) if item == 0], dtype=int)
            fixed = np.array([index for index, item in enumerate(status) if item != 0], dtype=int)
            step = np.zeros(count)
            for index in fixed:
                step[index] = lower[index] if status[index] < 0 else upper[index]
            residual = error - jacobian[:, fixed] @ step[fixed]
            if len(free):
                step[free] = np.linalg.lstsq(jacobian[:, free], residual, rcond=1e-10)[0]
            if np.any(step < lower - 1e-9) or np.any(step > upper + 1e-9):
                continue
            score = float(np.linalg.norm(jacobian @ step - error) ** 2 + 1e-14 * np.linalg.norm(step) ** 2)
            if best is None or score < best[0]:
                best = score, step
        if best is None:
            raise ValueError("No bounded local joint step is available.")
        return best[1]

    def solve(self, joints: Mapping[str, float], position: tuple[float, float, float],
              *, pitch: float | None = None) -> dict[str, float]:
        """Track position and tool pitch with bounded local steps and no seed clipping.

        The three pitch joints solve radial position, height and approach elevation.
        Pan and roll stay exactly at the supplied seed. A measured pose beyond its
        configured limits needs a separately authorized recovery policy.
        """
        import numpy as np

        result = dict(joints)
        q = np.array([joints[joint] for joint in self._pitch_joints], float)
        lower = np.array([self._limits[joint][0] for joint in self._pitch_joints])
        upper = np.array([self._limits[joint][1] for joint in self._pitch_joints])
        if np.any(q < lower - 1e-8) or np.any(q > upper + 1e-8):
            raise ValueError("Measured pitch joint is outside its configured limit; recovery is required.")
        heading = np.array(self.heading(joints))
        target = np.asarray(position, float)
        initial = np.array(self.position(joints))
        lateral = np.array([-heading[1], heading[0]])
        if abs(float((target[:2] - initial[:2]) @ lateral)) > 1e-5:
            raise ValueError("Requested position is outside the fixed shoulder plane.")
        wanted_pitch = self.pitch(joints) if pitch is None else float(pitch)
        approach_axis = np.asarray(self.kin.spec.tcp_approach, float)

        def apply(values):
            for joint, value in zip(self._pitch_joints, values, strict=True):
                result[joint] = float(value)
            transform = self.kin.tcp(result)
            point = transform[:3, 3]
            approach = transform[:3, :3] @ approach_axis
            elevation = math.atan2(float(approach[2]), float(approach[:2] @ heading))
            return np.array([float(point[:2] @ heading), point[2], elevation])

        wanted = np.array([float(target[:2] @ heading), target[2], wanted_pitch])
        for _ in range(24):
            current = apply(q)
            error = wanted - current
            error[2] = self._angle(float(error[2]))
            if np.linalg.norm(error[:2]) <= 0.00002 and abs(error[2]) <= math.radians(0.01):
                return dict(result)
            jacobian = np.empty((3, 3))
            for index in range(3):
                perturbed = q.copy()
                perturbed[index] += 0.001
                delta = apply(perturbed) - current
                delta[2] = self._angle(float(delta[2]))
                jacobian[:, index] = delta / 0.001
            # Metres and pitch radians have explicit comparable scale in the
            # least-squares objective. Both tolerances above must still pass.
            scale = np.array([1.0, 1.0, 0.1])
            weighted = scale[:, None] * jacobian
            step = self._bounded_step(weighted, scale * error,
                                      np.maximum(lower - q, -3.0), np.minimum(upper - q, 3.0))
            previous_error = float(np.linalg.norm(scale * error))
            accepted = False
            for attempt in range(8):
                candidate = q + step * (0.5 ** attempt)
                next_error = wanted - apply(candidate)
                next_error[2] = self._angle(float(next_error[2]))
                if float(np.linalg.norm(scale * next_error)) < previous_error - 1e-12:
                    q = candidate
                    accepted = True
                    break
            if not accepted:
                break
        raise ValueError("Fixed-tilt Cartesian path reaches a joint limit or local reach boundary.")

    def solve_position(self, joints: Mapping[str, float], position: tuple[float, float, float],
                       *, pitch_anchor: float, pitch_budget: float,
                       bounds: Mapping[str, tuple[float, float]] | None = None,
                       fixed: Mapping[str, float] | None = None) -> dict[str, float]:
        """Explicit recovery only: exact local position with a total tilt budget.

        Supplied ingress bounds are local to this solve; normal limits are never
        modified. Fixed coordinates let an ingress plan repair a violated joint
        monotonically while the remaining two joints compensate the TCP.
        """
        import numpy as np

        allowed = self._limits if bounds is None else bounds
        fixed = fixed or {}
        names = tuple(joint for joint in self._pitch_joints if joint not in fixed)
        result = dict(joints)
        result.update(fixed)
        q = np.array([result[joint] for joint in names], float)
        low = np.array([allowed[joint][0] for joint in names])
        high = np.array([allowed[joint][1] for joint in names])
        if any(not allowed[joint][0] - 1e-9 <= result[joint] <= allowed[joint][1] + 1e-9
               for joint in self._pitch_joints):
            raise ValueError("Recovery seed is outside its verified envelope.")
        heading = np.asarray(self.heading(joints), float)
        lateral = np.array([-heading[1], heading[0]])
        origin = np.asarray(self.position(joints), float)
        target = np.asarray(position, float)
        if abs(float((target[:2] - origin[:2]) @ lateral)) > 1e-5:
            raise ValueError("Recovery target is outside the fixed shoulder plane.")
        wanted = np.array([float(target[:2] @ heading), target[2]])

        def apply(values):
            for joint, value in zip(names, values, strict=True):
                result[joint] = float(value)
            point = self.kin.tcp(result)[:3, 3]
            return np.array([float(point[:2] @ heading), point[2]])

        for _ in range(24):
            current = apply(q)
            error = wanted - current
            if np.linalg.norm(error) <= 0.00002:
                if abs(self._angle(self.pitch(result) - pitch_anchor)) > pitch_budget:
                    raise ValueError("Recovery reached its total tilt budget.")
                return dict(result)
            jacobian = np.empty((2, len(names)))
            for index in range(len(names)):
                perturbed = q.copy()
                perturbed[index] += 0.001
                jacobian[:, index] = (apply(perturbed) - current) / 0.001
            step = self._bounded_step(jacobian, error, np.maximum(low - q, -3.0),
                                      np.minimum(high - q, 3.0))
            accepted = False
            previous_error = float(np.linalg.norm(error))
            for attempt in range(8):
                candidate = q + step * (0.5 ** attempt)
                if float(np.linalg.norm(wanted - apply(candidate))) < previous_error - 1e-12:
                    q = candidate
                    accepted = True
                    break
            if not accepted:
                break
        raise ValueError("Recovery path reaches a joint limit or local reach boundary.")
