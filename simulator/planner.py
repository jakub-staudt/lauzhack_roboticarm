"""Pick-and-place planning for the SO-101 in the gaze station, on the MuJoCo kinematics (no simulator needed).

A plan is a list of `(label, joints, dwell_s)`: joint targets in radians keyed by joint name, held for `dwell_s` seconds.
Both MuJoCo and Gazebo take these directly (position-controlled joints).

The grasp is top-down. The gripper's yaw is one of 0/90/180/270 degrees, so a held cube stays aligned with the grid.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from simulator.frame import TABLE_Z
from simulator.run import build_model, load_station
from soarm_lab.kinematics import RobotKinematics
from soarm_lab.station import PREFIX

CUBE = 0.02
GRIP_Z = TABLE_Z + CUBE / 2 + 0.002   # tip height when grasping / placing: cube centre
HOVER = 0.045                         # extra tip height for travel: a carried 20 mm cube still clears the table by 3 cm
GRIPPER_OPEN, GRIPPER_CLOSED = 45.0, 0.0
# Motion follows minimum-jerk profiles (smooth start and stop, peak speed = 1.875 x the average), sampled every SAMPLE_DT.
SAMPLE_DT = 0.05                      # seconds between waypoints
LINEAR_V = 0.09                       # peak tip speed [m/s]; the real arm's "fast" profile is 0.05
JOINT_W = 0.9                         # peak joint speed [rad/s] (about 50 deg/s)
GRIPPER_RATE = 60.0                   # peak gripper speed [units/s]
MIN_MOVE_S = 0.4                      # even a tiny move takes at least this long
SETTLE_S = 0.3                        # pause where the arm comes to rest: before closing and after opening the gripper
GRASP_SHIFT = 0.0095                  # the cube slides ~9.5 mm against the fixed jaw when grasped; aim the drop that far along the jaw axis
MAX_ROT_ERR = 0.2                     # rad: accepted deviation from a top-down gripper
YAWS = (0.0, math.pi / 2, -math.pi / 2, math.pi)

Plan = list[tuple[str, dict[str, float], float]]


class Unreachable(Exception):
    """The arm cannot do what was asked; the message says why."""


def _rz(angle: float) -> np.ndarray:
    c, s = math.cos(angle), math.sin(angle)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


# Tool X points down; tool Z (the direction the jaws close along) points along world +x for yaw 0.
_TOP_DOWN = np.array([[0.0, 0.0, 1.0], [0.0, 1.0, 0.0], [-1.0, 0.0, 0.0]])


class Planner:
    def __init__(self) -> None:
        self.station = load_station()
        self.model = build_model(self.station)
        self.robot = self.station.spec()
        self.kin = RobotKinematics(self.robot, self.model, prefix=PREFIX)
        self.base = np.array(self.station.T_base2world[:3, 3], float)
        self.home = dict(self.station.home or self.robot.home)

    # ------------------------------------------------------------------ helpers
    def to_app(self, rad: dict[str, float]) -> dict[str, float]:
        """Joint radians by name (as Gazebo reports them) -> the kinematics' joint units."""
        return self.kin.from_q(np.array([rad[n] for n in self.robot.joints], float))

    def to_rad(self, joints: dict[str, float]) -> dict[str, float]:
        return {n: float(v) for n, v in zip(self.robot.joints, self.kin.to_q(joints), strict=True)}

    def _solve(self, xy: tuple[float, float], z: float, yaw: float, seed: dict[str, float]):
        target = np.array([xy[0], xy[1], z]) - self.base
        rotation = _rz(yaw) @ _TOP_DOWN
        return self.kin.ik(target, rotation, seed, rot_gain=0.8, tol=0.0005, iters=300, restarts=2,
                           accept=lambda j: not self.kin.self_contacts(j)), rotation

    def _good(self, sol) -> bool:
        return sol.ok and sol.rot_err < MAX_ROT_ERR

    def _poses(self, xy, seed: dict[str, float], yaws=YAWS):
        """{yaw: (hover joints, grasp joints)} for the top-down yaws that reach both heights over `xy`."""
        found = {}
        for yaw in yaws:
            low, _ = self._solve(xy, GRIP_Z, yaw, seed)
            if not self._good(low):
                continue
            high, _ = self._solve(xy, GRIP_Z + HOVER, yaw, low.joints)
            if self._good(high):
                found[yaw] = (high.joints, low.joints)
        return found

    def reachable(self, xy: tuple[float, float]) -> bool:
        return bool(self._poses(xy, {**self.home, "gripper": GRIPPER_OPEN}))

    # ------------------------------------------------------------------ motion building
    @staticmethod
    def _min_jerk(tau: float) -> float:
        return tau**3 * (10 - 15 * tau + 6 * tau * tau)

    @staticmethod
    def _timing(length: float, peak: float) -> tuple[int, float]:
        """(samples, seconds per sample) for a move of `length` at peak speed `peak` with a minimum-jerk profile."""
        duration = max(MIN_MOVE_S, 1.875 * length / peak)
        n = max(2, int(np.ceil(duration / SAMPLE_DT)))
        return n, duration / n

    def _line(self, plan: Plan, label: str, start: np.ndarray, end: np.ndarray, rotation: np.ndarray,
              joints: dict[str, float]) -> dict[str, float]:
        """Straight Cartesian move (base frame), tracked by IK from `joints`; appends waypoints."""
        n, dt = self._timing(float(np.linalg.norm(end - start)), LINEAR_V)
        for i in range(1, n + 1):
            point = start + (end - start) * self._min_jerk(i / n)
            sol = self.kin.ik_step(point, rotation, joints, rot_gain=0.8, tol=0.0008, iters=120)
            if not self._good(sol):
                raise Unreachable("The straight path to that spot leaves the arm's reachable area.")
            joints = {**sol.joints, "gripper": joints["gripper"]}
            plan.append((label, self.to_rad(joints), dt))
        return joints

    def _joint_move(self, plan: Plan, label: str, start: dict[str, float], end: dict[str, float]) -> dict[str, float]:
        q0, q1 = self.kin.to_q(start), self.kin.to_q(end)
        n, dt = self._timing(float(np.max(np.abs(q1 - q0))), JOINT_W)
        for i in range(1, n + 1):
            q = q0 + (q1 - q0) * self._min_jerk(i / n)
            plan.append((label, {nm: float(v) for nm, v in zip(self.robot.joints, q, strict=True)}, dt))
        return end

    def _gripper(self, plan: Plan, label: str, joints: dict[str, float], target: float) -> dict[str, float]:
        start = joints["gripper"]
        if abs(start - target) < 1.0:
            return joints  # already there
        n, dt = self._timing(abs(target - start), GRIPPER_RATE)
        for i in range(1, n + 1):
            joints = {**joints, "gripper": float(start + (target - start) * self._min_jerk(i / n))}
            plan.append((label, self.to_rad(joints), dt))
        plan.append((label, self.to_rad(joints), SETTLE_S))  # let the grip build up / the jaws come to rest
        return joints

    # ------------------------------------------------------------------ plans
    def pick_and_place(self, current: dict[str, float], src: tuple[float, float], dst: tuple[float, float]) -> Plan:
        """`current`: measured joints in radians by name. `src`, `dst`: world (x, y) of the cube centres."""
        seed = {**self.to_app(current)}
        seed["gripper"] = GRIPPER_OPEN
        yaw, pick, place, why = None, None, None, ""
        for candidate in YAWS:  # the first top-down orientation that reaches both spots wins
            at_src = self._poses(src, seed, (candidate,))
            if not at_src:
                why = why or "The arm cannot reach the block at the source spot with a top-down grasp."
                continue
            drop = (dst[0] + GRASP_SHIFT * math.cos(candidate), dst[1] + GRASP_SHIFT * math.sin(candidate))
            at_dst = self._poses(drop, seed, (candidate,))
            if not at_dst:
                why = "The arm cannot reach the destination spot with a top-down grasp."
                continue
            yaw, pick, place = candidate, at_src, at_dst
            break
        if yaw is None:
            raise Unreachable(why)
        rotation = _rz(yaw) @ _TOP_DOWN
        (src_hover, src_low), (dst_hover, dst_low) = pick[yaw], place[yaw]
        pos = lambda j: np.array(self.kin.tcp(j)[:3, 3], float)  # noqa: E731

        plan: Plan = []
        joints = self.to_app(current)
        joints = self._gripper(plan, "open", joints, GRIPPER_OPEN)
        joints = self._joint_move(plan, "to source", joints, {**src_hover, "gripper": GRIPPER_OPEN})
        joints = self._line(plan, "lower", pos(joints), pos(src_low), rotation, joints)
        joints = self._gripper(plan, "grasp", joints, GRIPPER_CLOSED)
        joints = self._line(plan, "lift", pos(joints), pos(src_hover), rotation, joints)
        joints = self._line(plan, "carry", pos(joints), pos(dst_hover), rotation, joints)
        joints = self._line(plan, "lower", pos(joints), pos(dst_low) + [0, 0, 0.003], rotation, joints)
        joints = self._gripper(plan, "release", joints, GRIPPER_OPEN)
        joints = self._line(plan, "rise", pos(joints), pos(dst_hover), rotation, joints)
        self._joint_move(plan, "home", joints, {**self.home, "gripper": GRIPPER_OPEN})
        return plan

    def go_to(self, current: dict[str, float], xy: tuple[float, float], height: float = 0.03) -> Plan:
        """Move the tip over `xy` at `height` above the table, gripper unchanged, tool pointing down."""
        seed = self.to_app(current)
        for yaw in YAWS:
            sol, _ = self._solve(xy, TABLE_Z + height, yaw, seed)
            if self._good(sol):
                plan: Plan = []
                self._joint_move(plan, "move", seed, {**sol.joints, "gripper": seed["gripper"]})
                return plan
        raise Unreachable("The arm cannot reach that spot.")


def write_reach_map(path: str = "simulator/web/reach_map.json", step_cm: int = 2) -> None:
    """Which spots of each block can be picked/placed (about 0.25 s per spot; run once after changing the geometry)."""
    import json

    from simulator.frame import SIDE_CM, block_to_world

    planner = Planner()
    grid = range(0, int(SIDE_CM) + 1, step_cm)
    result = {"step_cm": step_cm, "hover": HOVER, "blocks": {
        block: [[cx, cy] for cy in grid for cx in grid if planner.reachable(block_to_world(block, cx, cy))]
        for block in ("left", "right")}}
    with open(path, "w") as f:
        json.dump(result, f)
    print(f"wrote {path}: " + ", ".join(f"{b} {len(v)} reachable spots" for b, v in result["blocks"].items()))


if __name__ == "__main__":
    write_reach_map()
