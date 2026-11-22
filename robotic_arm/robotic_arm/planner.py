"""Pure planning: (u, v) pick/place points -> a joint-space trajectory. No network, no hardware.

Sequence: open, hover over pick, descend, close, lift, travel to place at hover height, descend, open, lift, home.
Each leg is linear in joint space (eased) at a bounded joint speed; the gripper moves only in its own segments.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

from .calibration import Calibration
from .config import Config

ARM = ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll")
JOINTS = (*ARM, "gripper")


class PlanError(RuntimeError):
    """A point cannot be reached safely."""


@dataclass
class Segment:
    name: str
    start: int  # index of the first row in Plan.rows
    end: int  # exclusive


@dataclass
class Plan:
    rows: list[dict[str, float]] = field(default_factory=list)
    segments: list[Segment] = field(default_factory=list)
    waypoints: dict[str, dict[str, float]] = field(default_factory=dict)
    pitch_deg: dict[str, float] = field(default_factory=dict)

    def duration_s(self, hz: float) -> float:
        return len(self.rows) / hz

    def chunks(self, hz: float, max_s: float) -> list[list[dict[str, float]]]:
        """Row groups of at most `max_s` seconds, split only at segment boundaries."""
        limit = int(max_s * hz)
        out: list[list[dict[str, float]]] = []
        cur_start = 0
        last_end = 0
        for seg in self.segments:
            if seg.end - cur_start > limit and last_end > cur_start:
                out.append(self.rows[cur_start:last_end])
                cur_start = last_end
            if seg.end - cur_start > limit:
                raise PlanError(f"Segment {seg.name} alone exceeds {max_s:g} s; raise speed or shorten the move.")
            last_end = seg.end
        out.append(self.rows[cur_start:last_end])
        return out


def tool_rotation(theta: float, pitch_deg: float) -> np.ndarray:
    """Tool frame in the base frame for a gripper pointing `pitch_deg` below horizontal, towards radial angle theta.

    The tool's approach axis is its x axis and its jaw closing axis its z axis (robot.yaml). The jaws close
    along the tangent so that, with wrist_roll near 0, they straddle a block seen from the arm.
    """
    p = math.radians(pitch_deg)
    radial = np.array([math.cos(theta), math.sin(theta), 0.0])
    x = math.cos(p) * radial + np.array([0.0, 0.0, -math.sin(p)])
    z = np.array([-math.sin(theta), math.cos(theta), 0.0])
    y = np.cross(z, x)
    return np.column_stack([x, y, z])


class Planner:
    def __init__(self, kin, calibration: Calibration, cfg: Config):
        self.kin = kin
        self.cal = calibration
        self.cfg = cfg
        self.limits = kin.spec.limits

    # ------------------------------------------------------------------ geometry
    def check_uv(self, u: float, v: float) -> None:
        m = self.cfg.uv_margin
        if not (math.isfinite(u) and math.isfinite(v)) or not (-m <= u <= 1 + m and -m <= v <= 1 + m):
            raise PlanError(f"Point ({u}, {v}) is outside the rectangle.")

    def _within_limits(self, joints: dict[str, float]) -> bool:
        return all(self.limits[j][0] <= joints[j] <= self.limits[j][1] for j in ARM)

    def _solve(self, x: float, y: float, z: float, pitch: float, seed: dict[str, float]):
        sol = self.kin.ik(np.array([x, y, z]), tool_rotation(math.atan2(y, x), pitch), seed=seed,
                          tol=self.cfg.ik_tol_m)
        if not sol.ok or sol.rot_err > self.cfg.max_rot_err_rad or not self._within_limits(sol.joints):
            return None
        return sol.joints

    def solve_column(self, u: float, v: float, seed: dict[str, float], zs: dict[str, float]):
        """IK for several heights over one point with a single pitch. Returns (pitch, {name: joints})."""
        self.check_uv(u, v)
        x, y = self.cal.xy(u, v)
        for pitch in self.cfg.pitches_deg:
            chain, cur, ok = {}, seed, True
            for name, z in zs.items():  # in order: hover first, so each solution seeds the next
                joints = self._solve(x, y, self.cal.z_table + z, pitch, cur)
                if joints is None:
                    ok = False
                    break
                chain[name], cur = joints, joints
            if ok:
                return pitch, chain
        raise PlanError(f"({u:.2f}, {v:.2f}) is not reachable with a safe approach "
                        f"(tip at {x:.3f}, {y:.3f} m from the base).")

    # ------------------------------------------------------------------ trajectories
    @staticmethod
    def _ease(s: np.ndarray) -> np.ndarray:
        return s * s * s * (s * (6 * s - 15) + 10)

    def _leg(self, a: dict[str, float], b: dict[str, float]) -> list[dict[str, float]]:
        """Arm joints a -> b at the bounded speed, gripper unchanged (a's)."""
        delta = max(abs(b[j] - a[j]) for j in ARM)
        # 1.875 is the peak slope of the easing, so the peak joint speed stays at cfg.arm_speed.
        n = max(2, math.ceil(1.875 * delta / self.cfg.arm_speed * self.cfg.hz))
        out = []
        for s in self._ease(np.linspace(0, 1, n + 1)[1:]):
            row = {j: a[j] + (b[j] - a[j]) * float(s) for j in ARM}
            row["gripper"] = a["gripper"]
            out.append(row)
        return out

    def _grip(self, a: dict[str, float], value: float, dwell: bool = True) -> list[dict[str, float]]:
        delta = abs(value - a["gripper"])
        n = max(1, math.ceil(delta / self.cfg.gripper_speed * self.cfg.hz))
        if dwell:
            n += math.ceil(self.cfg.dwell_s * self.cfg.hz)
        out = []
        for i in range(1, n + 1):
            row = dict(a)
            ramp = min(1.0, i / max(1, math.ceil(delta / self.cfg.gripper_speed * self.cfg.hz)))
            row["gripper"] = a["gripper"] + (value - a["gripper"]) * ramp
            out.append(row)
        return out

    # ------------------------------------------------------------------ the whole task
    def plan(self, start: dict[str, float], pick: tuple[float, float], place: tuple[float, float]) -> Plan:
        cfg, home = self.cfg, dict(self.kin.spec.home)
        if math.dist(pick, place) < cfg.min_pick_place_dist:
            raise PlanError("The place point is too close to the pick point.")
        home["gripper"] = cfg.gripper_open
        zs_pick = {"hover": cfg.hover_height, "grasp": cfg.grasp_offset}
        zs_place = {"hover": cfg.hover_height, "grasp": cfg.release_offset}
        pitch_p, pk = self.solve_column(*pick, seed=home, zs=zs_pick)
        pitch_q, pl = self.solve_column(*place, seed=pk["hover"], zs=zs_place)

        plan = Plan(pitch_deg={"pick": pitch_p, "place": pitch_q})
        cur = {j: float(start[j]) for j in JOINTS}

        def add(name: str, rows: list[dict[str, float]]) -> None:
            nonlocal cur
            plan.segments.append(Segment(name, len(plan.rows), len(plan.rows) + len(rows)))
            plan.rows.extend(rows)
            cur = rows[-1]

        def with_grip(joints: dict[str, float], g: float) -> dict[str, float]:
            return {**{j: joints[j] for j in ARM}, "gripper": g}

        o, c = cfg.gripper_open, cfg.gripper_closed
        add("open", self._grip(cur, o, dwell=False))
        add("to_pick_hover", self._leg(cur, with_grip(pk["hover"], o)))
        add("descend_pick", self._leg(cur, with_grip(pk["grasp"], o)))
        add("close", self._grip(cur, c))
        add("lift_pick", self._leg(cur, with_grip(pk["hover"], c)))
        add("to_place_hover", self._leg(cur, with_grip(pl["hover"], c)))
        add("descend_place", self._leg(cur, with_grip(pl["grasp"], c)))
        add("release", self._grip(cur, o))
        add("lift_place", self._leg(cur, with_grip(pl["hover"], o)))
        add("home", self._leg(cur, with_grip(home, o)))
        plan.waypoints = {"pick_hover": pk["hover"], "pick_grasp": pk["grasp"],
                          "place_hover": pl["hover"], "place_grasp": pl["grasp"], "home": home}
        return plan

    def goto(self, start: dict[str, float], dest: tuple[float, float]) -> Plan:
        """Rise to travel height over the current tip position, glide to above `dest`, lower to it.

        The gripper keeps its value. The start must be above the table (not a lowered pose inside a block).
        """
        cfg = self.cfg
        cur = {j: float(start[j]) for j in JOINTS}
        tip = self.kin.tcp(cur)[:3, 3]
        zs = {"travel": cfg.travel_height, "final": cfg.goto_offset}
        x0, y0 = float(tip[0]), float(tip[1])
        up = None
        for pitch in cfg.pitches_deg:  # rise in place with the same pitch the destination will use
            up = self._solve(x0, y0, self.cal.z_table + zs["travel"], pitch, cur)
            if up is not None:
                break
        if up is None:
            raise PlanError("Cannot rise to travel height from the current pose; move the arm nearer the table.")
        pitch, col = self.solve_column(*dest, seed=up, zs=zs)
        plan = Plan(pitch_deg={"dest": pitch})
        g = cur["gripper"]
        for name, target in (("rise", up), ("glide", col["travel"]), ("lower", col["final"])):
            goal = {**{j: target[j] for j in ARM}, "gripper": g}
            rows = self._leg(cur, goal)
            plan.segments.append(Segment(name, len(plan.rows), len(plan.rows) + len(rows)))
            plan.rows.extend(rows)
            cur = rows[-1]
        plan.waypoints = {"rise": up, "travel": col["travel"], "final": col["final"]}
        return plan

    def go_home(self, start: dict[str, float]) -> Plan:
        home = {**self.kin.spec.home, "gripper": self.cfg.gripper_open}
        plan = Plan()
        cur = {j: float(start[j]) for j in JOINTS}
        rows = self._leg(cur, home)
        plan.segments.append(Segment("home", 0, len(rows)))
        plan.rows = rows
        return plan
