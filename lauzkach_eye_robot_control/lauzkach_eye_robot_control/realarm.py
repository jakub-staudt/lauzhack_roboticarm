"""The REAL arm behind the drive panel. Deliberately stricter than the simulator:

* HOVER ONLY: the tip never goes lower than 3 cm above the board (no touching, no grasping);
* it rises first, then glides at 5 cm (clears 2-3 cm blocks), then descends straight down at the target, never diagonally;
* small steps with a rest after each one, so it is slow (about 1.5 cm/s), and it never changes the controller's own speed setting;
* control is claimed just for the move and released afterwards, so the manual page keeps working between moves;
* a move is refused if any step would leave the controller's real (EEPROM) joint limits, if the gripper is open, if the
  emergency stop is latched, or if someone else holds control;
* the existing mover's own checks stay in force: it abandons the move if the emergency stop latches, control is lost, a joint is
  blocked, or the arm does not settle; a Stop (soft stop) holds the arm where it is.

It runs in the `robotic_arm` environment (it needs the arm's kinematics) and only talks to the RAMION dashboard's HTTP API.
Nothing here is run by the tests against a real arm.
"""

from __future__ import annotations

import tempfile
import threading
import time
from concurrent.futures import ThreadPoolExecutor
from dataclasses import dataclass
from pathlib import Path

from robotic_arm.clickmove import ClickSettings, ImagePlane, MoveError, Mover, Waypoint, plan_click, plan_nudge
import math

import numpy as np

from robotic_arm.planner import ARM, tool_rotation


@dataclass
class RealSettings:
    glide_m: float = 0.05            # tip height while gliding across the board
    min_hover_m: float = 0.03        # never lower than this above the table
    step_m: float = 0.01             # waypoint spacing on the glide: small steps = slow and smooth
    pause_s: float = 0.35            # rest after every waypoint: this, with the move itself, sets the speed
    margin_m: float = 0.03           # a target may lie this far outside the box around the calibrated points
    limit_margin_deg: float = 3.0    # every step must stay this far inside the controller's effective joint limits
    max_step_deg: float = 3.0        # no waypoint moves any joint by more than this: with pause_s that makes about 6 deg/s
    gripper_max: float = 25.0


class PacedMover(Mover):
    """The existing mover, plus a rest after each waypoint and a hook that runs when the move ends (to release control)."""

    pause_s: float = 0.35
    on_finish = None

    def _run(self, token: str, waypoints: list[Waypoint]) -> None:
        try:
            with ThreadPoolExecutor(max_workers=len(ARM)) as pool:
                for i, wp in enumerate(waypoints):
                    self._check()
                    self._set(step=wp.name, index=i)
                    list(pool.map(lambda j: self.http("POST", "/api/target", {"token": token, "joint": j, "value": wp.joints[j]}), ARM))
                    self._wait_reached(token, wp.joints)
                    end = time.monotonic() + self.pause_s
                    while time.monotonic() < end:
                        self._check()
                        time.sleep(0.05)
            self._set(state="done", step="done", message="Done.")
        except MoveError as exc:
            self._set(state="error", message=str(exc))
        finally:
            if self.on_finish is not None:
                self.on_finish()


def subdivide(start: dict[str, float], waypoints: list[Waypoint], max_step_deg: float) -> list[Waypoint]:
    """Insert joint-space steps so that no joint moves more than `max_step_deg` between neighbouring waypoints."""
    out: list[Waypoint] = []
    previous = {j: float(start[j]) for j in ARM}
    for wp in waypoints:
        n = max(1, int(-(-max(abs(wp.joints[j] - previous[j]) for j in ARM) // max_step_deg)))
        for i in range(1, n + 1):
            t = i / n
            out.append(Waypoint(wp.name, wp.xyz, {j: previous[j] + (wp.joints[j] - previous[j]) * t for j in ARM}))
        previous = {j: wp.joints[j] for j in ARM}
    return out


def _vertical(kin, settings: ClickSettings, start: dict[str, float], dz_total: float, z_floor: float | None,
              step: float = 0.01) -> list[Waypoint]:
    """Straight up or down by `dz_total` in steps of at most `step`, keeping the gripper's tilt and roll."""
    out: list[Waypoint] = []
    joints, remaining = dict(start), dz_total
    while abs(remaining) > 0.001:
        dz = max(-step, min(step, remaining))
        waypoint = plan_nudge(kin, settings, joints, dz, z_floor)[0]
        out.append(waypoint)
        joints = {**joints, **waypoint.joints}
        remaining -= dz
    return out


def _joint_glide(kin, settings: ClickSettings, joints: dict[str, float], target_xy: tuple[float, float], z_table: float,
                 z_floor: float | None, min_height_m: float) -> list[Waypoint]:
    """Fallback for paths where a straight Cartesian line makes the joints jump (the arm has to change configuration, for instance
    from the far corner to the near one): solve the pose over the target once, then move in joint space, and check by sampling that
    the tip stays above `min_height_m` the whole way."""
    permissive = ClickSettings(above_m=settings.above_m, step_m=1.0, max_joint_step_deg=180.0, gripper_max=settings.gripper_max)
    goal = [w for w in plan_click(kin, permissive, joints, target_xy, z_table, z_floor) if w.name == "glide"][-1]   # the pose over the target
    n = 240
    heights = []
    for i in range(1, n + 1):
        t = i / n
        mid = {**joints, **{j: joints[j] + (goal.joints[j] - joints[j]) * t for j in ARM}}
        heights.append(float(kin.tcp(mid)[2, 3]))
    if min(heights) < min_height_m:
        raise MoveError(f"Moving between those two spots would dip the tip to {min(heights) * 100:.1f} cm (limit {min_height_m * 100:.1f} cm); "
                        "refused. Go through a corner in between.")
    return [goal]


def _interpolate(kin, a: dict[str, float], b: dict[str, float], n: int) -> tuple[list[dict[str, float]], float]:
    """n joint-space points from a (exclusive) to b (inclusive), and the lowest tip height along them (cheap forward kinematics)."""
    pts = [{j: a[j] + (b[j] - a[j]) * i / n for j in ARM} for i in range(1, n + 1)]
    lowest = min(float(kin.tcp({**a, **p})[2, 3]) for p in pts)
    return pts, lowest


def plan_hover_fast(kin, settings: ClickSettings, start: dict[str, float], target_xy: tuple[float, float], z_table: float,
                    z_floor: float | None, hover_m: float, max_step_deg: float = 3.0) -> list[Waypoint]:
    """Same route as `plan_hover` (rise, glide at the glide height, straight down) but with THREE inverse-kinematics solves instead
    of one per centimetre: IK only at the corners of the route, joint-space interpolation between them, and a cheap forward-kinematics
    check that the tip never dips below the allowed height on the way. Raises MoveError if any check fails (the caller then falls
    back to the slow, line-by-line planner)."""
    z_above, z_hover = z_table + settings.above_m, z_table + hover_m
    pose0 = kin.tcp(start)
    tip0, R0 = np.array(pose0[:3, 3], float), np.array(pose0[:3, :3])
    th0 = math.atan2(tip0[1], tip0[0])
    tx, ty = target_xy
    low_ok = max(z_hover - 0.005, (z_floor + 0.001) if z_floor is not None else -1.0)          # nothing below this, ever

    def keep(p):
        a = math.atan2(p[1], p[0]) - th0
        c, sn = math.cos(a), math.sin(a)
        return np.array([[c, -sn, 0], [sn, c, 0], [0, 0, 1]]) @ R0

    orientations = [keep] + [lambda p, pp=pp: tool_rotation(math.atan2(p[1], p[0]), pp) for pp in settings.pitches_deg]
    reason = "no orientation worked"
    for rotation in orientations:
        stops: list[tuple[str, np.ndarray]] = []
        if tip0[2] < z_above - 0.002:
            stops.append(("rise", np.array([tip0[0], tip0[1], z_above])))
        stops += [("glide", np.array([tx, ty, z_above])), ("hover", np.array([tx, ty, z_hover]))]
        waypoints: list[Waypoint] = []
        seed, here, ok = dict(start), dict(start), True
        for name, p in stops:
            sol = kin.ik(p, rotation(p) if name != "rise" else R0, seed=seed, tol=settings.ik_tol_m)
            if not sol.ok or sol.rot_err > settings.max_rot_err_rad:
                ok, reason = False, f"cannot reach the {name} point ({p[0]:.3f}, {p[1]:.3f}, {p[2]:.3f})"
                break
            goal = {j: float(sol.joints[j]) for j in ARM}
            if any(not kin.spec.limits[j][0] <= goal[j] <= kin.spec.limits[j][1] for j in ARM):
                ok, reason = False, f"the {name} point is outside a joint limit"
                break
            n = max(1, math.ceil(max(abs(goal[j] - here[j]) for j in ARM) / max_step_deg))
            pts, lowest = _interpolate(kin, here, goal, n)
            ctrl_floor = (z_floor + 0.001) if z_floor is not None else -1.0
            if name == "rise":                                      # the arm may START low (resting on the board); it just must not dip further
                floor_here = max(min(float(kin.tcp(here)[2, 3]), p[2]) - 0.01, ctrl_floor)
            elif name == "hover":
                floor_here = low_ok
            else:                                                   # glide: stay near the glide height
                floor_here = max(min(float(kin.tcp(here)[2, 3]), p[2]) - 0.01, low_ok)
            if lowest < floor_here:
                ok, reason = False, f"the {name} leg would dip the tip to {lowest * 100:.1f} cm (limit {floor_here * 100:.1f} cm)"
                break
            waypoints += [Waypoint(name, [float(c) for c in p], pt) for pt in pts]
            here, seed = goal, {**seed, **sol.joints}
        if ok:
            return waypoints
    raise MoveError(f"Fast planner: {reason}.")


def plan_hover(kin, settings: ClickSettings, start: dict[str, float], target_xy: tuple[float, float], z_table: float,
               z_floor: float | None, hover_m: float, max_step_deg: float = 3.0) -> list[Waypoint]:
    """Rise in 1 cm steps to the glide height, glide over the target at `settings.above_m`, then straight down to `hover_m` above
    the table (also in 1 cm steps). Raises MoveError if any step is not possible."""
    z_above = z_table + settings.above_m
    waypoints: list[Waypoint] = []
    joints = dict(start)
    z_now = float(kin.tcp(joints)[2, 3])
    if z_now < z_above - 0.002:                                     # below the glide height: first straight up
        waypoints += _vertical(kin, settings, joints, z_above - z_now, z_floor)
        joints = {**joints, **waypoints[-1].joints}
    try:
        path = [w for w in plan_click(kin, settings, joints, target_xy, z_table, z_floor) if w.name not in ("down", "up")]
    except MoveError:                                               # a straight line needs a configuration change: glide in joint space
        path = _joint_glide(kin, settings, joints, target_xy, z_table, z_floor, z_above - 0.015)
    waypoints += path
    joints = {**joints, **waypoints[-1].joints}
    descent = z_above - (z_table + hover_m)
    if descent > 0.002:
        waypoints += _vertical(kin, settings, joints, -descent, z_floor)
    return subdivide(start, waypoints, max_step_deg)


class RealArmClient:
    """Same small interface as the simulator client (`state`, `go_to_base`, `stop`), so the drive service can use either."""

    def __init__(self, http, kin, calibration_xy: list[tuple[float, float]], z_table: float, settings: RealSettings | None = None,
                 label: str = "Camera drive") -> None:
        self.http, self.kin, self.z_table = http, kin, z_table
        self.settings = settings or RealSettings()
        s = self.settings
        self.box = (min(p[0] for p in calibration_xy) - s.margin_m, max(p[0] for p in calibration_xy) + s.margin_m,
                    min(p[1] for p in calibration_xy) - s.margin_m, max(p[1] for p in calibration_xy) + s.margin_m)
        self.label = label
        click = ClickSettings(above_m=s.glide_m, step_m=s.step_m, gripper_max=s.gripper_max)
        self.mover = PacedMover(http, kin, ImagePlane(Path(tempfile.gettempdir()) / "lauzkach_unused_plane.json"), click)
        self.mover.pause_s = s.pause_s
        self.mover.on_finish = self._release
        self._token: str | None = None
        self._lock = threading.Lock()

    # ------------------------------------------------------------------ the lease, held only for the length of a move
    def _claim(self) -> str:
        state = self.http("GET", "/api/state", None)
        owner = (state.get("control") or {}).get("owner_id")
        if owner is not None:
            raise MoveError(f"Someone else holds control ({(state.get('control') or {}).get('label')}). Release it in the manual page first.")
        generation = (state.get("emergency_stop") or {}).get("generation", 0)
        reply = self.http("POST", "/api/control/claim", {"label": self.label, "emergency_stop_generation": generation})
        token = reply.get("token")
        if not token:
            raise MoveError("The controller did not grant control.")
        with self._lock:
            self._token = token
        return token

    def _release(self) -> None:
        with self._lock:
            token, self._token = self._token, None
        if token:
            try:
                self.http("POST", "/api/control/release", {"token": token})
            except MoveError:
                pass                                               # the lease also expires by itself after 1.5 s

    # ------------------------------------------------------------------ the interface the drive service uses
    def state(self) -> dict:
        raw = self.http("GET", "/api/state", None)
        joints = {k: float(v) for k, v in raw["measured"].items()}
        tip = self.kin.tcp(joints)[:3, 3]
        status = self.mover.status()
        name = {"moving": "running", "done": "done", "idle": "idle"}.get(status.get("state"), "error")
        if name == "error" and "ancel" in str(status.get("message", "")):
            name = "stopped"
        return {"task": {"state": name, "message": status.get("message", "")},
                "tip_base": {"x": float(tip[0]), "y": float(tip[1]), "z": float(tip[2])},
                "connected": bool(raw.get("connected")), "torque": bool(raw.get("torque")),
                "emergency_stop": bool((raw.get("emergency_stop") or {}).get("latched")),
                "owner": (raw.get("control") or {}).get("label"), "gripper": joints.get("gripper")}

    def go_to_base(self, x: float, y: float, height_cm: float) -> dict:
        s = self.settings
        hover = height_cm / 100.0
        if hover < s.min_hover_m - 1e-9:
            raise MoveError(f"The real arm stays at least {s.min_hover_m * 100:.0f} cm above the board.")
        if not (self.box[0] <= x <= self.box[1] and self.box[2] <= y <= self.box[3]):
            raise MoveError("That point is outside the area the arm was calibrated on.")
        if self.mover.busy():
            raise MoveError("The arm is still moving. Wait for it to finish.")
        # Plan FIRST, with no lease held: planning takes seconds, and a lease lasts only 1.5 s without a heartbeat.
        state = self.http("GET", "/api/state", None)
        if (state.get("control") or {}).get("owner_id") is not None:
            raise MoveError(f"Someone else holds control ({(state.get('control') or {}).get('label')}). Release it in the manual page first.")
        measured, floor = self._preflight(state)
        limits = (state.get("cartesian") or {}).get("joint_limits", {}).get("effective")
        try:
            waypoints = plan_hover_fast(self.kin, self.mover.settings, measured, (x, y), self.z_table, floor, hover, s.max_step_deg)
        except MoveError:                                           # a check failed: use the slow planner that goes line by line
            waypoints = plan_hover(self.kin, self.mover.settings, measured, (x, y), self.z_table, floor, hover, s.max_step_deg)
        self._check_limits(waypoints, limits)
        token = self._claim()                                       # only now, and the first heartbeat goes out at once
        try:
            self.http("POST", "/api/control/heartbeat", {"token": token})
            self.mover._launch(token, waypoints, {"x": x, "y": y, "hover_cm": height_cm})
        except Exception:
            self._release()
            raise
        return {"state": "planning", "waypoints": len(waypoints)}

    def _preflight(self, state: dict) -> tuple[dict[str, float], float | None]:
        if (state.get("emergency_stop") or {}).get("latched"):
            raise MoveError("Emergency stop is latched.")
        if not state.get("connected") or not state.get("torque"):
            raise MoveError("The arm is disconnected or torque is off.")
        if (state.get("cartesian") or {}).get("active"):
            raise MoveError("Release the movement key first.")
        if float(state["measured"].get("gripper", 100.0)) > self.settings.gripper_max:
            raise MoveError("Close the gripper first (hold E): the real arm moves only with the gripper closed.")
        floor = ((state.get("cartesian") or {}).get("workspace") or {}).get("min_tcp_z_m")
        return {k: float(v) for k, v in state["measured"].items()}, floor

    def _check_limits(self, waypoints: list[Waypoint], limits: dict | None) -> None:
        if not limits:
            raise MoveError("The controller did not report its joint limits, so the move cannot be checked.")
        margin = self.settings.limit_margin_deg
        for wp in waypoints:
            for joint in ARM:
                lo, hi = limits[joint]
                if not lo + margin <= wp.joints[joint] <= hi - margin:
                    raise MoveError(f"The path needs {joint} at {wp.joints[joint]:.0f} deg, too close to its limit ({lo:.0f} to {hi:.0f}). "
                                    "Move that joint away from its stop in the manual page first (for the wrist roll, use the roll nudge).")

    def stop(self) -> dict:
        """A soft stop: the arm holds where it is. Nothing latches."""
        self.mover.cancel()
        try:
            self.http("POST", "/api/stop", {})
        except MoveError:
            pass
        self._release()
        return {"state": "stopped"}
