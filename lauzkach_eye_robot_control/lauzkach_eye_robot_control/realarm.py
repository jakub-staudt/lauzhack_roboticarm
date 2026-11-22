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
    min_measured_height_m: float = 0.02   # the MEASURED tip must stay at least this far above the table during glide/hover


@dataclass
class GraspSettings:
    """The first real grasp. Numbers measured on the real gripper (a 49 mm block stalls the jaws at a reading of 49.4) and in the model
    (the fixed jaw's tip is 20.1 mm to one side of the tool point, along the jaw-closing axis)."""
    cube_m: float = 0.049
    open_value: float = 80.0              # fingertip gap 73 mm: 12 mm to spare each side of the block
    close_floor: float = 12.0             # the jaws closed on nothing read about 10.7
    close_step: float = 2.0               # gripper units per 0.2 s while closing (slow: a hard push on the block made the controller trip)
    hold_min: float = 30.0                # a stall above this = holding a block; below ~22 = closed on nothing
    squeeze: float = 1.5                  # command this far below the stall value to hold firmly
    contact_lag: float = 2.5              # the jaws lag the command by more than this and stop moving = they touched the block
    stall_s: float = 1.0
    stall_eps: float = 0.4
    fixed_tip_m: float = 0.0201
    clearance_m: float = 0.004            # gap between the fixed jaw and the block's face before closing
    hover_above_table_m: float = 0.062    # tool point height for the hover (the block is 49 mm tall)
    glide_above_table_m: float = 0.07
    lift_m: float = 0.06                  # how far the block is lifted (the first test, 3 cm, worked)
    grasp_tip_above_table_m: float = 0.0185   # tool-point height for the grasp: about 8 mm below the block's centre, so the jaws hold more of it
    min_pitch_deg: float = 65.0           # the gripper must point nearly straight down
    lower_min_height_m: float = 0.010     # during the lowering the measured tool point may go this far above the table (the lowest fingertip is ~3 mm below it)
    lower_max_sag_deg: float = 4.5        # a joint further than this behind its command = something resists: stop

    pick_offset_override: float | None = None   # aim this far behind the block's centre along the jaw axis (a profile's value); None: computed
    profile: str = "current"

    @property
    def pick_offset_m(self) -> float:
        if self.pick_offset_override is not None:
            return self.pick_offset_override
        return self.cube_m / 2 + self.clearance_m - self.fixed_tip_m

    @classmethod
    def from_profile(cls, path: str, name: str) -> "GraspSettings":
        import json as _json
        data = _json.loads(Path(path).read_text())
        if name not in data or name.startswith("_"):
            raise ValueError(f"no grasp profile '{name}' in {path}; choose from {[k for k in data if not k.startswith('_')]}")
        prof = {k: v for k, v in data[name].items() if k in cls.__dataclass_fields__}
        return cls(profile=name, **prof)

    @property
    def grasp_tip_m(self) -> float:
        return self.grasp_tip_above_table_m


class PacedMover(Mover):
    """The existing mover, plus a rest after each waypoint and a hook that runs when the move ends (to release control)."""

    pause_s: float = 0.35
    on_finish = None
    min_height_m: float | None = None     # the MEASURED tip must stay above this (the arm sags under gravity at full stretch)
    trim: bool = True                     # after the last step, nudge the target up/over by the measured error (sag compensation)
    trim_tol_m: float = 0.004
    _armed: bool = False

    def run_job(self, token: str, fn, label: str) -> None:
        """Run `fn()` (a sequence of gripper and arm steps) in the mover's thread, with the same status, cancel and release rules."""
        self._cancel.clear()
        self._set(state="moving", step=label, index=0, total=0, message="")

        def body() -> None:
            try:
                fn()
                self._set(state="done", step="done", message=self.status().get("message") or "Done.")
            except MoveError as exc:
                self._set(state="error", message=str(exc))
            finally:
                if self.on_finish is not None:
                    self.on_finish()

        self._thread = threading.Thread(target=body, daemon=True)
        self._thread.start()

    def beat(self, token: str) -> None:
        self.http("POST", "/api/control/heartbeat", {"token": token})

    def run_waypoints(self, token: str, waypoints: list[Waypoint], phase: str = "", trim_last: bool = False) -> None:
        """Execute waypoints in order (blocking), with the height guard and sag handling of a normal move. `trim_last`: after the last
        waypoint, look at the MEASURED tip and push the command until the tip really is there (gravity sag at full reach)."""
        self._armed = False
        with ThreadPoolExecutor(max_workers=len(ARM)) as pool:
            for i, wp in enumerate(waypoints):
                self._check()
                self._set(step=phase or wp.name, index=i, total=len(waypoints))
                list(pool.map(lambda j: self.http("POST", "/api/target", {"token": token, "joint": j, "value": wp.joints[j]}), ARM))
                self._wait_reached(token, wp.joints)
                if not (trim_last and i == len(waypoints) - 1):             # the last step is checked after the sag trim, not before it
                    self._guard_height()
                end = time.monotonic() + self.pause_s
                while time.monotonic() < end:
                    self._check()
                    time.sleep(0.05)
        if trim_last and waypoints:
            self._trim(token, waypoints[-1], tries=4, tol=0.004, limit=0.05)
            self._guard_height(final=True)

    def _run(self, token: str, waypoints: list[Waypoint]) -> None:
        try:
            self._armed = False
            with ThreadPoolExecutor(max_workers=len(ARM)) as pool:
                for i, wp in enumerate(waypoints):
                    self._check()
                    self._set(step=wp.name, index=i)
                    list(pool.map(lambda j: self.http("POST", "/api/target", {"token": token, "joint": j, "value": wp.joints[j]}), ARM))
                    self._wait_reached(token, wp.joints)
                    if i < len(waypoints) - 1:
                        self._guard_height()
                    end = time.monotonic() + self.pause_s
                    while time.monotonic() < end:
                        self._check()
                        time.sleep(0.05)
                if self.trim and waypoints and waypoints[-1].name == "hover":
                    self._trim(token, waypoints[-1])
                self._guard_height(final=True)
            self._set(state="done", step="done", message="Done.")
        except MoveError as exc:
            self._set(state="error", message=str(exc))
        finally:
            if self.on_finish is not None:
                self.on_finish()

    def _measured(self) -> dict[str, float]:
        state = self.http("GET", "/api/state", None)
        return {k: float(v) for k, v in state["measured"].items()}

    def _guard_height(self, final: bool = False) -> None:
        """Stop if the MEASURED tip is below the height limit. Armed only after the tip has first been seen above it (the arm may
        START resting on the board), and the final step is checked after the sag trim, not before it."""
        if self.min_height_m is None:
            return
        z = float(self.kin.tcp(self._measured())[2, 3])
        if z >= self.min_height_m:
            self._armed = True
            return
        if self._armed or final:
            try:
                self.http("POST", "/api/stop", {})                 # soft stop: hold where it is
            except MoveError:
                pass
            raise MoveError(f"The tip is only {z * 100:.1f} cm above the origin plane, below the {self.min_height_m * 100:.1f} cm limit "
                            "(the arm sags at full stretch). Stopped; the arm holds its position.")

    def _trim(self, token: str, last: Waypoint, tries: int = 2, tol: float | None = None, limit: float = 0.03) -> None:
        """The arm sags under gravity: every joint rests a little away from its command (measured minus command), so the measured tip
        sits lower than the target. Solve the pose for the TARGET and subtract that per-joint offset from the command, up to twice."""
        target = np.array(last.xyz, float)
        tol = self.trim_tol_m if tol is None else tol
        for _ in range(tries):
            self._check()
            state = self.http("GET", "/api/state", None)
            measured = {k: float(v) for k, v in state["measured"].items()}
            command = {k: float(v) for k, v in (state.get("command") or state["measured"]).items()}
            tip = np.array(self.kin.tcp(measured)[:3, 3], float)
            err = float(np.linalg.norm(target - tip))
            if err <= tol or err > limit:
                return
            rot = np.array(self.kin.tcp(command)[:3, :3])             # the orientation that was commanded
            sol = self.kin.ik(target, rot, seed=command, tol=self.settings.ik_tol_m)
            if not sol.ok or sol.rot_err > self.settings.max_rot_err_rad:
                return
            offset = {j: measured[j] - command[j] for j in ARM}
            joints = {j: float(sol.joints[j]) - offset[j] for j in ARM}
            if any(not self.kin.spec.limits[j][0] <= joints[j] <= self.kin.spec.limits[j][1] for j in ARM):
                return
            if max(abs(joints[j] - command[j]) for j in ARM) > 8.0:
                return
            self._set(step="trim")
            for j in ARM:
                self.http("POST", "/api/target", {"token": token, "joint": j, "value": joints[j]})
            self._wait_reached(token, joints)


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
                    z_floor: float | None, hover_m: float, max_step_deg: float = 3.0, orient_from: dict[str, float] | None = None) -> list[Waypoint]:
    """Same route as `plan_hover` (rise, glide at the glide height, straight down) but with THREE inverse-kinematics solves instead
    of one per centimetre: IK only at the corners of the route, joint-space interpolation between them, and a cheap forward-kinematics
    check that the tip never dips below the allowed height on the way. Raises MoveError if any check fails (the caller then falls
    back to the slow, line-by-line planner)."""
    z_above, z_hover = z_table + settings.above_m, z_table + hover_m
    tip0 = np.array(kin.tcp(start)[:3, 3], float)
    pose_o = kin.tcp(orient_from or start)                            # the gripper orientation to keep (maybe a re-tilted copy of the pose)
    R0 = np.array(pose_o[:3, :3])
    th0 = math.atan2(pose_o[1, 3], pose_o[0, 3])
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
                 label: str = "Camera drive", grasp_settings: "GraspSettings | None" = None) -> None:
        self.http, self.kin, self.z_table = http, kin, z_table
        self.settings = settings or RealSettings()
        s = self.settings
        self.box = (min(p[0] for p in calibration_xy) - s.margin_m, max(p[0] for p in calibration_xy) + s.margin_m,
                    min(p[1] for p in calibration_xy) - s.margin_m, max(p[1] for p in calibration_xy) + s.margin_m)
        self.label = label
        click = ClickSettings(above_m=s.glide_m, step_m=s.step_m, gripper_max=s.gripper_max)
        self.mover = PacedMover(http, kin, ImagePlane(Path(tempfile.gettempdir()) / "lauzkach_unused_plane.json"), click)
        self.mover.pause_s = s.pause_s
        self.mover.min_height_m = z_table + s.min_measured_height_m
        self.mover.on_finish = self._release
        self._token: str | None = None
        self._lock = threading.Lock()
        self.gs = grasp_settings or GraspSettings()
        self.grasp: dict = {"stage": "idle", "pending": None}

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
        return {"task": {"state": name, "message": status.get("message", ""), "step": status.get("step"),
                         "index": status.get("index"), "total": status.get("total")},
                "tip_base": {"x": float(tip[0]), "y": float(tip[1]), "z": float(tip[2])},
                "connected": bool(raw.get("connected")), "torque": bool(raw.get("torque")),
                "emergency_stop": bool((raw.get("emergency_stop") or {}).get("latched")),
                "owner": (raw.get("control") or {}).get("label"), "gripper": joints.get("gripper"),
                "height_guard": self.mover.min_height_m is not None and self.mover.trim, "grasp": self._grasp_state(raw)}

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
            self.grasp = {"stage": "idle", "pending": None}             # a plain move that really started ends the grasp procedure (a refused one does not)
        except Exception:
            self._release()
            raise
        return {"state": "planning", "waypoints": len(waypoints)}

    def _preflight(self, state: dict, allow_open_gripper: bool = False) -> tuple[dict[str, float], float | None]:
        if (state.get("emergency_stop") or {}).get("latched"):
            raise MoveError("Emergency stop is latched.")
        if not state.get("connected") or not state.get("torque"):
            raise MoveError("The arm is disconnected or torque is off.")
        if (state.get("cartesian") or {}).get("active"):
            raise MoveError("Release the movement key first.")
        if not allow_open_gripper and float(state["measured"].get("gripper", 100.0)) > self.settings.gripper_max:
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

    # ------------------------------------------------------------------ the staged grasp (each stage is one supervised button press)
    STAGES = ("open", "hover", "lower", "close", "lift", "putback")

    def _reconcile(self, raw: dict | None) -> None:
        """If the jaws are visibly holding a block (stalled between the hold and open values, the command ahead of the reading) while the
        stage says otherwise, e.g. the controller dropped control during the close, trust what the arm shows."""
        if not raw or self.mover.busy() or self.grasp["stage"] not in ("idle", "open", "hover", "lower"):
            return
        try:
            m = float(raw["measured"]["gripper"])
            c = float((raw.get("command") or raw["measured"]).get("gripper", m))
        except (KeyError, TypeError, ValueError):
            return
        gs = self.gs
        if gs.hold_min <= m <= gs.open_value - 12 and m - c >= 1.0:
            self.grasp.update(stage="closed", hold=m, last=f"The jaws are holding a block (they read {m:.1f}).")

    def _grasp_state(self, raw: dict | None = None) -> dict:
        self._settle_stage()
        self._reconcile(raw)
        g = self.grasp
        nxt = {"idle": ["open"], "open": ["hover", "open"], "hover": ["lower", "open"], "lower": ["close", "open"],
               "closed": ["lift", "open"], "lifted": ["putback"], "miss": ["open"]}.get(g["stage"], ["open"])
        return {"stage": g["stage"], "profile": self.gs.profile, "next": nxt, "block": g.get("block"), "hold_value": g.get("hold"), "last": g.get("last", "")}

    def _settle_stage(self) -> None:
        """A stage counts only when its job finished successfully; an error leaves the previous stage in place."""
        pending = self.grasp.get("pending")
        if not pending or self.mover.busy():
            return
        status = self.mover.status()
        if status.get("state") == "done":
            self.grasp.update(stage=pending[0], **pending[1])
            self.grasp["last"] = status.get("message", "")
        elif status.get("state") in ("error", "stopped"):
            self.grasp["last"] = status.get("message", "")
        self.grasp["pending"] = None

    def grasp_action(self, action: str, x: float | None = None, y: float | None = None) -> dict:
        self._settle_stage()
        if self.mover.busy():
            raise MoveError("The arm is still moving. Wait for it to finish.")
        state = self.http("GET", "/api/state", None)
        g = self._grasp_state(state)
        stage = g["stage"]
        if action not in g["next"]:
            raise MoveError(f"Not now: after '{stage}' the next steps are {', '.join(g['next'])}.")
        measured, floor = self._preflight(state, allow_open_gripper=True)
        owner = (state.get("control") or {}).get("owner_id")
        if owner is not None:
            raise MoveError(f"Someone else holds control ({(state.get('control') or {}).get('label')}).")
        gs = self.gs
        if action == "open":
            return self._launch_job("open gripper", lambda t: self._gripper_to(t, gs.open_value), ("open", {}))
        if action == "hover":
            if x is None or y is None:
                raise MoveError("Click the block on the picture first.")
            if not (self.box[0] <= x <= self.box[1] and self.box[2] <= y <= self.box[3]):
                raise MoveError("That point is outside the area the arm was calibrated on.")
            return self._grasp_hover(state, measured, floor, float(x), float(y))
        if action == "lower":
            return self._grasp_lower(state, measured, floor)
        if action == "close":
            return self._launch_job("close gripper", self._close_gripper, None)
        if action == "lift":
            return self._grasp_lift(measured, floor)
        return self._grasp_putback(measured, floor)

    def _launch_job(self, label: str, fn, pending) -> dict:
        token = self._claim()
        try:
            self.http("POST", "/api/control/heartbeat", {"token": token})
            self.grasp["pending"] = pending
            self.mover.run_job(token, lambda: fn(token), label)
        except Exception:
            self.grasp["pending"] = None
            self._release()
            raise
        return {"state": "planning", "stage": label}

    # ---- gripper
    def _gripper_state(self) -> tuple[float, float]:
        st = self.http("GET", "/api/state", None)
        m = float(st["measured"]["gripper"])
        return m, float((st.get("command") or st["measured"]).get("gripper", m))

    def _gripper_to(self, token: str, value: float) -> None:
        measured, command = self._gripper_state()
        target = command
        deadline = time.monotonic() + 12.0
        while abs(target - value) > 0.01:
            self.mover._check()
            target += max(-self.gs.close_step * 2, min(self.gs.close_step * 2, value - target))
            self.http("POST", "/api/target", {"token": token, "joint": "gripper", "value": target})
            self.mover.beat(token)
            time.sleep(0.2)
        while time.monotonic() < deadline:
            self.mover._check()
            self.mover.beat(token)
            measured, _ = self._gripper_state()
            if abs(measured - value) <= 2.0:
                return
            time.sleep(0.2)
        raise MoveError(f"The gripper did not reach {value:.0f} (it reads {measured:.0f}).")

    def _close_gripper(self, token: str) -> None:
        """Close slowly; the moment the jaws touch (they lag the command and stop moving) stop pushing: command just below the reading."""
        gs = self.gs
        measured, command = self._gripper_state()
        target, last, still_since = command, measured, None
        while True:
            self.mover._check()
            target = max(gs.close_floor, target - gs.close_step)
            self.http("POST", "/api/target", {"token": token, "joint": "gripper", "value": target})
            self.mover.beat(token)
            time.sleep(0.2)
            measured, _ = self._gripper_state()
            moving = last - measured > gs.stall_eps
            still_since = None if moving else (still_since or time.monotonic())
            last = measured
            touching = measured - target > gs.contact_lag and still_since is not None and time.monotonic() - still_since >= 0.4
            if touching or target <= gs.close_floor:
                break
        for _ in range(3):                                                    # let it settle, then read the stall value
            time.sleep(0.2)
            self.mover.beat(token)
            measured, _ = self._gripper_state()
        if measured >= gs.hold_min:
            self.http("POST", "/api/target", {"token": token, "joint": "gripper", "value": measured - gs.squeeze})
            self.grasp["pending"] = ("closed", {"hold": measured})
            self.mover._set(message=f"Holding a block: the jaws stalled at {measured:.1f} (block about 49 mm).")
            return
        self._gripper_to(token, gs.open_value)                                # closed on nothing: open again
        self.grasp.update(stage="miss", pending=None)
        raise MoveError(f"Missed: the jaws closed to {measured:.0f} on nothing. The gripper is open again; hover and lower again.")

    # ---- arm
    def _tool_aim(self, measured: dict[str, float], x: float, y: float) -> tuple[float, float, float]:
        """Where to put the tool point so the block sits just clear of the fixed jaw: PICK_OFFSET behind the block's centre along the
        jaw-closing axis (measured from the gripper's real orientation, turned with the shoulder to the new azimuth). -> (x, y, pitch deg)."""
        approach, closing = self.kin.tool_axes(measured)
        tip = np.array(self.kin.tcp(measured)[:3, 3], float)
        th0 = math.atan2(tip[1], tip[0])
        pitch = math.degrees(math.asin(max(-1.0, min(1.0, -float(approach[2])))))
        ax, ay = x, y
        for _ in range(2):
            a = math.atan2(ay, ax) - th0
            c, sn = math.cos(a), math.sin(a)
            cx, cy = c * closing[0] - sn * closing[1], sn * closing[0] + c * closing[1]
            norm = math.hypot(cx, cy)
            if norm < 0.5:
                raise MoveError("The jaws do not close sideways (horizontally); rotate the wrist before grasping.")
            ax, ay = x - self.gs.pick_offset_m * cx / norm, y - self.gs.pick_offset_m * cy / norm
        return ax, ay, pitch

    def _grasp_hover(self, state: dict, measured: dict[str, float], floor, x: float, y: float) -> dict:
        gs = self.gs
        if measured["gripper"] < gs.open_value - 12:
            raise MoveError("Open the gripper first.")
        _, _, pitch0 = self._tool_aim(measured, x, y)
        limits = (state.get("cartesian") or {}).get("joint_limits", {}).get("effective") or {}
        click = ClickSettings(above_m=gs.glide_above_table_m, step_m=self.settings.step_m, gripper_max=1e9)
        wf = measured.get("wrist_flex", 0.0)
        wanted = [p for p in (pitch0,) if gs.min_pitch_deg <= p <= 78.0] + [74.0, 70.0, 78.0, 66.0]   # keep a good tilt, else choose one
        last_error: MoveError | None = None
        for pitch in wanted:
            tilted = {**measured, "wrist_flex": wf + (pitch - pitch0)}          # same jaw direction, only the tilt changes
            ax, ay, _ = self._tool_aim(tilted, x, y)
            try:
                try:
                    wps = plan_hover_fast(self.kin, click, measured, (ax, ay), self.z_table, floor, gs.hover_above_table_m, self.settings.max_step_deg, orient_from=tilted)
                except MoveError:
                    wps = plan_hover(self.kin, click, tilted, (ax, ay), self.z_table, floor, gs.hover_above_table_m, self.settings.max_step_deg)
                self._check_limits(wps, limits)
                break
            except MoveError as exc:
                last_error = exc
        else:
            raise MoveError(f"The arm cannot hover over that block with the gripper pointing about 66 to 78 degrees below horizontal "
                            f"({last_error}). Try a block nearer the base.")
        if abs(pitch - pitch0) > 1.0:
            self.mover._set(message=f"Tilting the gripper from {pitch0:.0f} to {pitch:.0f} degrees below horizontal while hovering.")
        return self._launch_job("hover over the block", lambda t: self.mover.run_waypoints(t, wps, "hover over the block"),
                                ("hover", {"block": (x, y), "aim": (ax, ay)}))

    def _vertical_waypoints(self, measured: dict[str, float], dz: float, floor) -> list[Waypoint]:
        return _vertical(self.kin, ClickSettings(gripper_max=1e9), measured, dz, floor, 0.01)

    def _grasp_lower(self, state: dict, measured: dict[str, float], floor) -> dict:
        gs = self.gs
        tip_z = float(self.kin.tcp(measured)[2, 3])
        dz = (self.z_table + gs.grasp_tip_m) - tip_z
        if dz > -0.004:
            raise MoveError("The gripper is already at grasp height.")
        wps = self._vertical_waypoints(measured, dz, floor)
        self._check_limits(wps, (state.get("cartesian") or {}).get("joint_limits", {}).get("effective"))

        def job(token: str) -> None:
            saved = (self.mover.min_height_m, self.mover.settings.max_sag_deg, self.mover.pause_s)
            self.mover.min_height_m, self.mover.settings.max_sag_deg, self.mover.pause_s = self.z_table + gs.lower_min_height_m, gs.lower_max_sag_deg, 0.4
            try:
                self.mover.run_waypoints(token, wps, "lowering to grasp height", trim_last=True)
            finally:
                self.mover.min_height_m, self.mover.settings.max_sag_deg, self.mover.pause_s = saved
        return self._launch_job("lower to grasp height", job, ("lower", {}))

    def _grasp_lift(self, measured: dict[str, float], floor) -> dict:
        gs = self.gs
        if measured["gripper"] < gs.hold_min - 4:
            raise MoveError("The gripper is not holding a block (it is nearly closed).")
        wps = self._vertical_waypoints(measured, gs.lift_m, floor)

        def job(token: str) -> None:
            z0 = float(self.kin.tcp(measured)[2, 3])
            self.mover.run_waypoints(token, wps, "lifting the block", trim_last=True)
            now = self.http("GET", "/api/state", None)
            rose = float(self.kin.tcp({k: float(v) for k, v in now["measured"].items()})[2, 3]) - z0
            if rose < 0.6 * gs.lift_m:
                raise MoveError(f"The arm did not lift: the tip rose only {rose * 100:.1f} cm of {gs.lift_m * 100:.0f} cm (the arm sags at this reach). "
                                "The block is still gripped on the table; try Lift again or Put back and release.")
            held, _ = self._gripper_state()
            if held < gs.hold_min - 6:
                self.grasp.update(stage="miss", pending=None)
                raise MoveError(f"The block was lost during the lift (the jaws read {held:.0f}).")
        return self._launch_job("lift the block", job, ("lifted", {}))

    def _grasp_putback(self, measured: dict[str, float], floor) -> dict:
        gs = self.gs
        down = self._vertical_waypoints(measured, -gs.lift_m, floor)

        def job(token: str) -> None:
            saved = (self.mover.min_height_m, self.mover.settings.max_sag_deg, self.mover.pause_s)
            self.mover.min_height_m, self.mover.settings.max_sag_deg, self.mover.pause_s = self.z_table + gs.lower_min_height_m, gs.lower_max_sag_deg, 0.4
            try:
                self.mover.run_waypoints(token, down, "putting the block back", trim_last=True)
            finally:
                self.mover.min_height_m, self.mover.settings.max_sag_deg, self.mover.pause_s = saved
            self._gripper_to(token, gs.open_value)
            now = self.http("GET", "/api/state", None)
            joints = {k: float(v) for k, v in now["measured"].items()}
            self.mover.run_waypoints(token, self._vertical_waypoints(joints, gs.lift_m + 0.02, floor), "rising clear")
        return self._launch_job("put the block back", job, ("hover", {}))

    def stop(self) -> dict:
        """A soft stop: the arm holds where it is. Nothing latches."""
        self.mover.cancel()
        try:
            self.http("POST", "/api/stop", {})
        except MoveError:
            pass
        self._release()
        return {"state": "stopped"}
