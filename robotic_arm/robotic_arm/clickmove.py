"""Click-to-move MVP: a click on the overhead image -> hover 25 mm above that table point, lower, back up.

* `ImagePlane` learns image -> table from pairs recorded by the user: the gripper tip is placed on a visible spot, the spot is
  clicked in the image, and the tip's 3D position (forward kinematics of the measured joints, arm base frame) is stored.
  Four or more pairs give a homography (normalised image x,y) -> table (x,y); five or more give a residual check.
* `plan_click` turns a target into joint waypoints with inverse kinematics (pure, testable, nothing is sent).
* `Mover` sends the waypoints as plain joint targets (`POST /api/target`, the same command the dashboard sliders use) and waits for
  each to be reached. The controller ramps toward each target itself, so link latency only slows the motion down; heartbeats keep
  the control lease alive. The controller's tip-floor check does not apply to joint targets, so the floor is enforced here.
"""

from __future__ import annotations

import http.client
import json
import math
import threading
import time
from collections.abc import Callable
from concurrent.futures import ThreadPoolExecutor
from dataclasses import dataclass, field
from pathlib import Path
from urllib.error import HTTPError, URLError
from urllib.parse import urlsplit
from urllib.request import Request, urlopen

import numpy as np

from .calibration import apply_homography, fit_homography
from .planner import ARM, tool_rotation


class MoveError(RuntimeError):
    """A click that cannot be carried out; the message is shown to the user."""


@dataclass
class ClickSettings:
    above_m: float = 0.025  # hover height above the table
    down_offset_m: float = 0.003  # lowest tip height above the calibrated table height
    step_m: float = 0.025  # spacing of intermediate waypoints on the glide
    ramp_tol_deg: float = 0.6  # the controller's own command has reached the waypoint within this
    max_sag_deg: float = 6.0  # a settled joint further than this from its command is blocked (gravity sag of a few degrees is normal)
    wp_timeout_s: float = 12.0
    region_margin_m: float = 0.03  # a click may be this far outside the calibrated spots
    max_joint_step_deg: float = 35.0  # larger jumps between waypoints mean the IK changed branch: refuse
    pitches_deg: tuple[float, ...] = (90.0, 75.0, 60.0, 45.0, 35.0)
    ik_tol_m: float = 0.003
    max_rot_err_rad: float = 0.6
    gripper_max: float = 25.0  # moves run with the gripper closed (0..100 scale); it is never commanded here


# --------------------------------------------------------------------------- image -> table
class ImagePlane:
    """Pairs (normalised image x, y) <-> tip position, persisted as JSON."""

    def __init__(self, path: str | Path):
        self.path = Path(path)
        self.pairs: list[dict] = []
        if self.path.exists():
            self.pairs = json.loads(self.path.read_text())

    def _save(self) -> None:
        self.path.write_text(json.dumps(self.pairs, indent=2))

    def add(self, px: float, py: float, xyz: list[float], joints: dict[str, float]) -> None:
        self.pairs.append({"px": float(px), "py": float(py), "xyz": [float(c) for c in xyz],
                           "joints": {k: float(v) for k, v in joints.items()}, "t": time.time()})
        self._save()

    def undo(self) -> None:
        if self.pairs:
            self.pairs.pop()
            self._save()

    def clear(self) -> None:
        self.pairs = []
        self._save()

    @property
    def ready(self) -> bool:
        return len(self.pairs) >= 4

    def _fit(self) -> np.ndarray:
        if not self.ready:
            raise MoveError(f"Calibrate first: {len(self.pairs)} of at least 4 spots recorded.")
        src = np.array([[p["px"], p["py"]] for p in self.pairs])
        dst = np.array([p["xyz"][:2] for p in self.pairs])
        return fit_homography(src, dst)

    def xy(self, px: float, py: float) -> tuple[float, float]:
        x, y = apply_homography(self._fit(), np.array([[px, py]]))[0]
        return float(x), float(y)

    @property
    def z_table(self) -> float:
        return float(np.mean([p["xyz"][2] for p in self.pairs]))

    def region_contains(self, x: float, y: float, margin: float) -> bool:
        pts = np.array([p["xyz"][:2] for p in self.pairs])
        lo, hi = pts.min(axis=0) - margin, pts.max(axis=0) + margin
        return bool(lo[0] <= x <= hi[0] and lo[1] <= y <= hi[1])

    def summary(self) -> dict:
        residuals: list[float | None] = [None] * len(self.pairs)
        rms = worst = None
        if self.ready:
            H = self._fit()
            src = np.array([[p["px"], p["py"]] for p in self.pairs])
            dst = np.array([p["xyz"][:2] for p in self.pairs])
            err = np.linalg.norm(apply_homography(H, src) - dst, axis=1) * 1000
            residuals = [float(e) for e in err]
            if len(self.pairs) >= 5:  # with exactly 4 spots the fit is exact and says nothing
                rms, worst = float(np.sqrt((err**2).mean())), float(err.max())
        return {"count": len(self.pairs), "ready": self.ready, "checked": len(self.pairs) >= 5,
                "z_table": self.z_table if self.pairs else None, "rms_mm": rms, "max_mm": worst,
                "pairs": [{"px": p["px"], "py": p["py"], "xyz": p["xyz"], "residual_mm": r}
                          for p, r in zip(self.pairs, residuals, strict=True)]}


# --------------------------------------------------------------------------- planning (pure)
@dataclass
class Waypoint:
    name: str
    xyz: list[float]
    joints: dict[str, float]


def _pitch_of(kin, joints: dict[str, float]) -> float:
    approach, _ = kin.tool_axes(joints)
    return float(np.degrees(np.arcsin(np.clip(-approach[2], -1, 1))))


def plan_click(kin, s: ClickSettings, start: dict[str, float], target_xy: tuple[float, float],
               z_table: float, z_floor: float | None) -> list[Waypoint]:
    """Waypoints: (rise) -> glide at >= hover height -> above target -> down -> up. Raises MoveError."""
    z_above = z_table + s.above_m
    z_down = z_table + s.down_offset_m
    if z_floor is not None:
        z_down = max(z_down, z_floor + 0.001)  # joint targets bypass the controller's tip floor, so keep it here
    z_down = min(z_down, z_above)
    tip0 = kin.tcp(start)[:3, 3]
    tx, ty = target_xy
    path: list[tuple[str, np.ndarray]] = []
    cur = np.array(tip0, float)
    if cur[2] < z_above - 0.002:
        cur = np.array([cur[0], cur[1], z_above])
        path.append(("rise", cur.copy()))
    goal = np.array([tx, ty, z_above])
    n = max(1, math.ceil(float(np.linalg.norm(goal - cur)) / s.step_m))
    path += [("glide", cur + (goal - cur) * i / n) for i in range(1, n + 1)]
    path += [("down", np.array([tx, ty, z_down])), ("up", goal.copy())]

    # First choice: keep the gripper's current tilt AND roll, turning only with the shoulder as the tip moves around the base.
    # Fallbacks: fixed tool pitches (these may swing the wrist roll, so the joint-jump check below still guards them).
    R0 = np.array(kin.tcp(start)[:3, :3])
    th0 = math.atan2(tip0[1], tip0[0])

    def keep(p: np.ndarray) -> np.ndarray:
        a = math.atan2(p[1], p[0]) - th0
        c, sn = math.cos(a), math.sin(a)
        return np.array([[c, -sn, 0], [sn, c, 0], [0, 0, 1]]) @ R0

    candidates = [("current orientation", keep)] + [
        (f"pitch {pp:.0f} deg", lambda p, pp=pp: tool_rotation(math.atan2(p[1], p[0]), pp)) for pp in s.pitches_deg]
    reason = "no orientation worked"
    for label, rotation in candidates:
        waypoints: list[Waypoint] = []
        seed, ok = dict(start), True
        for name, p in path:
            sol = kin.ik(p, rotation(p), seed=seed, tol=s.ik_tol_m)
            if not sol.ok or sol.rot_err > s.max_rot_err_rad:
                ok, reason = False, f"cannot reach {name} point ({p[0]:.3f}, {p[1]:.3f}, {p[2]:.3f}) with {label}"
                break
            if any(not kin.spec.limits[j][0] <= sol.joints[j] <= kin.spec.limits[j][1] for j in ARM):
                ok, reason = False, f"{name} point is outside a joint limit with {label}"
                break
            if max(abs(sol.joints[j] - seed[j]) for j in ARM) > s.max_joint_step_deg:
                ok, reason = False, f"{name} point needs a large joint jump with {label}"
                break
            waypoints.append(Waypoint(name, [float(c) for c in p], {j: float(sol.joints[j]) for j in ARM}))
            seed = {**seed, **sol.joints}
        if ok:
            return waypoints
    raise MoveError(f"Target not reachable: {reason}.")


def plan_nudge(kin, s: ClickSettings, start: dict[str, float], dz: float, z_floor: float | None) -> list[Waypoint]:
    """One waypoint: the tip moves vertically by dz [m], keeping its x, y and the gripper's tilt and roll."""
    pose = kin.tcp(start)
    tip, R0 = np.array(pose[:3, 3], float), np.array(pose[:3, :3])
    z_new = float(tip[2] + dz)
    if z_floor is not None and dz < 0 and z_new < z_floor + 0.001:
        raise MoveError(f"Too low: the controller's tip floor is {z_floor * 1000:.1f} mm and the tip is at {tip[2] * 1000:.1f} mm.")
    goal = np.array([tip[0], tip[1], z_new])
    sol = kin.ik(goal, R0, seed=start, tol=s.ik_tol_m)
    if not sol.ok or sol.rot_err > s.max_rot_err_rad:
        raise MoveError("The arm cannot move the tip there while keeping its tilt (near the edge of its reach?).")
    if any(not kin.spec.limits[j][0] <= sol.joints[j] <= kin.spec.limits[j][1] for j in ARM):
        raise MoveError("That would exceed a joint limit.")
    if max(abs(sol.joints[j] - start[j]) for j in ARM) > s.max_joint_step_deg:
        raise MoveError("That would need a large joint jump; refused.")
    return [Waypoint("up" if dz > 0 else "down", [float(c) for c in goal], {j: float(sol.joints[j]) for j in ARM})]


# --------------------------------------------------------------------------- execution
Http = Callable[[str, str, dict | None], dict]


def urllib_http(base_url: str, timeout_s: float = 3.0) -> Http:
    def call(method: str, path: str, body: dict | None = None) -> dict:
        request = Request(base_url.rstrip("/") + path, method=method,
                          data=None if body is None else json.dumps(body).encode(),
                          headers={"Content-Type": "application/json"} if body is not None else {})
        try:
            with urlopen(request, timeout=timeout_s) as response:
                return json.loads(response.read() or b"{}")
        except HTTPError as exc:
            try:
                message = json.loads(exc.read()).get("error", str(exc.code))
            except ValueError:
                message = str(exc.code)
            raise MoveError(f"Controller refused {path}: {message}") from None
        except (URLError, TimeoutError, OSError) as exc:
            raise MoveError(f"Controller unavailable ({type(exc).__name__}).") from None
    return call


def pooled_http(base_url: str, timeout_s: float = 4.0) -> Http:
    """Like `urllib_http`, but reuses one connection per thread (HTTP keep-alive): over a relayed link each new connection costs
    extra round trips, which is most of the latency of a small request."""
    parts = urlsplit(base_url)
    local = threading.local()

    def call(method: str, path: str, body: dict | None = None) -> dict:
        payload = None if body is None else json.dumps(body).encode()
        headers = {"Content-Type": "application/json"} if payload is not None else {}
        for attempt in (0, 1):  # a kept-alive connection may have been closed by the server: retry once on a fresh one
            conn = getattr(local, "conn", None)
            if conn is None:
                conn = local.conn = http.client.HTTPConnection(parts.hostname, parts.port or 80, timeout=timeout_s)
            try:
                conn.request(method, path, body=payload, headers=headers)
                response = conn.getresponse()
                data = response.read()
                break
            except (http.client.HTTPException, OSError) as exc:
                conn.close(); local.conn = None
                if attempt == 1:
                    raise MoveError(f"Controller unavailable ({type(exc).__name__}).") from None
        if response.status >= 400:
            try:
                message = json.loads(data).get("error", str(response.status))
            except ValueError:
                message = str(response.status)
            raise MoveError(f"Controller refused {path}: {message}")
        return json.loads(data or b"{}")
    return call


@dataclass
class Mover:
    http: Http
    kin: object
    plane: ImagePlane
    settings: ClickSettings = field(default_factory=ClickSettings)

    def __post_init__(self) -> None:
        self._lock = threading.Lock()
        self._cancel = threading.Event()
        self._thread: threading.Thread | None = None
        self._status: dict = {"state": "idle", "message": ""}

    # ---- status
    def status(self) -> dict:
        with self._lock:
            return dict(self._status)

    def _set(self, **kw) -> None:
        with self._lock:
            self._status = {**self._status, **kw}

    def busy(self) -> bool:
        return self._thread is not None and self._thread.is_alive()

    # ---- recording calibration spots (read-only)
    def record_spot(self, px: float, py: float) -> dict:
        state = self.http("GET", "/api/state", None)
        joints = {k: float(v) for k, v in state["measured"].items()}
        xyz = self.kin.tcp(joints)[:3, 3]
        self.plane.add(px, py, list(xyz), joints)
        return self.plane.summary()

    # ---- moving
    def _preflight(self, *, need_closed_gripper: bool) -> tuple[dict[str, float], float | None]:
        if self.busy():
            raise MoveError("The arm is still moving. Wait for it to finish.")
        state = self.http("GET", "/api/state", None)
        if state.get("emergency_stop", {}).get("latched"):
            raise MoveError("Emergency stop is latched.")
        if not state.get("connected") or not state.get("torque"):
            raise MoveError("The arm is disconnected or torque is off.")
        if state.get("control", {}).get("owner_id") is None:
            raise MoveError("Take control first.")
        if state.get("cartesian", {}).get("active"):
            raise MoveError("Release the movement key first.")
        if need_closed_gripper and float(state["measured"].get("gripper", 100.0)) > self.settings.gripper_max:
            raise MoveError("Close the gripper first (hold E). Click-moves run with the gripper closed.")
        floor = (state.get("cartesian", {}).get("workspace") or {}).get("min_tcp_z_m")
        return {k: float(v) for k, v in state["measured"].items()}, floor

    def _launch(self, token: str, waypoints: list[Waypoint], target: dict) -> dict:
        self._cancel.clear()
        self._set(state="moving", step="starting", index=0, total=len(waypoints), message="", target=target)
        self._thread = threading.Thread(target=self._run, args=(token, waypoints), daemon=True)
        self._thread.start()
        return self.status()

    def start_click(self, token: str, px: float, py: float) -> dict:
        if self.busy():
            raise MoveError("The arm is still moving. Wait for it to finish.")
        if not (math.isfinite(px) and math.isfinite(py)) or not (0 <= px <= 1 and 0 <= py <= 1):
            raise MoveError("Click is outside the camera image.")
        x, y = self.plane.xy(px, py)
        if not self.plane.region_contains(x, y, self.settings.region_margin_m):
            raise MoveError("That point is outside the calibrated area. Add calibration spots nearer to it first.")
        measured, floor = self._preflight(need_closed_gripper=True)
        waypoints = plan_click(self.kin, self.settings, measured, (x, y), self.plane.z_table, floor)
        return self._launch(token, waypoints, {"x": x, "y": y, "z_above": waypoints[-1].xyz[2],
                                               "z_down": min(w.xyz[2] for w in waypoints)})

    def start_nudge(self, token: str, dz: float) -> dict:
        """Move the tip straight up (dz > 0) or down by a small step, whatever the gripper is doing."""
        if not math.isfinite(dz) or not 0 < abs(dz) <= 0.03:
            raise MoveError("The step must be between 1 and 30 mm.")
        measured, floor = self._preflight(need_closed_gripper=False)
        waypoints = plan_nudge(self.kin, self.settings, measured, dz, floor)
        return self._launch(token, waypoints, {"dz": dz})

    def cancel(self) -> None:
        self._cancel.set()
        if self.busy():
            try:  # hold the arm where it is (this also releases control)
                self.http("POST", "/api/stop", {})
            except MoveError:
                pass

    def _run(self, token: str, waypoints: list[Waypoint]) -> None:
        try:
            with ThreadPoolExecutor(max_workers=len(ARM)) as pool:
                for i, wp in enumerate(waypoints):
                    self._check()
                    self._set(step=wp.name, index=i)
                    list(pool.map(lambda j: self.http("POST", "/api/target", {"token": token, "joint": j, "value": wp.joints[j]}), ARM))
                    self._wait_reached(token, wp.joints)
            self._set(state="done", step="done", message="Done. Click the next point.")
        except MoveError as exc:
            self._set(state="error", message=str(exc))

    def _check(self) -> None:
        if self._cancel.is_set():
            raise MoveError("Cancelled.")

    def _wait_reached(self, token: str, goal: dict[str, float]) -> None:
        """Done when the controller's ramp has reached the waypoint and the joints stopped moving.

        Measured joints rest a few degrees away from their command under load (gravity at full stretch), so comparing the
        measurement with the goal would time out for a loaded joint that is working fine.
        """
        s = self.settings
        deadline = time.monotonic() + s.wp_timeout_s
        last_beat, previous, stable = 0.0, None, 0
        while True:
            self._check()
            if time.monotonic() - last_beat > 0.25:
                self.http("POST", "/api/control/heartbeat", {"token": token})
                last_beat = time.monotonic()
            state = self.http("GET", "/api/state", None)
            if state.get("emergency_stop", {}).get("latched"):
                raise MoveError("Emergency stop latched; movement abandoned.")
            if state.get("control", {}).get("owner_id") is None:
                raise MoveError("Control was lost; movement abandoned (the arm holds its position).")
            m = state["measured"]
            c = state.get("command") or m
            if all(abs(c[j] - goal[j]) <= s.ramp_tol_deg for j in ARM):
                moving = previous is not None and max(abs(m[j] - previous[j]) for j in ARM) > 0.3
                stable = 0 if moving else stable + 1
                if stable >= 3:
                    worst = max(ARM, key=lambda j: abs(m[j] - c[j]))
                    if abs(m[worst] - c[worst]) > s.max_sag_deg:
                        raise MoveError(f"{worst} is blocked: it rests {abs(m[worst] - c[worst]):.1f} deg from where it is "
                                        "commanded (is the tip pressing on the table?). Movement stopped.")
                    return
            previous = {j: m[j] for j in ARM}
            if time.monotonic() > deadline:
                far = max(ARM, key=lambda j: abs(c[j] - goal[j]))
                raise MoveError(f"The arm did not settle in time ({far} is {abs(c[far] - goal[far]):.1f} deg from its target); movement stopped.")
            time.sleep(0.08)
