"""The drive service: what the page asks for ("go to this click", "next corner", "stop") and how it is carried out safely.

It holds the camera-rectangle -> arm calibration, the lag protections and the connection to whatever executes the move
(today the simulator; later the real arm, next to which this whole module is meant to run).
"""

from __future__ import annotations

import math

import threading
import time
from collections.abc import Callable

from lauzkach_eye_robot_control import servo
from lauzkach_eye_robot_control.drive import FACES, CommandGuard, DeadMan, SimClient, SimError, px_to_uv, uv_to_px
from lauzkach_eye_robot_control.robot_map import RobotMap

INSIDE_MARGIN = 0.03        # a click this far outside the rectangle (fraction of its side) still counts as on it
HEIGHTS_CM = (1.0, 3.0)     # "low" and "hover" tip heights above the table, for the simulator


class ApiError(RuntimeError):
    def __init__(self, status: int, message: str, code: str = "error") -> None:
        super().__init__(message)
        self.status, self.code = status, code


class DriveService:
    def __init__(self, matrix_source: Callable[[], list[list[float]] | None], robot_map: RobotMap | None, sim: SimClient,
                 faces: str = "up", deadman_s: float = 3.0, guard: CommandGuard | None = None,
                 closed_loop: tuple[servo.Mover, servo.Observer] | None = None, servo_config: servo.ServoConfig | None = None,
                 sim_calibration_error: tuple[float, float] = (0.0, 0.0), real=None, wrist_fetch=None) -> None:
        self._matrix_source, self.robot_map, self.sim = matrix_source, robot_map, sim
        self.runner = None
        self.wrist_fetch = wrist_fetch                           # () -> the gripper camera picture (BGR), or None
        self.watcher = None
        self.real = real                                          # the REAL arm (RealArmClient), or None: off unless the server was told to
        self.closed_loop, self.servo_config = closed_loop, servo_config or servo.ServoConfig()
        self.sim_error = sim_calibration_error                    # demo only: the simulated board differs from the arm's coordinates by this
        self._servo_thread: threading.Thread | None = None
        self._servo_state: dict = {"running": False, "last": None, "label": ""}
        self.guard = guard or CommandGuard()
        self.faces = faces
        self.log: list[str] = []
        self._lock = threading.Lock()
        self.deadman = DeadMan(deadman_s, self._sim_busy, self._hold_after_silence)

    def start(self) -> None:
        self.deadman.start()

    def stop_background(self) -> None:
        self.deadman.stop()

    def touch(self) -> None:
        """Any request from the page counts as 'still there'."""
        self.deadman.touch()

    # ------------------------------------------------------------------ helpers
    def _matrix(self) -> list[list[float]]:
        matrix = self._matrix_source()
        if matrix is None:
            raise ApiError(409, "The rectangle is not locked yet. Wait for the 5 second measurement to finish.", "not_locked")
        return matrix

    def _require_map(self) -> RobotMap:
        if self.robot_map is None:
            raise ApiError(503, "No robot calibration file was loaded (start the server with --robot-points).", "no_calibration")
        return self.robot_map

    def _sim_busy(self) -> bool:
        for client in (self.sim, self.real):
            if client is None:
                continue
            try:
                if client.state()["task"]["state"] in ("planning", "running"):
                    return True
            except (RuntimeError, KeyError, OSError):
                continue
        return False

    def _hold_after_silence(self) -> None:
        self.guard.stop()
        if self.runner is not None:
            self.runner.cancel()
        for client in (self.sim, self.real):
            if client is not None:
                try:
                    client.stop()
                except (RuntimeError, OSError):
                    pass
        self._note(f"The page went silent for {self.deadman.timeout_s:.0f} s during a move: the arm was held where it is.")

    def _note(self, text: str) -> None:
        with self._lock:
            self.log = (self.log + [f"{time.strftime('%H:%M:%S')} {text}"])[-8:]

    def _admit(self, env: dict) -> dict | None:
        """None = go ahead; a dict = a duplicate, return this instead of moving again."""
        admission = self.guard.admit(env.get("cid"), env.get("sent_ms"), env.get("gen"), env.get("max_age_ms"))
        if admission.status == "duplicate":
            return {**(admission.cached or {}), "duplicate": True}
        if not admission.ok:
            self._note(f"Refused a move: {admission.detail}")
            raise ApiError(409, admission.detail, admission.status)
        return None

    # ------------------------------------------------------------------ what the page can ask for
    def overview(self) -> dict:
        out: dict = {"server_ms": self.guard.now_ms(), "generation": self.guard.generation, "faces": self.faces, "faces_options": list(FACES),
                     "heights_cm": list(HEIGHTS_CM), "log": list(self.log), "robot_map": None, "corners": [], "sim": {"connected": False},
                     "servo": {"available": self.closed_loop is not None, **self._servo_state, "demo_error_mm": [round(self.sim_error[0] * 1000, 1), round(self.sim_error[1] * 1000, 1)]}}
        matrix = self._matrix_source()
        if self.robot_map is not None:
            out["robot_map"] = self.robot_map.summary()
            for corner in self.robot_map.corners():
                entry = {**corner, "x": round(corner["x"], 4), "y": round(corner["y"], 4), "px": None}
                if matrix is not None:
                    entry["px"] = [round(v, 1) for v in uv_to_px(matrix, corner["u"], corner["v"], self.faces)]
                out["corners"].append(entry)
        out["real"] = self._real_overview(matrix)
        try:
            state = self.sim.state()
            tip = state.get("tip_base")
            sim = {"connected": True, "task": state["task"], "tip_base": tip, "twin": state.get("twin")}
            if tip and self.robot_map is not None:
                u, v = self.robot_map.xy_to_uv(tip["x"] - self.sim_error[0], tip["y"] - self.sim_error[1])   # where the board sees the tip
                sim["tip_uv"] = [round(u, 3), round(v, 3)]
                if matrix is not None:
                    sim["tip_px"] = [round(c, 1) for c in uv_to_px(matrix, u, v, self.faces)]
            out["sim"] = sim
        except SimError as exc:
            out["sim"] = {"connected": False, "error": str(exc)}
        return out

    def _real_overview(self, matrix) -> dict:
        if self.real is None:
            return {"available": False}
        try:
            st = self.real.state()
        except (RuntimeError, OSError, KeyError) as exc:
            return {"available": True, "connected": False, "error": str(exc)}
        out = {"available": True, "sequence": None if self.runner is None else dict(self.runner.status), "vision": None if self.watcher is None else dict(self.watcher.status), "connected": st["connected"], "task": st["task"], "tip_base": st["tip_base"], "owner": st["owner"],
               "emergency_stop": st["emergency_stop"], "gripper": st["gripper"],
               "height_guard": bool(st.get("height_guard")), "grasp": st.get("grasp")}
        if self.robot_map is not None and matrix is not None:
            u, v = self.robot_map.xy_to_uv(st["tip_base"]["x"], st["tip_base"]["y"])
            out["tip_px"] = [round(c, 1) for c in uv_to_px(matrix, u, v, self.faces)]
        return out

    def _require_real(self, closed_loop: bool, confirm_real: bool) -> None:
        if self.real is None:
            raise ApiError(409, "The real arm is not enabled on this server (start it with --real-arm).", "no_real")
        if not confirm_real:
            raise ApiError(409, "Every move of the real arm needs an explicit confirmation.", "confirm")
        if closed_loop:
            raise ApiError(400, "Closed-loop correction is only available on the simulator.", "bad_request")

    def _move(self, env: dict, u: float, v: float, height_cm: float, label: str, real: bool = False) -> dict:
        robot_map = self._require_map()
        height = float(height_cm)
        if height not in HEIGHTS_CM:
            raise ApiError(400, f"height must be one of {HEIGHTS_CM} cm", "bad_request")
        x, y = robot_map.uv_to_xy(u, v)
        client = self.real if real else self.sim
        try:
            result = client.go_to_base(x, y, height)
        except SimError as exc:
            raise ApiError(exc.status if exc.status in (400, 409, 503) else 502, str(exc), "sim") from exc
        except RuntimeError as exc:                               # the real arm's refusals (a MoveError) carry a plain reason
            self._note(f"The real arm refused: {exc}")
            raise ApiError(409, str(exc), "real_refused") from exc
        answer = {"target": {"u": round(u, 3), "v": round(v, 3), "x": round(x, 4), "y": round(y, 4), "height_cm": height, "label": label},
                  "task": result, "real": real}
        self.guard.record(env.get("cid"), answer)
        self._note(f"Moving the {'REAL' if real else 'simulated'} tip over {label} (x {x * 100:.1f}, y {y * 100:.1f} cm from the arm base).")
        return answer

    def _move_closed(self, env: dict, u: float, v: float, height_cm: float, label: str) -> dict:
        """Move over a board point and refine it from the camera: the loop runs in the background, the page follows its progress."""
        robot_map = self._require_map()
        if self.closed_loop is None:
            raise ApiError(409, "Closed-loop correction is not available: no tip observer is configured.", "no_observer")
        if float(height_cm) not in HEIGHTS_CM:
            raise ApiError(400, f"height must be one of {HEIGHTS_CM} cm", "bad_request")
        with self._lock:
            if self._servo_thread is not None and self._servo_thread.is_alive():
                raise ApiError(409, "A closed-loop move is still running. Press Stop, or wait for it to finish.", "busy")
        target = robot_map.uv_to_xy(u, v)
        mover, observer = self.closed_loop
        generation = self.guard.generation

        def job() -> None:
            result = servo.run(target, float(height_cm), mover, observer, self.servo_config,
                               should_stop=lambda: self.guard.generation != generation)
            with self._lock:
                self._servo_state.update(running=False, last=result.summary())
            self._note(f"Closed loop for {label}: {'done' if result.ok else 'ended'} ({result.reason}).")

        with self._lock:
            self._servo_state.update(running=True, last=None, label=label)
            self._servo_thread = threading.Thread(target=job, daemon=True, name="servo")
            self._servo_thread.start()
        answer = {"target": {"u": round(u, 3), "v": round(v, 3), "x": round(target[0], 4), "y": round(target[1], 4), "height_cm": float(height_cm),
                             "label": label}, "closed_loop": True}
        self.guard.record(env.get("cid"), answer)
        self._note(f"Closed-loop move over {label}: moving, then checking with the camera.")
        return answer

    def go_click(self, env: dict, px: float, py: float, height_cm: float, closed_loop: bool = False, target: str = "sim",
                 confirm_real: bool = False) -> dict:
        duplicate = self._admit(env)
        if duplicate:
            return duplicate
        real = target == "real"
        if real:
            self._require_real(closed_loop, confirm_real)
            height_cm = 3.0                                        # the real arm only hovers, 3 cm above the board
            if not self.real.state().get("height_guard"):
                raise ApiError(409, "Clicking arbitrary points needs the newer arm agent (with the measured-height guard and sag "
                                    "compensation) on the server; the corner buttons still work.", "agent_old")
        matrix = self._matrix()
        u, v = px_to_uv(matrix, px, py, self.faces)
        if not (-INSIDE_MARGIN <= u <= 1 + INSIDE_MARGIN and -INSIDE_MARGIN <= v <= 1 + INSIDE_MARGIN):
            raise ApiError(400, "That click is outside the rectangle.", "outside")
        u, v = min(1.0, max(0.0, u)), min(1.0, max(0.0, v))
        label = f"the clicked point (u {u:.2f}, v {v:.2f})"
        return self._move_closed(env, u, v, height_cm, label) if closed_loop else self._move(env, u, v, height_cm, label, real)

    def grasp(self, env: dict, action: str, px: float | None = None, py: float | None = None, confirm_real: bool = False,
              box: list | None = None, park_after: bool = False, colour: str | None = None) -> dict:
        """One stage of the supervised real grasp (open, hover, lower, close, lift, putback). The agent enforces the order and the safety checks."""
        duplicate = self._admit(env)
        if duplicate:
            return duplicate
        self._require_real(False, confirm_real)
        if not hasattr(self.real, "grasp_action"):
            raise ApiError(409, "This arm client cannot grasp.", "no_grasp")
        x = y = yaw = None
        if action == "hover":
            if px is None or py is None:
                raise ApiError(400, "Click the block on the picture first.", "bad_request")
            u, v = px_to_uv(self._matrix(), px, py, self.faces)
            if not (-INSIDE_MARGIN <= u <= 1 + INSIDE_MARGIN and -INSIDE_MARGIN <= v <= 1 + INSIDE_MARGIN):
                raise ApiError(400, "That click is outside the rectangle.", "outside")
            x, y = self._require_map().uv_to_xy(min(1.0, max(0.0, u)), min(1.0, max(0.0, v)))
            if box and len(box) >= 2:                              # the detected block's outline: its rotation in the ARM's frame (mod 90 degrees)
                (bx0, by0), (bx1, by1) = box[0], box[1]
                u0, v0 = px_to_uv(self._matrix(), bx0, by0, self.faces)
                u1, v1 = px_to_uv(self._matrix(), bx1, by1, self.faces)
                (xa, ya), (xb, yb) = self._require_map().uv_to_xy(u0, v0), self._require_map().uv_to_xy(u1, v1)
                yaw = math.atan2(yb - ya, xb - xa)
        try:
            kwargs = {"park_after": True} if park_after else {}
            result = self.real.grasp_action(action, x, y, yaw, **kwargs) if yaw is not None else self.real.grasp_action(action, x, y, **kwargs)
        except RuntimeError as exc:
            self._note(f"The real arm refused the grasp step '{action}': {exc}")
            raise ApiError(409, str(exc), "real_refused") from exc
        if action == "lower" and colour and self.wrist_fetch is not None:
            from lauzkach_eye_robot_control.lower_watch import LowerWatcher
            real = self.real
            self.watcher = LowerWatcher(colour, self.wrist_fetch, lambda: real.state()["task"]["state"] in ("planning", "running"),
                                        lambda: real.grasp_action("lower_stop"), self._note).start()
            self._note(f"Watching the gripper camera for the {colour} block's edge to reach the bottom of the picture while lowering.")
        answer = {"action": action, "real": True, "task": result}
        self.guard.record(env.get("cid"), answer)
        self._note(f"Grasp step '{action}' started on the REAL arm" + (f" for the block at x {x * 100:.1f}, y {y * 100:.1f} cm." if x is not None else "."))
        return answer

    def _arm_xy(self, px: float, py: float) -> tuple[float, float]:
        u, v = px_to_uv(self._matrix(), px, py, self.faces)
        if not (-INSIDE_MARGIN <= u <= 1 + INSIDE_MARGIN and -INSIDE_MARGIN <= v <= 1 + INSIDE_MARGIN):
            raise ApiError(400, "That point is outside the rectangle.", "outside")
        return self._require_map().uv_to_xy(min(1.0, max(0.0, u)), min(1.0, max(0.0, v)))

    def pick_place(self, env: dict, pick: dict | None, dest: dict, others: list | None = None, confirm_real: bool = False, park_after: bool = True) -> dict:
        """Move the block at `pick` ({px, py, box, colour}) to `dest` ({px, py}); `others`: the other detected blocks' picture positions.
        If the arm is already holding a block (an earlier try stopped after the grip), `pick` is not needed: it carries on from there."""
        duplicate = self._admit(env)
        if duplicate:
            return duplicate
        self._require_real(False, confirm_real)
        if not hasattr(self.real, "grasp_action"):
            raise ApiError(409, "This arm client cannot grasp.", "no_grasp")
        if self.runner is not None and self.runner.busy():
            raise ApiError(409, "A pick and place is already running. Press Stop first.", "busy")
        try:
            stage = (self.real.state().get("grasp") or {}).get("stage", "idle")
        except (RuntimeError, OSError, KeyError):
            stage = "idle"
        first = {"closed": 4, "lifted": 5, "carried": 6}.get(stage, 0)                 # continue with the step after the last one done
        dx_, dy_ = self._arm_xy(float(dest["px"]), float(dest["py"]))
        bx = by = yaw = colour = None
        if first == 0:
            if not pick:
                raise ApiError(400, "Choose the starting block first.", "bad_request")
            bx, by = self._arm_xy(float(pick["px"]), float(pick["py"]))
            # Only before the pick-up: the destination must not be on another block. The picked block itself is not "another block" (the detector's
            # centre for it moves a little from frame to frame), and once a block is held no block-distance checks are made at all.
            for o in (others or []):
                ox, oy = self._arm_xy(float(o[0]), float(o[1]))
                if math.hypot(ox - bx, oy - by) < 0.035:
                    continue
                if math.hypot(dx_ - ox, dy_ - oy) < 0.055:
                    raise ApiError(400, "The destination is on top of another block: choose a free spot.", "bad_request")
            if math.hypot(dx_ - bx, dy_ - by) < 0.06:
                raise ApiError(400, "The destination is too close to the starting block (needs 6 cm).", "bad_request")
            box = pick.get("box")
            if box and len(box) >= 2:
                (u0, v0), (u1, v1) = px_to_uv(self._matrix(), box[0][0], box[0][1], self.faces), px_to_uv(self._matrix(), box[1][0], box[1][1], self.faces)
                (xa, ya), (xb, yb) = self._require_map().uv_to_xy(u0, v0), self._require_map().uv_to_xy(u1, v1)
                yaw = math.atan2(yb - ya, xb - xa)
            colour = pick.get("colour")
        elif pick:
            colour = pick.get("colour")
        real = self.real

        def step(action: str) -> dict:
            kw = {"hover": dict(x=bx, y=by, yaw=yaw), "carry": dict(x=dx_, y=dy_), "place": dict(park_after=park_after)}.get(action, {})
            try:
                result = real.grasp_action(action, **{k: v for k, v in kw.items() if v is not None})
            except RuntimeError as exc:
                self._note(f"The real arm refused '{action}': {exc}")
                raise
            if action == "lower" and colour and self.wrist_fetch is not None:
                from lauzkach_eye_robot_control.lower_watch import LowerWatcher
                self.watcher = LowerWatcher(colour, self.wrist_fetch, lambda: real.state()["task"]["state"] in ("planning", "running"),
                                            lambda: real.grasp_action("lower_stop"), self._note).start()
            return result

        from lauzkach_eye_robot_control.pickplace import PickPlaceRunner
        self.runner = PickPlaceRunner(step, real.state, self._note)
        label = (f"{colour or 'the'} block from x {bx * 100:.1f}, y {by * 100:.1f} cm to x {dx_ * 100:.1f}, y {dy_ * 100:.1f} cm" if first == 0
                 else f"the block the arm is holding to x {dx_ * 100:.1f}, y {dy_ * 100:.1f} cm")
        self.runner.start(label, first)
        self._note(f"Pick and place started on the REAL arm: {label}" + ("" if first == 0 else f" (carrying on from step {first + 1})") + ".")
        answer = {"sequence": dict(self.runner.status), "real": True}
        self.guard.record(env.get("cid"), answer)
        return answer

    def go_corner(self, env: dict, index: int, height_cm: float, closed_loop: bool = False, target: str = "sim",
                  confirm_real: bool = False) -> dict:
        duplicate = self._admit(env)
        if duplicate:
            return duplicate
        real = target == "real"
        if real:
            self._require_real(closed_loop, confirm_real)
        corners = self._require_map().corners()
        if not 0 <= index < len(corners):
            raise ApiError(400, "no such corner", "bad_request")
        c = corners[index]
        label = f"corner {index + 1} ({c['name']})"
        return self._move_closed(env, c["u"], c["v"], height_cm, label) if closed_loop else self._move(env, c["u"], c["v"], height_cm, label, real)

    def stop(self) -> dict:
        """A soft stop: halt the arm where it is and void every move still on its way. NOT the controller's emergency stop:
        nothing latches, and the next move can be sent right away."""
        generation = self.guard.stop()
        if self.runner is not None:
            self.runner.cancel()
        held = True
        for name, client in (("simulator", self.sim), ("real arm", self.real)):
            if client is None:
                continue
            try:
                client.stop()
            except (RuntimeError, OSError) as exc:
                held = False
                self._note(f"Stop requested, but the {name} did not confirm it ({exc}).")
        if held:
            self._note("Stop: motion halted, queued moves cancelled.")
        return {"generation": generation, "held": held}

    def set_faces(self, faces: str) -> dict:
        if faces not in FACES:
            raise ApiError(400, f"faces must be one of {FACES}", "bad_request")
        self.faces = faces
        self._note(f"The robot is now taken to face {faces} in the picture.")
        return {"faces": faces}
