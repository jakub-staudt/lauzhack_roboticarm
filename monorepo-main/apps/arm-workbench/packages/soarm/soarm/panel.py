"""Hardware control and HTTP API for the SO-101.

`soarm-panel` connects the arm through the `feetech` backend, runs a 20 Hz control loop
and serves the controller API. The dashboard owns the operator interface.
"""

from __future__ import annotations

import argparse
import json
import logging
import math
import os
import secrets
import tempfile
import threading
import time
import webbrowser
from collections.abc import Callable, Iterable
from dataclasses import dataclass
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from importlib import util
from pathlib import Path
from typing import Any
from urllib.parse import urlsplit

from machinekind_twin_api.backend import BackendContext

from . import ROBOT_DIR
from .feetech import ADDR_TORQUE_ENABLE, FeetechArm
from .panel_cameras import CameraManager, parse_camera_source
from .panel_cartesian import (
    DIRECTIONS,
    OBSERVATION_MAX_AGE_S,
    ORIENTATION_DIRECTIONS,
    PROFILES,
    TILT_DIRECTIONS,
    TILT_FLOOR_NOISE_M,
    TILT_FLOOR_PROGRESS_TIMEOUT_S,
    TRANSLATION_DIRECTIONS,
    WATCHDOG_S,
    CartesianSolver,
    Workspace,
)
from .panel_cartesian_ranges import CartesianRangeEstimator
from .panel_cartesian_recovery import (
    INGRESS_INNER_MARGIN_DEG,
    MAX_PITCH_CHANGE_DEG,
    MAX_SOFT_VIOLATION_DEG,
    MAX_TIP_DRIFT_M,
    ingress_plan,
)
from .panel_viewer import PoseViewer
from .serial_bridge import split_listen

logger = logging.getLogger(__name__)

TICK_HZ = 20.0
#: Default panel speed [deg/s for the arm joints, units/s for the gripper].
DEFAULT_SPEED = 30.0
#: A stalled tick (a slow bridge, a busy machine) must not turn into one big step at servo speed.
MAX_DT_S = 0.1
#: Only an explicit owner heartbeat renews the control lease.
HEARTBEAT_TIMEOUT_S = 1.5
# Model sessions have independent server-side watchdogs; owner heartbeats alone never sustain motion.
MODEL_MAX_DURATION_S = 60.0
MODEL_MAX_SPEED = 15.0
MODEL_ACTION_TIMEOUT_S = 0.5
MODEL_OBSERVATION_MAX_AGE_S = 3.0
MODEL_CAMERA_MAX_AGE_S = 1.0
MODEL_MAX_TRACKING_DEG = 15.0


class PanelError(Exception):
    """A command the panel refuses, with the HTTP status it answers."""

    def __init__(self, message: str, status: HTTPStatus = HTTPStatus.BAD_REQUEST) -> None:
        super().__init__(message)
        self.status = status


@dataclass
class EmergencyStopState:
    """Process-local latch with optional persistence shared by the station services."""

    path: Path | None = None
    latched: bool = False
    generation: int = 0
    hardware_session: bool = False
    hold_status: str = "not_requested"
    hold_error: str | None = None

    @classmethod
    def load(cls, path: Path | None) -> EmergencyStopState:
        if path is None:
            return cls()
        try:
            data = json.loads(path.read_text(encoding="utf-8"))
            if (not isinstance(data, dict) or type(data.get("latched")) is not bool
                    or type(data.get("generation")) is not int or data["generation"] < 0):
                raise ValueError("invalid emergency stop state")
            hardware_session = data.get("hardware_session", False)
            if type(hardware_session) is not bool:
                raise ValueError("invalid hardware session marker")
            latched = data["latched"] or hardware_session
            generation = data["generation"] + int(hardware_session and not data["latched"])
            return cls(path, latched, generation, False,
                       "unconfirmed" if latched else "not_requested",
                       "No current hold result is available after restart." if latched else None)
        except FileNotFoundError:
            return cls(path)
        except (OSError, ValueError):
            logger.exception("Could not read emergency stop state. Starting latched.")
            return cls(path, True, secrets.randbits(52), False, "unconfirmed",
                       "Emergency stop storage could not be read.")

    def snapshot(self) -> dict[str, Any]:
        return {"latched": self.latched, "generation": self.generation,
                "hold_status": self.hold_status, "hold_error": self.hold_error}

    def persist(self, *, latched: bool) -> None:
        if self.path is None:
            return
        self.path.parent.mkdir(parents=True, exist_ok=True)
        temp = None
        try:
            with tempfile.NamedTemporaryFile(mode="w", encoding="utf-8", dir=self.path.parent,
                                             prefix=f".{self.path.name}.", delete=False) as handle:
                temp = Path(handle.name)
                json.dump({"latched": latched, "generation": self.generation,
                           "hardware_session": self.hardware_session}, handle)
                handle.flush()
                os.fsync(handle.fileno())
            os.replace(temp, self.path)
            if os.name == "posix":
                directory = os.open(self.path.parent, os.O_RDONLY)
                try:
                    os.fsync(directory)
                finally:
                    os.close(directory)
        finally:
            if temp is not None:
                temp.unlink(missing_ok=True)


@dataclass(frozen=True)
class RobotDescription:
    """The part of `robot.yaml` the panel uses."""

    name: str
    joints: tuple[str, ...]
    gripper: str | None
    limits: dict[str, tuple[float, float]]
    max_vel: dict[str, float]
    home: dict[str, float]
    robot_dir: Path = ROBOT_DIR

    @classmethod
    def load(cls, folder: Path) -> RobotDescription:
        try:
            import yaml
        except ImportError as exc:  # pragma: no cover - depends on the environment
            raise RuntimeError('soarm-panel needs pyyaml:  pip install "soarm[panel]"') from exc

        data = yaml.safe_load((Path(folder) / "robot.yaml").read_text(encoding="utf-8"))
        return cls(
            name=data["name"],
            joints=tuple(data["joints"]),
            gripper=data.get("gripper"),
            limits={name: tuple(pair) for name, pair in data["limits"].items()},
            max_vel=dict(data["max_vel"]),
            home=dict(data.get("home") or {}),
            robot_dir=Path(folder),
        )

    def context(self, port: str, settings: dict[str, Any]) -> BackendContext:
        """The `BackendContext` for the `feetech` backend; `settings` is its `feetech:` section."""
        return BackendContext(robot=self.name, joints=self.joints, gripper=self.gripper, limits=self.limits,
                              max_vel=self.max_vel, port=port, settings=settings)


class PanelController:
    """The control loop and the operator's commands on a connected `FeetechArm`.

    `target` is where the operator wants each joint, `command` what went to the servos last. Each tick
    reads the joints, moves `command` towards `target` by at most `speed * dt` per joint and sends it.
    One lock guards the state and the bus, held for a whole tick and for each command.
    """

    def __init__(self, arm: FeetechArm, description: RobotDescription, *,
                 stop_state: EmergencyStopState | None = None) -> None:
        self._emergency_stop = stop_state if stop_state is not None else EmergencyStopState()
        self.arm = arm
        self.description = description
        self._lock = threading.RLock()
        self._halt = threading.Event()
        self._thread = threading.Thread(target=self._run, name="panel-loop", daemon=True)

        eeprom = arm.joint_limits()
        self.software_limits = dict(description.limits)
        self.hardware_limits = dict(eeprom)
        self.limits: dict[str, tuple[float, float]] = {}
        for joint in description.joints:
            low, high = description.limits[joint]
            if joint in eeprom:
                low = max(low, eeprom[joint][0])
                high = min(high, eeprom[joint][1])
            self.limits[joint] = (low, high)

        self.measured = arm.read_joints()
        self.command = dict(self.measured)
        self.target = dict(self.measured)
        self.torque = True  # FeetechArm.connect enables it
        self.speed = DEFAULT_SPEED
        #: The last target cut to its limit, as a sentence for the page; None when it fitted.
        self.clip: str | None = None
        self.faults = arm.faults()
        self.last_heartbeat = time.monotonic()
        self._token: str | None = None
        self._owner_id: str | None = None
        self._owner_label: str | None = None
        self._closed = False
        self._loop_ok = not bool(self.faults)
        self._connected = arm.is_connected
        self._sample_at = time.monotonic()
        self._sequence = 0
        self._sample_times = {self._sequence: self._sample_at}
        self._events: list[dict[str, Any]] = []
        self._tick_times: list[float] = []
        self._model_session: str | None = None
        self._model_deadline = 0.0
        self._model_last_action = 0.0
        self._model_speed = 0.0
        self._model_supervisor: Any = None
        self._model_cameras: Callable[[], dict[str, Any]] | None = None
        self._camera_change_in_progress = False
        self._cartesian_workspace = Workspace.from_environment(os.environ)
        self._cartesian_solver: Any = None
        self._cartesian_error: str | None = None
        self._cartesian_motion: dict[str, Any] | None = None
        self._cartesian_sequence = -1
        self._cartesian_profile = "slow"
        self._cartesian_mode = "normal"
        self._cartesian_recovery_reason: str | None = None
        self._cartesian_stop_reason: str | None = None
        self._cartesian_stopped_motion_id: str | None = None
        self._cartesian_ranges = CartesianRangeEstimator(
            lambda: CartesianSolver(description.robot_dir, dict(self.limits)),
            self.limits, self._cartesian_workspace)
        self._cartesian_ranges.observe(self.measured, self._sequence, self._sample_at)

    def claim(self, label: str) -> dict[str, str]:
        with self._lock:
            self._require_emergency_stop_reset()
            self._expire_owner()
            if not self._healthy():
                if self._token is not None:
                    self._revoke("Feedback is stale or unhealthy; control released.", "warning")
                raise PanelError("Control is unavailable; check the connection and faults.",
                                 HTTPStatus.CONFLICT)
            if self._token is not None:
                raise PanelError("Another operator owns control.", HTTPStatus.CONFLICT)
            if not label.strip() or len(label) > 80:
                raise PanelError("The operator label must contain 1 to 80 characters.")
            self._token = secrets.token_urlsafe(32)
            self._owner_id = secrets.token_hex(8)
            self._owner_label = label.strip()
            self._cartesian_sequence = -1
            self.last_heartbeat = time.monotonic()
            self._event("info", f"Control claimed by {self._owner_label}.")
            return {"token": self._token, "owner_id": self._owner_id}

    def heartbeat(self, token: str) -> None:
        with self._lock:
            self._authorize(token)
            self.last_heartbeat = time.monotonic()

    def release(self, token: str) -> None:
        with self._lock:
            self._authorize(token)
            self._revoke("Control released.")

    def _healthy(self) -> bool:
        return (not self._closed and self._loop_ok and self._connected
                and time.monotonic() - self._sample_at <= HEARTBEAT_TIMEOUT_S)

    def _authorize(self, token: str) -> None:
        self._require_emergency_stop_reset()
        self._expire_owner()
        if not self._healthy():
            if self._token is not None:
                self._revoke("Feedback is stale or unhealthy; control released.", "warning")
            raise PanelError("Fresh robot feedback is required to control the arm.", HTTPStatus.CONFLICT)
        if self._token is None or not secrets.compare_digest(self._token.encode(), token.encode()):
            raise PanelError("A live control token is required.", HTTPStatus.FORBIDDEN)

    def _expire_owner(self) -> None:
        if self._token is not None and time.monotonic() - self.last_heartbeat > HEARTBEAT_TIMEOUT_S:
            self._revoke("Control heartbeat expired; holding the arm.", "warning")

    def _revoke(self, message: str, level: str = "info") -> None:
        if self._cartesian_motion is not None:
            self._cartesian_stop_reason = message
            self._cartesian_stopped_motion_id = self._cartesian_motion["motion_id"]
        self._hold()
        self._clear_model()
        self._token = self._owner_id = self._owner_label = None
        self._event(level, message)
        if self.torque:
            self._send(self.command)

    def _event(self, level: str, message: str) -> None:
        if self._events and self._events[-1]["level"] == level and self._events[-1]["message"] == message:
            return
        self._events.append({"timestamp": time.time(), "level": level, "message": message})
        del self._events[:-100]

    def jog(self, joint: str, delta: float) -> None:
        with self._lock:
            if joint not in self.limits:
                raise PanelError(f"Unknown joint: {joint}")
            self._aim({joint: self.target[joint] + delta})

    def cartesian_state(self) -> dict[str, Any]:
        """Cached capabilities; configured TCP floor is not collision clearance."""
        motion = self._cartesian_motion
        workspace = self._cartesian_workspace.snapshot()
        ready = (self._healthy() and self.torque and not self._emergency_stop.latched
                 and self._model_session is None)
        reason = self._cartesian_error or workspace["error"]
        if not workspace["configured"] and reason is None:
            reason = "Configure RAMION_CARTESIAN_TABLE_Z_M in the robot base frame for spatial motion."
        if reason is None and self._cartesian_solver is None and util.find_spec("machinekind_twin") is None:
            reason = "Spatial motion requires the optional workbench kinematics dependency."
        orientation_reason = reason or (None if ready else "Fresh enabled arm control is required.")
        tilt_reason = orientation_reason
        tilt_limits = self.limits.get("wrist_flex")
        if tilt_reason is None and tilt_limits is None:
            tilt_reason = "This robot has no wrist-flex joint."
        if (tilt_reason is None and tilt_limits is not None
                and not tilt_limits[0] <= self.measured["wrist_flex"] <= tilt_limits[1]):
            tilt_reason = "Return wrist tilt inside its limits before tilting."
        roll_reason = orientation_reason
        if roll_reason is None and "wrist_roll" not in self.limits:
            roll_reason = "This robot has no wrist-roll joint."
        roll_limits = self.limits.get("wrist_roll")
        if (roll_reason is None and roll_limits is not None
                and not roll_limits[0] <= self.measured["wrist_roll"] <= roll_limits[1]):
            roll_reason = "Return wrist roll inside its limits before rolling."
        return {"active": motion is not None, "motion_id": motion["motion_id"] if motion else None,
                "direction": motion["direction"] if motion else None, "profile": self._cartesian_profile,
                "sequence": self._cartesian_sequence,
                "expires_in_s": max(0.0, motion["deadline"] - time.monotonic()) if motion else 0.0,
                "watchdog_s": WATCHDOG_S, "available": ready,
                "orientation": "bounded_tilt" if self._cartesian_mode == "recovery" else "fixed_tilt",
                "orientation_controls": {
                    "supported": True, "tilt": {"supported": True, "available": tilt_reason is None,
                                                "reason": tilt_reason},
                    "roll": {"supported": True, "available": roll_reason is None, "reason": roll_reason},
                    "angular_rate_key": "angular_deg_s", "joint_rate_key": "joint_deg_s",
                    "tilt_tip_compensated": False, "roll_tip_compensated": False,
                    "tilt_view": "tip_up_down", "tilt_up_joint_sign": -1, "tilt_down_joint_sign": 1,
                    "tilt_floor_reentry_noise_m": TILT_FLOOR_NOISE_M,
                    "roll_view": "from_wrist_toward_tip", "roll_left_joint_sign": 1,
                    "roll_right_joint_sign": -1},
                "mode": self._cartesian_mode, "phase": motion["phase"] if motion else None,
                "stop_reason": self._cartesian_stop_reason,
                "stopped_motion_id": self._cartesian_stopped_motion_id,
                "floor_reentry_active": bool(motion and motion.get("floor_reentry")),
                "recovery": {"supported": True, "max_soft_violation_deg": MAX_SOFT_VIOLATION_DEG,
                             "ingress_inner_margin_deg": INGRESS_INNER_MARGIN_DEG,
                             "max_tip_drift_m": MAX_TIP_DRIFT_M,
                             "max_pitch_change_deg": MAX_PITCH_CHANGE_DEG,
                             "phase": (motion["phase"] if motion and self._cartesian_mode == "recovery"
                                       else None),
                             "pitch_change_deg": motion.get("pitch_change_deg") if motion else None,
                             "tip_drift_m": motion.get("tip_drift_m") if motion else None,
                             "reason": self._cartesian_recovery_reason},
                "joint_limits": {"software": dict(self.software_limits),
                                 "hardware": dict(self.hardware_limits),
                                 "effective": dict(self.limits)},
                "ranges": self._cartesian_ranges.snapshot(),
                "spatial_available": ready and reason is None, "reason": reason,
                "workspace": workspace, "profiles": {key: dict(value) for key, value in PROFILES.items()}}

    def _cartesian_order(self, sequence: int) -> None:
        if type(sequence) is not int or sequence < 0 or sequence <= self._cartesian_sequence:
            raise PanelError("Cartesian sequence must increase for this control lease.", HTTPStatus.CONFLICT)
        self._cartesian_sequence = sequence

    def _end_cartesian(self, reason: str | None = None) -> None:
        if self._cartesian_motion is not None:
            self._cartesian_stop_reason = reason
            self._cartesian_stopped_motion_id = self._cartesian_motion["motion_id"]
            self._cartesian_motion = None
            # Jogging has no queued trajectory: stop at the last accepted setpoint.
            # Rebasing to feedback here ratchets a steady servo offset into motion.
            self.target = dict(self.command)
            if reason:
                if self._cartesian_mode == "recovery":
                    self._cartesian_recovery_reason = reason
                self.clip = reason
                self._event("warning", reason)

    def _cartesian_watchdog(self) -> None:
        if self._cartesian_motion is not None and time.monotonic() >= self._cartesian_motion["deadline"]:
            self._end_cartesian("Hold-jog timed out; press again to move.")

    def start_cartesian(self, direction: str, profile: str, sequence: int,
                        observation_sequence: int, mode: str = "normal") -> dict[str, Any]:
        self._cartesian_watchdog()
        self._cartesian_order(sequence)
        if not isinstance(profile, str) or direction not in DIRECTIONS or profile not in PROFILES:
            raise PanelError("Unknown Cartesian direction or speed profile.")
        if mode not in ("normal", "recovery"):
            raise PanelError("Unknown Cartesian mode.")
        translation = direction in TRANSLATION_DIRECTIONS
        if not translation:
            mode = "normal"
        plan: list[dict[str, float]] = []
        floor_reentry: dict[str, float] | None = None
        if self._cartesian_motion is not None:
            raise PanelError("Release the active direction before starting another.", HTTPStatus.CONFLICT)
        sample_at = self._sample_times.get(observation_sequence)
        if sample_at is None or not 0 <= time.monotonic() - sample_at <= OBSERVATION_MAX_AGE_S:
            raise PanelError("A fresh observation is required to start hold-jog.", HTTPStatus.CONFLICT)
        if not self.torque:
            raise PanelError("Torque is off; turn it on to move the arm.", HTTPStatus.CONFLICT)
        for joint, (low, high) in self.hardware_limits.items():
            measured = self.measured.get(joint)
            if measured is None or not math.isfinite(measured) or not low <= measured <= high:
                raise PanelError(f"Measured {joint} is outside its EEPROM limit; check calibration locally.",
                                 HTTPStatus.CONFLICT)
        if direction not in ("open", "close"):
            if self.description.name != "so101":
                raise PanelError("Spatial hold-jog currently supports the SO-101 model only.")
            readiness = self.cartesian_state()
            if not readiness["spatial_available"]:
                raise PanelError(readiness["reason"] or "Spatial motion is unavailable.", HTTPStatus.CONFLICT)
            if direction in ORIENTATION_DIRECTIONS:
                kind = "tilt" if direction in TILT_DIRECTIONS else "roll"
                capability = readiness["orientation_controls"][kind]
                if not capability["available"]:
                    raise PanelError(capability["reason"], HTTPStatus.CONFLICT)
            if self._cartesian_solver is None:
                try:
                    self._cartesian_solver = CartesianSolver(self.description.robot_dir, self.limits)
                except Exception as exc:
                    self._cartesian_error = f"Cartesian kinematics unavailable: {exc}"
                    raise PanelError(self._cartesian_error, HTTPStatus.CONFLICT) from exc
            if direction in TILT_DIRECTIONS:
                floor_reentry = self._prepare_tilt_floor_reentry(direction)
            else:
                self._check_cartesian_floor(self.measured)
                self._check_cartesian_floor(self.command)
            if translation:
                try:
                    if mode == "recovery":
                        # Feedback still has to satisfy the recovery envelope, but
                        # the trajectory starts at the setpoint already on the bus.
                        ingress_plan(self._cartesian_solver, self.measured, self.limits,
                                     self.hardware_limits, self._preview_cartesian_goals)
                        plan = ingress_plan(self._cartesian_solver, self.command, self.limits,
                                            self.hardware_limits, self._preview_cartesian_goals)
                        for waypoint in plan:
                            self._check_cartesian_floor(waypoint)
                    else:
                        # A feedback overshoot must still require explicit recovery;
                        # command-based anchoring does not relax measured limits.
                        self._cartesian_solver.solve(
                            self.measured, self._cartesian_solver.position(self.measured),
                            pitch=self._cartesian_solver.pitch(self.measured))
                        self._cartesian_solver.solve(
                            self.command, self._cartesian_solver.position(self.command),
                            pitch=self._cartesian_solver.pitch(self.command))
                except ValueError as exc:
                    self._cartesian_recovery_reason = str(exc) if mode == "recovery" else None
                    raise PanelError(str(exc), HTTPStatus.CONFLICT) from exc
            # Loading the optional model can be slow. Never give a delayed press
            # a fresh motion deadline after its original observation has expired.
            if time.monotonic() - sample_at > OBSERVATION_MAX_AGE_S or not self._healthy():
                raise PanelError("Kinematics loaded; refresh feedback and press again.", HTTPStatus.CONFLICT)
        if direction in ("open", "close") and self.description.gripper is None:
            raise PanelError("This robot has no gripper.")
        # Cancel any unfinished manual target without adopting tracking error as
        # a new setpoint. Unrelated joints must retain their existing commands.
        self.target = dict(self.command)
        self.clip = None
        self._cartesian_profile = profile
        self._cartesian_mode = mode
        self._cartesian_recovery_reason = None
        self._cartesian_stop_reason = None
        self._cartesian_stopped_motion_id = None
        self._cartesian_motion = {"motion_id": secrets.token_hex(16), "direction": direction,
                                  "deadline": time.monotonic() + WATCHDOG_S, "travel_m": 0.0,
                                  "achieved_m": 0.0,
                                  "elapsed_s": 0.0, "phase": "ingress" if plan else "tracking",
                                  "ingress_plan": plan, "ingress_index": -1,
                                  "angle_deg": 0.0, "achieved_angle_deg": 0.0,
                                  "floor_reentry": floor_reentry,
                                  "pitch_change_deg": 0.0, "tip_drift_m": 0.0}
        if translation or direction in ORIENTATION_DIRECTIONS:
            self._cartesian_motion["anchor_position"] = self._cartesian_solver.position(self.command)
            self._cartesian_motion["anchor_pitch"] = self._cartesian_solver.pitch(self.command)
        if direction in ORIENTATION_DIRECTIONS:
            self._cartesian_motion["anchor_joints"] = dict(self.command)
            self._cartesian_motion["angle_sign"] = -1 if direction in ("tilt_up", "roll_right") else 1
            self._cartesian_motion["orientation_joint"] = (
                "wrist_flex" if direction in TILT_DIRECTIONS else "wrist_roll")
        if translation:
            heading = self._cartesian_solver.heading(self.command)
            self._cartesian_motion["vector"] = (
                (0.0, 0.0, 1.0 if direction == "up" else -1.0) if direction in ("up", "down")
                else (heading[0] * (1 if direction == "forward" else -1),
                      heading[1] * (1 if direction == "forward" else -1), 0.0))
        return {"cartesian": self.cartesian_state()}

    def _prepare_tilt_floor_reentry(self, direction: str) -> dict[str, float] | None:
        """Authorize only an upward encoder step from an existing floor deficit."""
        floor = self._cartesian_workspace.snapshot()["min_tcp_z_m"]
        position = self._cartesian_solver.position(self.measured)
        command_position = self._cartesian_solver.position(self.command)
        if floor is None or not all(math.isfinite(value) for value in (*position, *command_position)):
            raise PanelError("Configured TCP floor or measured pose is invalid.", HTTPStatus.CONFLICT)
        if position[2] >= floor and command_position[2] >= floor:
            return None
        probe = dict(self.command)
        probe["wrist_flex"] += (-1 if direction == "tilt_up" else 1) * 360.0 / 4096
        probe = self._preview_cartesian_goals(probe)
        low, high = self.limits["wrist_flex"]
        if not low <= probe["wrist_flex"] <= high:
            raise PanelError("Wrist-tilt limit prevents upward TCP-floor re-entry.", HTTPStatus.CONFLICT)
        probe_z = self._cartesian_solver.position(probe)[2]
        if not math.isfinite(probe_z) or probe_z <= command_position[2] + 1e-9:
            raise PanelError("This tilt would lower the tip below the TCP floor; use the upward direction.",
                             HTTPStatus.CONFLICT)
        return {"anchor_z": position[2], "best_measured_z": position[2], "last_goal_z": command_position[2],
                "last_progress_z": position[2], "last_progress_at": time.monotonic()}

    def _check_tilt_floor_reentry(self, motion: dict[str, Any], goals: dict[str, float]) -> None:
        state = motion["floor_reentry"]
        floor = self._cartesian_workspace.snapshot()["min_tcp_z_m"]
        measured_z = self._cartesian_solver.position(self.measured)[2]
        command_z = self._cartesian_solver.position(self.command)[2]
        goal_z = self._cartesian_solver.position(goals)[2]
        if not all(math.isfinite(value) for value in (measured_z, command_z, goal_z)):
            raise ValueError("TCP floor re-entry requires finite feedback and goals.")
        if measured_z >= floor and command_z >= floor:
            motion["floor_reentry"] = None
            self._check_cartesian_floor(self.measured)
            self._check_cartesian_floor(goals)
            return
        if goal_z < state["last_goal_z"] - 1e-9:
            raise ValueError("TCP-floor re-entry may not lower the commanded tip.")
        reference_z = max(state["anchor_z"], state["best_measured_z"])
        if measured_z < reference_z - TILT_FLOOR_NOISE_M:
            raise ValueError("Measured tip worsened during TCP-floor re-entry; motion stopped.")
        state["best_measured_z"] = max(state["best_measured_z"], measured_z)
        if state["best_measured_z"] > state["last_progress_z"] + 0.00005:
            state["last_progress_z"] = state["best_measured_z"]
            state["last_progress_at"] = time.monotonic()
        elif time.monotonic() - state["last_progress_at"] > TILT_FLOOR_PROGRESS_TIMEOUT_S:
            raise ValueError("TCP-floor re-entry made no measured upward progress; check the arm locally.")

    def _orientation_goals(self, motion: dict[str, Any], profile: dict[str, float],
                           dt: float) -> tuple[dict[str, float], float]:
        """One direct wrist-axis trajectory, anchored at the deliberate press."""
        joint = motion["orientation_joint"]
        sign = motion["angle_sign"]
        increment = profile["angular_deg_s"] * dt * min(1.0, motion["elapsed_s"] / 0.2)
        for _ in range(8):
            travel = motion["angle_deg"] + increment
            goals = dict(motion["anchor_joints"])
            goals[joint] += sign * travel
            goals = self._preview_cartesian_goals(goals)
            ratio = max(abs(goals[joint] - self.command[joint]) /
                        max(min(profile["joint_deg_s"], self.description.max_vel[joint]) * dt, 1e-12)
                        for joint in self.description.joints if joint != self.description.gripper)
            if ratio <= 1.0:
                achieved = sign * (goals[joint] - motion["anchor_joints"][joint])
                if achieved < motion["achieved_angle_deg"] - 1e-8:
                    goals = dict(self.command)
                return goals, travel
            increment *= min(0.8, 0.9 / ratio)
        raise ValueError("Orientation hold reached a local joint-speed boundary.")

    def _preview_cartesian_goals(self, goals: dict[str, float]) -> dict[str, float]:
        """Pure encoder conversion, without a bus write or temporary limit change."""
        to_ticks = getattr(self.arm, "_to_ticks", None)
        to_units = getattr(self.arm, "_to_units", None)
        if to_ticks is None or to_units is None:
            return dict(goals)
        return {joint: to_units(joint, to_ticks(joint, value)) for joint, value in goals.items()}

    def renew_cartesian(self, motion_id: str, sequence: int) -> dict[str, Any]:
        self._cartesian_watchdog()
        self._cartesian_order(sequence)
        if self._cartesian_motion is None or motion_id != self._cartesian_motion["motion_id"]:
            if self._cartesian_motion is None and self._cartesian_stop_reason:
                raise PanelError(f"Hold-jog has ended: {self._cartesian_stop_reason}", HTTPStatus.CONFLICT)
            raise PanelError("Hold-jog has ended; press again to move.", HTTPStatus.CONFLICT)
        self._cartesian_motion["deadline"] = time.monotonic() + WATCHDOG_S
        return {"cartesian": self.cartesian_state()}

    def stop_cartesian(self, motion_id: str | None, sequence: int) -> dict[str, Any]:
        self._cartesian_order(sequence)
        # No ID means cancellation of a press whose start response has not arrived yet.
        if (motion_id is not None and self._cartesian_motion is not None
                and motion_id != self._cartesian_motion["motion_id"]):
            raise PanelError("This hold-jog motion ID is no longer active.", HTTPStatus.CONFLICT)
        self._end_cartesian()
        return {"cartesian": self.cartesian_state()}

    def _check_cartesian_floor(self, joints: dict[str, float]) -> None:
        position = self._cartesian_solver.position(joints)
        floor = self._cartesian_workspace.snapshot()["min_tcp_z_m"]
        if floor is None or not all(math.isfinite(value) for value in position) or position[2] < floor:
            raise PanelError("TCP floor reached or invalid; spatial motion stopped.", HTTPStatus.CONFLICT)

    def _step_cartesian(self, dt: float) -> None:
        motion = self._cartesian_motion
        assert motion is not None
        profile = PROFILES[self._cartesian_profile]
        direction = motion["direction"]
        goals = dict(self.command)
        motion["elapsed_s"] += dt
        accepted_travel = motion["travel_m"]
        accepted_angle = motion["angle_deg"]
        ingress = motion["phase"] == "ingress"
        write_attempted = False
        try:
            if ingress:
                plan = motion["ingress_plan"]
                index = motion["ingress_index"]
                ingress_ramp = min(1.0, motion["elapsed_s"] / 0.2)
                ingress_dt = dt * ingress_ramp
                for candidate_index in range(index + 1, len(plan)):
                    candidate = plan[candidate_index]
                    if all(abs(candidate[joint] - self.command[joint]) <=
                           min(profile["joint_deg_s"], self.description.max_vel[joint]) * ingress_dt + 1e-8
                           for joint in self.description.joints):
                        goals, index = candidate, candidate_index
                    else:
                        break
                motion["ingress_index"] = index
                if index == len(plan) - 1:
                    if all(self.limits[joint][0] <= self.measured[joint] <= self.limits[joint][1]
                           for joint in self.description.joints):
                        motion["phase"] = "tracking"
                    elif time.monotonic() - motion.setdefault("settling_started", time.monotonic()) > 1.0:
                        raise ValueError("Recovery joint did not return inside its limit; check locally.")
            elif direction in ORIENTATION_DIRECTIONS:
                goals, accepted_angle = self._orientation_goals(motion, profile, dt)
            elif direction in ("left", "right", "open", "close"):
                joint = "shoulder_pan" if direction in ("left", "right") else self.description.gripper
                if joint is None:
                    raise ValueError("This robot has no gripper.")
                rate = profile["angular_deg_s"] if joint == "shoulder_pan" else profile["gripper_units_s"]
                # SO-101 pan positive rotates toward base -Y (right).
                goals[joint] += rate * dt * (1 if direction in ("right", "open") else -1)
            else:
                x, y, z = motion["anchor_position"]
                # Accelerate the scalar path speed over 200 ms. Joint coordinates
                # are solved together; no independent joint smoothing bends the line.
                ramp = min(1.0, motion["elapsed_s"] / 0.2)
                distance = profile["linear_m_s"] * dt * ramp
                vector = motion["vector"]
                floor = self._cartesian_workspace.snapshot()["min_tcp_z_m"]
                # Near extension, the requested TCP rate may need more joint speed
                # than allowed. Shorten the same Cartesian segment and solve again;
                # never ramp joints independently off the requested line.
                for _ in range(6):
                    travel = motion["travel_m"] + distance
                    position = (x + travel * vector[0], y + travel * vector[1], z + travel * vector[2])
                    if position[2] < floor:
                        raise ValueError("Configured TCP floor reached.")
                    if self._cartesian_mode == "recovery":
                        goals = self._cartesian_solver.solve_position(
                            self.command, position, pitch_anchor=motion["anchor_pitch"],
                            pitch_budget=math.radians(MAX_PITCH_CHANGE_DEG))
                    else:
                        goals = self._cartesian_solver.solve(self.command, position,
                                                             pitch=motion["anchor_pitch"])
                    goals = self._preview_cartesian_goals(goals)
                    ratio = max(abs(goals[joint] - self.command[joint]) /
                                max(min(profile["joint_deg_s"], self.description.max_vel[joint]) * dt, 1e-12)
                                for joint in self.description.joints if joint != self.description.gripper)
                    if ratio <= 1.0:
                        accepted_travel = travel
                        break
                    distance *= min(0.8, 0.9 / ratio)
                else:
                    raise ValueError("Local joint speed boundary reached.")
                quantized_point = self._cartesian_solver.position(goals)
                quantized_progress = sum((quantized_point[index] - motion["anchor_position"][index]) *
                                         motion["vector"][index] for index in range(3))
                if quantized_progress < motion["achieved_m"] - 1e-9:
                    # Rounding a coordinated pose can briefly move the tip backward.
                    # Hold the last encoder pose while bounded fractional progress
                    # accumulates; never oscillate commands to chase that fraction.
                    goals = dict(self.command)
            if set(goals) != set(self.description.joints):
                raise ValueError("Kinematics returned an incomplete pose.")
            for joint, value in goals.items():
                low, high = self.limits[joint]
                rate = (profile["gripper_units_s"] if joint == self.description.gripper
                        else profile["joint_deg_s"])
                max_step = min(rate, self.description.max_vel[joint]) * dt + 1e-8
                # An unchanged measured soft-limit violation must not trap unrelated
                # pan/gripper controls. It needs verified EEPROM bounds and may never
                # worsen. Translation recovery follows a separate explicit policy.
                unchanged = (direction in (("left", "right", "open", "close") + tuple(ORIENTATION_DIRECTIONS))
                             and abs(value - self.command[joint]) <= 1e-9
                             and joint in self.hardware_limits
                             and self.hardware_limits[joint][0] <= value <= self.hardware_limits[joint][1])
                physical = self.hardware_limits.get(joint)
                monotonic_ingress = (ingress and physical is not None and physical[0] <= value <= physical[1]
                                     and max(low - value, value - high, 0.0) <=
                                     max(low - self.command[joint], self.command[joint] - high, 0.0) + 1e-8)
                if (not math.isfinite(value) or (not low <= value <= high and not unchanged
                                               and not monotonic_ingress)
                        or abs(value - self.command[joint]) > max_step):
                    raise ValueError("Joint limit, speed limit or local reach boundary reached.")
            if direction not in ("open", "close"):
                if direction in TILT_DIRECTIONS and motion.get("floor_reentry"):
                    self._check_tilt_floor_reentry(motion, goals)
                else:
                    self._check_cartesian_floor(self.measured)
                    self._check_cartesian_floor(goals)
            if direction in ORIENTATION_DIRECTIONS:
                moving = {motion["orientation_joint"]}
                if any(abs(goals[joint] - motion["anchor_joints"][joint]) > 1e-8
                       for joint in self.description.joints if joint not in moving):
                    raise ValueError("Orientation hold attempted to move an unrelated joint.")
            if direction in ("forward", "back", "up", "down"):
                for pose in ((self.measured, goals) if self._cartesian_mode == "recovery" else (goals,)):
                    point = self._cartesian_solver.position(pose)
                    displacement = [point[index] - motion["anchor_position"][index] for index in range(3)]
                    along = sum(displacement[index] * motion["vector"][index] for index in range(3))
                    drift = (math.dist(point, motion["anchor_position"]) if ingress else
                             math.sqrt(sum((displacement[index] - along * motion["vector"][index]) ** 2
                                           for index in range(3))))
                    tilt = abs(self._cartesian_solver._angle(
                        self._cartesian_solver.pitch(pose) - motion["anchor_pitch"]))
                    drift_limit = MAX_TIP_DRIFT_M if self._cartesian_mode == "recovery" else 0.001
                    tilt_limit = MAX_PITCH_CHANGE_DEG if self._cartesian_mode == "recovery" else 0.25
                    if drift > drift_limit or tilt > math.radians(tilt_limit):
                        raise ValueError("Cartesian gesture reached its total tip or tilt budget.")
            # Optional model work must not turn an expired hold into a late write.
            if time.monotonic() >= motion["deadline"]:
                self._end_cartesian("Hold-jog timed out during planning; press again to move.")
                return
            if time.monotonic() - self._sample_at > OBSERVATION_MAX_AGE_S:
                self._revoke("Hold-jog feedback expired during planning; control released.", "warning")
                return
            write_attempted = True
            sent = self._send(goals)
            if set(sent) != set(goals):
                raise ValueError("Hold-jog write incomplete; press again after checking the connection.")
            if any(not math.isfinite(sent[joint]) or abs(sent[joint] - goals[joint]) > 0.1
                   for joint in goals):
                raise ValueError("Hold-jog goal was clipped by the backend; motion stopped.")
            self.command = dict(sent)
            self.target = dict(sent)
            if motion.get("floor_reentry"):
                sent_z = self._cartesian_solver.position(self.command)[2]
                if sent_z < motion["floor_reentry"]["last_goal_z"] - 1e-9:
                    raise ValueError("Accepted TCP-floor re-entry goal lowered the tip.")
                motion["floor_reentry"]["last_goal_z"] = sent_z
            if direction in ORIENTATION_DIRECTIONS:
                reached = self._cartesian_solver.position(self.command)
                motion["tip_drift_m"] = math.dist(reached, motion["anchor_position"])
                joint = motion["orientation_joint"]
                motion["achieved_angle_deg"] = motion["angle_sign"] * (
                    self.command[joint] - motion["anchor_joints"][joint])
                if abs(motion["achieved_angle_deg"] - accepted_angle) > 0.25:
                    raise ValueError("Encoder tracking departed from the continuous orientation path.")
                motion["angle_deg"] = accepted_angle
            elif direction in TRANSLATION_DIRECTIONS:
                reached = self._cartesian_solver.position(self.command)
                delta = tuple(reached[index] - motion["anchor_position"][index] for index in range(3))
                progress = sum(delta[index] * motion["vector"][index] for index in range(3))
                transverse = math.sqrt(sum((delta[index] - progress * motion["vector"][index]) ** 2
                                           for index in range(3)))
                pitch_error = self._cartesian_solver._angle(
                    self._cartesian_solver.pitch(self.command) - motion["anchor_pitch"])
                tip_drift = math.dist(reached, motion["anchor_position"]) if ingress else transverse
                motion["pitch_change_deg"] = math.degrees(pitch_error)
                motion["tip_drift_m"] = tip_drift
                maximum_drift = MAX_TIP_DRIFT_M if self._cartesian_mode == "recovery" else 0.001
                maximum_pitch = MAX_PITCH_CHANGE_DEG if self._cartesian_mode == "recovery" else 0.25
                if tip_drift > maximum_drift or abs(pitch_error) > math.radians(maximum_pitch):
                    raise ValueError("Accepted goal departed from the fixed-tilt Cartesian path.")
                if not ingress:
                    if abs(progress - accepted_travel) > 0.002:
                        raise ValueError("Encoder tracking departed from the continuous Cartesian path.")
                    # Keep accepted continuous progress, including sub-encoder-tick
                    # fractions. Stops and refused writes never accumulate a backlog.
                    motion["travel_m"] = accepted_travel
                    motion["achieved_m"] = progress
        except (ValueError, PanelError) as exc:
            if write_attempted:
                # A refused/invalid bus result or failed post-write check is a
                # fault: immediately hold feedback and release ownership.
                self._revoke(str(exc), "warning")
            else:
                self._end_cartesian(str(exc))

    def set_target(self, joint: str, value: float) -> None:
        with self._lock:
            if joint not in self.limits:
                raise PanelError(f"Unknown joint: {joint}")
            self._aim({joint: value})

    def home(self) -> None:
        """Targets the `home` pose of `robot.yaml`; the loop ramps there at the panel speed."""
        with self._lock:
            self._aim(self.description.home)

    def stop(self) -> None:
        """Holds the arm where it was measured last. Torque stays on, so the arm does not drop."""
        with self._lock:
            if not self._emergency_stop.latched:
                self._revoke("STOP requested; holding the arm.", "warning")

    def _require_emergency_stop_reset(self) -> None:
        if self._emergency_stop.latched:
            raise PanelError("Emergency stop is latched. Reset it before claiming new control.",
                             HTTPStatus.CONFLICT)

    def emergency_stop(self) -> dict[str, Any]:
        with self._lock:
            stop_state = self._emergency_stop
            # Even an already-latched STOP supersedes any in-flight reset.
            stop_state.generation += 1
            if not stop_state.latched:
                stop_state.latched = True
                self._event("warning", "Emergency stop latched; control revoked.")
                self._apply_emergency_hold()
            try:
                stop_state.persist(latched=True)
            except OSError as exc:
                self._event("error", "Emergency stop is latched in memory but could not be saved.")
                raise PanelError("Emergency stop is latched, but saving it failed. Check station storage.",
                                 HTTPStatus.SERVICE_UNAVAILABLE) from exc
            return stop_state.snapshot()

    def _apply_emergency_hold(self) -> None:
        # Clear authorization and pending goals before any serial I/O.
        self._clear_model()
        self._token = self._owner_id = self._owner_label = None
        self._hold()
        try:
            self.measured = self.arm.read_joints()
            self._send_emergency_hold()
        except Exception:
            self._emergency_stop.hold_status = "unconfirmed"
            self._emergency_stop.hold_error = "The arm hold request failed. Check the arm locally."
            logger.exception("Could not send the emergency stop hold")

    def _send_emergency_hold(self, *, retry_only: bool = False) -> None:
        """Only send a pose from the complete measurement just read by this caller."""
        stop_state = self._emergency_stop
        if not self.torque:
            stop_state.hold_status = "unavailable"
            stop_state.hold_error = "The controller has torque off. No hold was sent."
            return
        if (not self.arm.is_connected or getattr(self.arm, "_link_silent", False)
                or getattr(self.arm, "_stale", []) or set(self.measured) != set(self.description.joints)
                or any(not math.isfinite(value) for value in self.measured.values())):
            stop_state.hold_status = "pending"
            stop_state.hold_error = "Waiting for complete fresh arm feedback before sending a hold."
            return
        if retry_only and stop_state.hold_status == "sent":
            return
        self._hold()
        sent = self._send(self.command)
        if set(sent) != set(self.description.joints):
            stop_state.hold_status = "unconfirmed"
            stop_state.hold_error = "The controller could not send the complete hold goal."
            return
        stop_state.hold_status = "sent"
        stop_state.hold_error = None

    def reset_emergency_stop(self, generation: int) -> dict[str, Any]:
        with self._lock:
            stop_state = self._emergency_stop
            if generation != stop_state.generation:
                raise PanelError("Emergency stop changed. Review the current stop before resetting it.",
                                 HTTPStatus.CONFLICT)
            if stop_state.latched:
                if (not getattr(self, "preview_only", False) and self.torque
                        and stop_state.hold_status != "sent"):
                    raise PanelError("Wait for fresh arm feedback and a hold write before resetting.",
                                     HTTPStatus.CONFLICT)
                try:
                    stop_state.persist(latched=False)
                except OSError as exc:
                    raise PanelError("Emergency stop reset could not be saved. The latch remains set.",
                                     HTTPStatus.SERVICE_UNAVAILABLE) from exc
                stop_state.latched = False
                stop_state.hold_status = "not_requested"
                stop_state.hold_error = None
                self._event("info", "Emergency stop reset. Claim new control to move the arm.")
            return stop_state.snapshot()

    def set_torque(self, on: bool) -> None:
        with self._lock:
            self._require_emergency_stop_reset()
            if on:
                # The goal register keeps where the servo was sent last: torque enabled before the goal
                # is overwritten would jerk the arm there. Same order as FeetechArm.connect.
                self.measured = self.arm.read_joints()
                self._hold()
                if not self._send(self.measured):
                    raise PanelError("The goal did not reach the servos; torque stays off.")
            for dev_id in self.arm.ids.values():
                self.arm.bus.write(dev_id, ADDR_TORQUE_ENABLE, 1 if on else 0, 1)
            self.torque = on

    def set_speed(self, value: float) -> None:
        if value <= 0.0:
            raise PanelError("The speed must be positive.")
        with self._lock:
            self._require_emergency_stop_reset()
            self.speed = value

    def _camera_problem(self) -> str | None:
        if self._model_cameras is None:
            return "Model control needs the workbench's overhead and wrist cameras."
        cameras = self._model_cameras()
        for name in ("overhead", "wrist"):
            camera = cameras.get(name, {})
            age = camera.get("age_s")
            if (camera.get("status") != "live" or not isinstance(age, (int, float))
                    or isinstance(age, bool) or not math.isfinite(age)
                    or not 0 <= age <= MODEL_CAMERA_MAX_AGE_S):
                return f"The {name} camera is missing or stale; model control requires fresh frames."
        return None

    def _new_model_supervisor(self, speed: float) -> Any:
        try:
            from machinekind_twin.config import JointConfig, SafetyConfig, TwinConfig
            from machinekind_twin.safety import SafetySupervisor
        except ImportError as exc:
            raise PanelError("Model control needs the workbench dependency group (SafetySupervisor).",
                             HTTPStatus.SERVICE_UNAVAILABLE) from exc
        cfg = TwinConfig(
            joints={joint: JointConfig(min=self.limits[joint][0], max=self.limits[joint][1],
                                       max_vel=min(speed, self.description.max_vel[joint]))
                    for joint in self.description.joints},
            safety=SafetyConfig(return_home_timeout_s=0.0, home_on_exit=False,
                                home=dict(self.measured)),
        )
        supervisor = SafetySupervisor(cfg, gripper=self.description.gripper)
        supervisor.start(dict(self.measured), go_home=False, keep_outside=True)
        return supervisor

    def arm_model(self, duration_s: float, speed: float) -> dict[str, Any]:
        """Begin a bounded session at the measured pose; arming never requests Home or enables torque."""
        with self._lock:
            self._require_emergency_stop_reset()
            if self._cartesian_motion is not None:
                raise PanelError("Release hold-jog before arming a model session.", HTTPStatus.CONFLICT)
            if self._model_session is not None:
                raise PanelError("A model session is already armed.", HTTPStatus.CONFLICT)
            if self._camera_change_in_progress:
                raise PanelError("Wait for camera reconfiguration to finish before arming.",
                                 HTTPStatus.CONFLICT)
            if not 0 < duration_s <= MODEL_MAX_DURATION_S:
                raise PanelError(f"Model duration must be between 0 and {MODEL_MAX_DURATION_S:g} seconds.")
            if not 0 < speed <= MODEL_MAX_SPEED:
                raise PanelError(f"Model speed must be between 0 and {MODEL_MAX_SPEED:g} units/s.")
            if not self.torque:
                raise PanelError("Torque is off; model sessions cannot enable it.", HTTPStatus.CONFLICT)
            problem = self._camera_problem()
            if problem:
                raise PanelError(problem, HTTPStatus.CONFLICT)
            self._model_supervisor = self._new_model_supervisor(speed)
            self._hold()
            self._model_session = secrets.token_hex(16)
            now = time.monotonic()
            self._model_deadline = now + duration_s
            self._model_last_action = now
            self._model_speed = speed
            self._event("info", f"Model session armed for {duration_s:g}s at {speed:g} units/s.")
            return {"session_id": self._model_session, "duration_s": duration_s, "speed": speed,
                    "action_timeout_s": MODEL_ACTION_TIMEOUT_S,
                    "observation_max_age_s": MODEL_OBSERVATION_MAX_AGE_S}

    def disarm_model(self) -> None:
        """Hold and close the model session, keeping the operator lease until explicit release."""
        with self._lock:
            self._hold()
            self._clear_model()
            if self.torque:
                self._send(self.command)
            self._event("info", "Model session disarmed; holding the arm.")

    def _clear_model(self) -> None:
        self._model_session = None
        self._model_supervisor = None
        self._model_deadline = self._model_last_action = self._model_speed = 0.0

    def set_model_targets(self, session_id: str, goals: Any, observation_sequence: Any) -> None:
        """Validate a complete action before replacing any target; the loop supervises its execution."""
        with self._lock:
            self._require_emergency_stop_reset()
            if self._model_session is None or not secrets.compare_digest(self._model_session, session_id):
                raise PanelError("A current armed model session is required.", HTTPStatus.CONFLICT)
            if not self._model_watchdogs():
                raise PanelError("Model session stopped by its watchdog.", HTTPStatus.CONFLICT)
            if not isinstance(goals, dict) or set(goals) != set(self.description.joints):
                raise PanelError("Model targets must contain exactly every robot joint, "
                                 "including the gripper.")
            checked = {joint: _arg(goals, joint, float) for joint in self.description.joints}
            for joint, value in checked.items():
                low, high = self.limits[joint]
                if not low <= value <= high:
                    raise PanelError(f"Model target {joint} is outside [{low:g}, {high:g}].")
            sampled = (self._sample_times.get(observation_sequence)
                       if type(observation_sequence) is int else None)
            if sampled is None or time.monotonic() - sampled > MODEL_OBSERVATION_MAX_AGE_S:
                self._revoke("Model observation is stale or unknown; holding the arm.", "warning")
                raise PanelError("A recent workbench observation_sequence is required.", HTTPStatus.CONFLICT)
            self.target = checked
            self.clip = None
            self._model_last_action = time.monotonic()

    def _model_watchdogs(self) -> bool:
        if self._model_session is None:
            return False
        now = time.monotonic()
        problem = None
        if now >= self._model_deadline:
            problem = "Model session duration elapsed."
        elif now - self._model_last_action > MODEL_ACTION_TIMEOUT_S:
            problem = "Model action stream timed out."
        elif not self._healthy() or not self.torque:
            problem = "Model session lost fresh robot feedback or torque."
        else:
            problem = self._camera_problem()
        if problem is None:
            for joint in self.description.joints:
                if joint != self.description.gripper and abs(self.command[joint] - self.measured[joint]) > \
                        MODEL_MAX_TRACKING_DEG:
                    problem = f"Model tracking error exceeded {MODEL_MAX_TRACKING_DEG:g} degrees at {joint}."
                    break
        if problem:
            self._revoke(problem + " Holding and releasing control.", "warning")
            return False
        return True

    def state(self) -> dict[str, Any]:
        """A cached snapshot: observers never renew control or touch the hardware."""
        with self._lock:
            now = time.monotonic()
            return {
                "robot": self.description.name,
                "port": self.arm.port,
                "joints": list(self.description.joints),
                "gripper": self.description.gripper,
                "limits": self.limits,
                "measured": dict(self.measured),
                "command": dict(self.command),
                "target": dict(self.target),
                "torque": self.torque,
                "speed": self.speed,
                "clip": self.clip,
                "faults": list(self.faults),
                "loop_hz": len(self._tick_times),
                "age_s": max(0.0, now - self._sample_at),
                "sequence": self._sequence,
                "loop_ok": self._loop_ok,
                "connected": self._connected,
                "control": {"owner_id": self._owner_id, "label": self._owner_label,
                            "expires_in_s": max(0.0, HEARTBEAT_TIMEOUT_S - (now - self.last_heartbeat))
                            if self._token is not None else 0.0},
                "sample": {"age_s": max(0.0, now - self._sample_at), "sequence": self._sequence,
                           "loop_ok": self._loop_ok, "connected": self._connected},
                "events": [dict(event) for event in self._events],
                "emergency_stop": self._emergency_stop.snapshot(),
                "cartesian": self.cartesian_state(),
                "model_session": {"active": self._model_session is not None,
                                  "expires_in_s": max(0.0, self._model_deadline - now),
                                  "speed": self._model_speed,
                                  "last_action_age_s": max(0.0, now - self._model_last_action)
                                  if self._model_session else None},
            }

    def model_readiness(self) -> dict[str, Any]:
        """Cached prerequisites, not a lease grant or an assurance of physical clearance."""
        with self._lock:
            reasons = []
            if self._emergency_stop.latched:
                reasons.append("Emergency stop is latched. Explicit reset is required.")
            if getattr(self, "preview_only", False):
                reasons.append("Disconnected preview cannot control a robot.")
            else:
                if not self._healthy():
                    reasons.append("Fresh healthy robot feedback is required.")
                if not self.torque:
                    reasons.append("Torque is off.")
                if self._camera_change_in_progress:
                    reasons.append("Camera reconfiguration is in progress.")
            if util.find_spec("machinekind_twin") is None:
                reasons.append("The SafetySupervisor dependency is not installed.")
            problem = self._camera_problem()
            if problem:
                reasons.append(problem)
            return {"ready": not reasons, "reasons": reasons, "requires_control_lease": True}

    def tick(self, dt: float) -> None:
        """One control cycle: read the joints, ramp `command` towards `target`, send it, collect faults."""
        with self._lock:
            now = time.monotonic()
            self._tick_times = [t for t in self._tick_times if now - t < 1.0]
            self._tick_times.append(now)
            try:
                was_stale = now - self._sample_at > HEARTBEAT_TIMEOUT_S
                self.measured = self.arm.read_joints()
                if was_stale and not self._emergency_stop.latched:
                    self._revoke("Feedback interrupted; previous movement discarded.", "warning")
                self._connected = self.arm.is_connected and not getattr(self.arm, "_link_silent", False)
                if not getattr(self.arm, "_stale", []):
                    self._sample_at = time.monotonic()
                    self._sequence += 1
                    self._sample_times[self._sequence] = self._sample_at
                    self._cartesian_ranges.observe(self.measured, self._sequence, self._sample_at)
                    self._sample_times = {sequence: stamp for sequence, stamp in self._sample_times.items()
                                          if self._sample_at - stamp <= MODEL_OBSERVATION_MAX_AGE_S}
                self.faults = self.arm.faults()
                self._expire_owner()
                if self._emergency_stop.latched:
                    self._loop_ok = not self.faults
                    self._send_emergency_hold(retry_only=True)
                    return
                if self.faults:
                    self._loop_ok = False
                    self._revoke("Arm fault; holding the arm.", "error")
                    return
                self._loop_ok = True
                if self._cartesian_motion is not None:
                    if not self._connected or getattr(self.arm, "_stale", []):
                        self._revoke("Hold-jog feedback interrupted; control released.", "warning")
                        return
                    if any(abs(self.command[joint] - self.measured[joint]) > 5.0
                           for joint in self.description.joints if joint != self.description.gripper):
                        self._revoke("Hold-jog tracking error; control released.", "warning")
                        return
                    self._cartesian_watchdog()
                if self._model_session is not None and not self._model_watchdogs():
                    return
                if self.torque:
                    bounded_dt = min(max(dt, 0.0), MAX_DT_S)
                    if self._cartesian_motion is not None:
                        self._step_cartesian(bounded_dt)
                    else:
                        self._step(bounded_dt)
                else:
                    self._hold()
            except Exception as exc:
                self._loop_ok = False
                self._connected = False
                self.faults = [f"control loop error: {exc}"]
                self._hold()
                self._clear_model()
                self._token = self._owner_id = self._owner_label = None
                self._event("error", self.faults[0])
                try:
                    if self._emergency_stop.latched:
                        self._emergency_stop.hold_status = "unconfirmed"
                        self._emergency_stop.hold_error = "The arm hold failed. Check the arm locally."
                    elif self.torque:
                        self._send(self.command)
                except Exception:
                    logger.exception("Could not send the fail-safe hold")
                raise

    def start(self) -> None:
        self._thread.start()

    def close(self) -> None:
        """Stops the loop, revokes control and holds without changing torque."""
        self._halt.set()
        if self._thread.is_alive():
            self._thread.join()
        self._cartesian_ranges.close()
        with self._lock:
            self._closed = True
            try:
                self.measured = self.arm.read_joints()
                if self._emergency_stop.latched:
                    self._send_emergency_hold()
                    return
            except Exception:
                logger.exception("Could not refresh the shutdown hold pose")
                if self._emergency_stop.latched:
                    return
            try:
                self._revoke("Panel closed; holding the arm.")
            except Exception:
                logger.exception("Could not send the shutdown hold")

    def _run(self) -> None:
        last = time.monotonic()
        while not self._halt.is_set():
            now = time.monotonic()
            try:
                self.tick(now - last)
            except Exception:
                logger.exception("The panel control loop failed")
            last = now
            self._halt.wait(max(0.0, 1.0 / TICK_HZ - (time.monotonic() - now)))

    # The methods below run under the lock.
    def _aim(self, goals: dict[str, float]) -> None:
        self._require_emergency_stop_reset()
        if self._cartesian_motion is not None:
            raise PanelError("Release hold-jog before another manual command.", HTTPStatus.CONFLICT)
        if not self.torque:
            raise PanelError("Torque is off; turn it on to move the arm.")

        self.clip = None
        for joint, wanted in goals.items():
            low, high = self.limits[joint]
            self.target[joint] = min(max(wanted, low), high)
            if self.target[joint] != wanted:
                name = joint.replace("_", " ")
                self.clip = f"clipped to the limit: {name} {wanted:.1f} -> {self.target[joint]:.1f}"

    def _hold(self) -> None:
        self._cartesian_motion = None
        self.command = dict(self.measured)
        self.target = dict(self.measured)
        if self._model_supervisor is not None:
            self._model_supervisor.hold(dict(self.measured))

    def _step(self, dt: float) -> None:
        if self._model_session is not None:
            step, _ = self._model_supervisor.step(dict(self.target), dt, hand_present=True, engaged=True)
            sent = self._send(step)
            for joint in sent:
                self.command[joint] = sent[joint]
            # The supervisor must follow what reached the bus even when a write was refused.
            self._model_supervisor.reseed(dict(self.command))
            return
        step: dict[str, float] = {}
        for joint in self.description.joints:
            reach = min(self.speed, self.description.max_vel[joint]) * dt
            gap = self.target[joint] - self.command[joint]
            if abs(gap) <= reach:
                step[joint] = self.target[joint]
            else:
                step[joint] = self.command[joint] + math.copysign(reach, gap)

        # A send held back on a dead link must not advance the command: once the link is back,
        # the servos would jump to it.
        for joint in self._send(step):
            self.command[joint] = step[joint]

    def _send(self, goals: dict[str, float]) -> dict[str, float]:
        # The backend clamps to its limits before sending. A joint that starts outside its range (an arm
        # folded at rest) would jump to the limit at servo speed; widened to where the command stands,
        # it is led back by the ramp instead.
        widened: dict[str, tuple[float, float]] = {}
        for joint, value in goals.items():
            low, high = self.limits[joint]
            widened[joint] = (min(low, value), max(high, value))
        self.arm.set_joint_limits(widened)
        return self.arm.send_joints(goals)


_OPERATOR_MOTION_COMMANDS = (
    "jog", "target", "home", "stop", "torque", "speed",
    "cartesian/start", "cartesian/renew", "cartesian/stop",
)
_OPERATOR_COMMANDS = (
    "control/claim", "control/heartbeat", "control/release", "emergency-stop", "emergency-stop/reset",
    "cameras", "view", *_OPERATOR_MOTION_COMMANDS,
)
_MODEL_COMMANDS = ("model/arm", "model/disarm", "targets")
_DEFAULT_DASHBOARD_URL = "http://127.0.0.1:8780/"


class PanelServer(ThreadingHTTPServer):
    """The JSON API of one PanelController and a link to the operator dashboard."""

    daemon_threads = True

    def __init__(self, address: tuple[str, int], controller: PanelController,
                 cameras: CameraManager | None = None, viewer: PoseViewer | None = None, *,
                 dashboard_url: str = _DEFAULT_DASHBOARD_URL) -> None:
        self.controller = controller
        self.cameras = cameras or CameraManager()
        if isinstance(controller, PanelController):
            controller._model_cameras = self.cameras.status
        self.viewer = viewer
        self.closing = threading.Event()
        self.dashboard_url = dashboard_url
        super().__init__(address, PanelHandler)

    def state(self) -> dict[str, Any]:
        snapshot = self.controller.state()
        snapshot["cameras"] = self.cameras.status()
        snapshot["capabilities"] = {"model_control": not getattr(self.controller, "preview_only", False),
                                    "model_control_version": 1,
                                    "operator_commands": [f"/api/{name}" for name in _OPERATOR_COMMANDS],
                                    "atomic_targets": True,
                                    "supervisor": "machinekind_twin.safety.SafetySupervisor",
                                    "model_max_duration_s": MODEL_MAX_DURATION_S,
                                    "model_max_speed": MODEL_MAX_SPEED,
                                    "model_action_timeout_s": MODEL_ACTION_TIMEOUT_S,
                                    "model_observation_max_age_s": MODEL_OBSERVATION_MAX_AGE_S,
                                    "model_camera_max_age_s": MODEL_CAMERA_MAX_AGE_S,
                                    "model_max_tracking_deg": MODEL_MAX_TRACKING_DEG}
        snapshot["model_readiness"] = self.controller.model_readiness()
        snapshot.setdefault("model_session", {"active": False, "expires_in_s": 0.0,
                                               "speed": 0.0, "last_action_age_s": None})
        snapshot["viewer"] = self.viewer.status() if self.viewer else {
            "available": False, "port": None, "error": "The 3D viewer is disabled.", "selected_joint": None}
        return snapshot

    def server_close(self) -> None:
        self.closing.set()
        super().server_close()
        self.cameras.close()
        if self.viewer:
            self.viewer.close()


class PanelHandler(BaseHTTPRequestHandler):
    def do_GET(self) -> None:
        assert isinstance(self.server, PanelServer)
        path = urlsplit(self.path).path
        if path == "/":
            self.send_response(HTTPStatus.TEMPORARY_REDIRECT)
            self.send_header("Location", self.server.dashboard_url)
            self.send_header("Cache-Control", "no-store")
            self.send_header("Content-Length", "0")
            self.end_headers()
        elif path in ("/api/state", "/api/telemetry"):
            self._answer(HTTPStatus.OK, self.server.state())
        elif path in ("/cameras/overhead.mjpg", "/cameras/wrist.mjpg", "/cameras/observer.mjpg"):
            self._stream(path.split("/")[-1].split(".")[0])
        elif path in ("/cameras/overhead.jpg", "/cameras/wrist.jpg", "/cameras/observer.jpg"):
            frame = self.server.cameras.jpeg(path.split("/")[-1].split(".")[0])
            if frame:
                self._answer(HTTPStatus.OK, frame[1], "image/jpeg")
            else:
                self._answer(HTTPStatus.SERVICE_UNAVAILABLE, {"error": "No fresh camera frame"})
        else:
            self._answer(HTTPStatus.NOT_FOUND, {"error": f"not found: {path}"})

    def do_POST(self) -> None:
        assert isinstance(self.server, PanelServer)
        # A cross-site page may send a form or text/plain POST without asking; a JSON one needs a CORS
        # preflight this server never grants. So no other website in the browser can move the arm.
        if self.headers.get_content_type() != "application/json":
            self._answer(HTTPStatus.UNSUPPORTED_MEDIA_TYPE, {"error": "expected application/json"})
            return

        try:
            length = int(self.headers.get("Content-Length") or 0)
            if length < 0 or length > 65536:
                self._answer(HTTPStatus.REQUEST_ENTITY_TOO_LARGE, {"error": "Request body is too large"})
                return
            body = json.loads(self.rfile.read(length)) if length > 0 else {}
        except ValueError as exc:
            self._answer(HTTPStatus.BAD_REQUEST, {"error": f"Bad JSON: {exc}"})
            return

        try:
            command = urlsplit(self.path).path.removeprefix("/api/")
            if not isinstance(body, dict):
                raise PanelError("Expected a JSON object.")
            if command == "cameras":
                controller = self.server.controller
                with controller._lock:
                    controller._authorize(_arg(body, "token", str))
                    if controller._model_session is not None or controller._camera_change_in_progress:
                        raise PanelError("Disarm the model session before changing cameras.",
                                         HTTPStatus.CONFLICT)
                    controller._camera_change_in_progress = True
                # Capture setup cannot hold the control-loop lock.
                try:
                    observer = self.server.cameras.status().get("observer", {}).get("source")
                    assignments = (body.get("overhead"), body.get("wrist"))
                    if "observer" in body or observer is not None:
                        self.server.cameras.configure(*assignments, body.get("observer", observer))
                    else:
                        self.server.cameras.configure(*assignments)
                finally:
                    with controller._lock:
                        controller._camera_change_in_progress = False
                with controller._lock:
                    controller._event("info", "Camera assignments updated.")
                result = None
            elif command == "view":
                if self.server.viewer is None:
                    raise PanelError("The 3D viewer is disabled.", HTTPStatus.SERVICE_UNAVAILABLE)
                self.server.viewer.select_joint(_arg(body, "joint", str))
                result = None
            else:
                result = run_command(self.server.controller, command, body)
        except PanelError as exc:
            self._answer(exc.status, {"error": str(exc)})
        except (ValueError, RuntimeError) as exc:
            self._answer(HTTPStatus.BAD_REQUEST, {"error": str(exc)})
        else:
            self._answer(HTTPStatus.OK, result or {})

    def _stream(self, name: str) -> None:
        assert isinstance(self.server, PanelServer)
        self.connection.settimeout(3.0)
        self.send_response(HTTPStatus.OK)
        self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
        self.send_header("Cache-Control", "no-store")
        self.send_header("Connection", "close")
        self.end_headers()
        sequence = None
        try:
            while not self.server.closing.is_set():
                frame = self.server.cameras.jpeg(name)
                if frame is not None and frame[0] != sequence:
                    sequence, data = frame
                    self.wfile.write((f"--frame\r\nContent-Type: image/jpeg\r\n"
                                      f"Content-Length: {len(data)}\r\n\r\n").encode() + data + b"\r\n")
                    self.wfile.flush()
                self.server.closing.wait(0.04)
        except (OSError, ConnectionError):
            pass
        self.close_connection = True

    def log_message(self, format: str, *args: Any) -> None:
        logger.debug("%s - " + format, self.address_string(), *args)

    def _answer(self, status: HTTPStatus, payload: object, content_type: str = "application/json") -> None:
        """Sends `payload`: bytes as they are, anything else as JSON."""
        body = payload if isinstance(payload, bytes) else json.dumps(payload).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        try:
            self.wfile.write(body)
        except (BrokenPipeError, ConnectionResetError, ConnectionAbortedError):
            self.close_connection = True


def run_command(controller: PanelController, command: str, body: Any) -> dict[str, Any] | None:
    """Runs `POST /api/<command>` with its JSON body. Raises PanelError for a refused or unknown one."""
    if not isinstance(body, dict):
        raise PanelError("Expected a JSON object.")

    if command == "emergency-stop":
        return {"emergency_stop": controller.emergency_stop()}
    if command == "emergency-stop/reset":
        generation = _arg(body, "emergency_stop_generation", int)
        return {"emergency_stop": controller.reset_emergency_stop(generation)}

    # Authorization and mutation share the controller lock, including nested direct methods.
    with controller._lock:
        if command != "stop":
            controller._require_emergency_stop_reset()
            if getattr(controller, "preview_only", False):
                raise PanelError("Arm control is unavailable in disconnected preview.", HTTPStatus.CONFLICT)
        if command == "control/claim":
            generation = _arg(body, "emergency_stop_generation", int)
            if generation != controller._emergency_stop.generation:
                raise PanelError("Emergency stop invalidated this task. Start a new task after reset.",
                                 HTTPStatus.CONFLICT)
            return controller.claim(_arg(body, "label", str))
        if command == "control/heartbeat":
            controller.heartbeat(_arg(body, "token", str))
            return None
        if command == "control/release":
            controller.release(_arg(body, "token", str))
            return None
        if command not in _OPERATOR_MOTION_COMMANDS and command not in _MODEL_COMMANDS:
            raise PanelError(f"Unknown command: {command}", HTTPStatus.NOT_FOUND)
        if command != "stop":
            token = body.get("token")
            if not isinstance(token, str):
                raise PanelError("A live control token is required.", HTTPStatus.FORBIDDEN)
            controller._authorize(token)
        if command == "model/arm":
            return controller.arm_model(_arg(body, "duration_s", float), _arg(body, "speed", float))
        if command == "model/disarm":
            controller.disarm_model()
            return None
        if command == "targets":
            controller.set_model_targets(_arg(body, "session_id", str), body.get("targets"),
                                         body.get("observation_sequence"))
            return None
        if getattr(controller, "_model_session", None) is not None and command != "stop":
            raise PanelError("Disarm the model session before manual commands.", HTTPStatus.CONFLICT)
        if command == "cartesian/start":
            return controller.start_cartesian(_arg(body, "direction", str), body.get("profile", "slow"),
                                              _arg(body, "sequence", int),
                                              _arg(body, "observation_sequence", int),
                                              body.get("mode", "normal"))
        if command == "cartesian/renew":
            return controller.renew_cartesian(_arg(body, "motion_id", str), _arg(body, "sequence", int))
        if command == "cartesian/stop":
            motion_id = _arg(body, "motion_id", str) if "motion_id" in body else None
            return controller.stop_cartesian(motion_id, _arg(body, "sequence", int))
        if (getattr(controller, "_cartesian_motion", None) is not None
                and command not in ("stop", "torque")):
            raise PanelError("Release hold-jog before another manual command.", HTTPStatus.CONFLICT)
        _run_authorized_command(controller, command, body)
    return None


def _run_authorized_command(controller: PanelController, command: str, body: dict[str, Any]) -> None:

    if command == "jog":
        controller.jog(_arg(body, "joint", str), _arg(body, "delta", float))
    elif command == "target":
        controller.set_target(_arg(body, "joint", str), _arg(body, "value", float))
    elif command == "home":
        controller.home()
    elif command == "stop":
        controller.stop()
    elif command == "torque":
        controller.set_torque(_arg(body, "on", bool))
    elif command == "speed":
        controller.set_speed(_arg(body, "value", float))
    else:
        raise PanelError(f"Unknown command: {command}", HTTPStatus.NOT_FOUND)


def _arg(body: dict[str, Any], key: str, kind: type) -> Any:
    """The field `key` of a command's body, checked to be a `kind`: str, bool or a finite float."""
    value = body.get(key)
    if kind is float:
        # A JSON number arrives as int or float; true and false are ints to Python, not numbers here.
        if isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value):
            return float(value)
    elif kind is int:
        if type(value) is int:
            return value
    elif isinstance(value, kind):
        return value
    raise PanelError(f"Bad field `{key}`: {value!r} (expected {kind.__name__}).")


def detect_port(ports: Iterable[Any]) -> str:
    """The one USB-serial device among `ports` (from `serial.tools.list_ports.comports()`).

    Raises ValueError when there is none or more than one; the message says to pass `--port`.
    """
    devices = sorted(str(port.device) for port in ports if getattr(port, "vid", None) is not None)
    if not devices:
        raise ValueError("No USB-serial device found. Plug the arm in, or pass --port.")
    if len(devices) > 1:
        raise ValueError(f"Several USB-serial devices: {', '.join(devices)}. Pass --port.")
    return devices[0]


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(prog="soarm-panel", description="Run the SO-101 controller API.")
    parser.add_argument("--port", default=None,
                        help="the arm's serial port, e.g. COM5, /dev/ttyACM0 or socket://host:5555 "
                             "(default: the one USB-serial device)")
    parser.add_argument("--baudrate", type=int, default=1_000_000, help="baud rate (default 1 Mbaud)")
    parser.add_argument("--listen", default="127.0.0.1:8765",
                        help="where to listen; default `127.0.0.1:8765`, this machine only")
    parser.add_argument("--robot", type=Path, default=ROBOT_DIR,
                        help="the folder with robot.yaml (default: the SO-101 in the package)")
    parser.add_argument("--overhead-camera", type=parse_camera_source, default=None,
                        help="overhead camera index or Linux /dev/v4l/by-id path")
    parser.add_argument("--wrist-camera", type=parse_camera_source, default=None,
                        help="wrist camera index or Linux /dev/v4l/by-id path")
    parser.add_argument("--observer-camera", type=parse_camera_source, default=None,
                        help="optional observer camera index or Linux /dev/v4l/by-id path")
    parser.add_argument("--preview", action="store_true",
                        help="serve cameras and the viewer without opening or energizing an arm")
    parser.add_argument("--viewer-port", type=int, default=8766, help="read-only 3D viewer port")
    parser.add_argument("--no-viewer", action="store_true", help="disable the optional 3D viewer")
    parser.add_argument("--dashboard-url",
                        help="operator dashboard URL (default: the listen host on port 8780)")
    parser.add_argument("--no-browser", action="store_true", help="do not open the dashboard in a browser")
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)-7s %(message)s",
                        datefmt="%H:%M:%S")

    host, tcp_port = split_listen(args.listen)
    shown_host = "127.0.0.1" if host in ("0.0.0.0", "") else host
    dashboard_url = args.dashboard_url or f"http://{shown_host}:8780/"
    if any(ord(character) < 32 for character in dashboard_url):
        logger.error("The dashboard URL must not contain control characters.")
        return 2
    try:
        dashboard_address = urlsplit(dashboard_url)
        if dashboard_address.scheme not in ("http", "https") or not dashboard_address.hostname:
            raise ValueError("expected an absolute HTTP or HTTPS URL")
    except ValueError as exc:
        logger.error("Invalid dashboard URL: %s", exc)
        return 2
    if host not in ("127.0.0.1", "localhost"):
        logger.warning("The panel listens on %s. Anyone who can reach this port can move the arm.", host)

    # Validate camera configuration before touching the arm.
    arm = controller = server = None
    cameras = viewer = None
    try:
        if args.observer_camera is None:
            cameras = CameraManager(args.overhead_camera, args.wrist_camera)
        else:
            cameras = CameraManager(args.overhead_camera, args.wrist_camera, args.observer_camera)
        if not 1 <= args.viewer_port <= 65535 or args.viewer_port == tcp_port:
            raise ValueError("The viewer needs a distinct TCP port between 1 and 65535.")
        description = RobotDescription.load(args.robot)
        stop_path = os.environ.get("RAMION_STOP_STATE_FILE")
        stop_state = EmergencyStopState.load(Path(stop_path) if stop_path else None)
        gripper_range_q = None
        if args.preview:
            from .panel_preview import PreviewController

            controller = PreviewController(description, stop_state=stop_state)
        else:
            if stop_state.latched:
                raise RuntimeError("Emergency stop is latched. Start preview and explicitly reset before "
                                   "connecting the arm.")
            port = args.port
            if port is None:
                from serial.tools.list_ports import comports

                port = detect_port(comports())
                logger.info("Port: %s", port)
            stop_state.hardware_session = True
            stop_state.persist(latched=False)
            arm = FeetechArm(description.context(port, {"baudrate": args.baudrate}))
            arm.connect()
            controller = PanelController(arm, description, stop_state=stop_state)
            gripper_range_q = arm.gripper_range_q()
        if not args.no_viewer:
            viewer = PoseViewer(args.robot, controller.state, host, args.viewer_port,
                                gripper_range_q=gripper_range_q)
        server = PanelServer((host, tcp_port), controller, cameras, viewer, dashboard_url=dashboard_url)
    except (RuntimeError, ValueError, OSError) as exc:
        logger.error("The panel did not start: %s", exc)
        if server:
            server.server_close()
        if controller:
            controller.close()
        if arm:
            arm.disconnect()
        return 2

    controller.start()
    cameras.start()
    if viewer:
        viewer.start()
        if not viewer.status()["available"]:
            logger.warning("%s", viewer.status()["error"])
    mode = "disconnected preview" if args.preview else "the arm keeps torque on"
    logger.info("Controller API: http://%s:%s/ (Ctrl-C ends it; %s)", shown_host, server.server_port, mode)
    logger.info("Operator dashboard: %s", dashboard_url)
    if not args.no_browser:
        webbrowser.open(dashboard_url)

    try:
        server.serve_forever()
    except KeyboardInterrupt:
        logger.info("Stopped.")
    finally:
        controller.close()
        server.server_close()
        # FeetechArm.disconnect leaves torque on by default: a limp arm would drop.
        if arm is not None:
            arm.disconnect()
    return 0
