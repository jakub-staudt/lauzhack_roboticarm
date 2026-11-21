"""Disconnected workbench controller: cameras/UI without a serial backend."""

from __future__ import annotations

import os
import threading
from http import HTTPStatus
from typing import Any, NoReturn

from .panel import DEFAULT_SPEED, EmergencyStopState, PanelController, PanelError, RobotDescription
from .panel_cartesian import PROFILES, WATCHDOG_S, Workspace


class PreviewController(PanelController):
    """Expose description metadata without inventing any robot measurements.

    Deliberately does not call PanelController.__init__: no arm, bus or control
    loop is constructed. Every control entry point rejects except inert STOP.
    """

    preview_only = True

    def __init__(self, description: RobotDescription, *,
                 stop_state: EmergencyStopState | None = None) -> None:
        self._emergency_stop = stop_state if stop_state is not None else EmergencyStopState()
        self.description = description
        self._lock = threading.RLock()
        self._events: list[dict[str, Any]] = []
        self._cartesian_workspace = Workspace.from_environment(os.environ)

    def state(self) -> dict[str, Any]:
        with self._lock:
            return {
                "preview_only": True,
                "robot": self.description.name,
                "port": None,
                "joints": list(self.description.joints),
                "gripper": self.description.gripper,
                "limits": dict(self.description.limits),
                "measured": {},
                "command": {},
                "target": {},
                "torque": None,
                "speed": DEFAULT_SPEED,
                "clip": None,
                "faults": ["Disconnected preview: no arm connected."],
                "loop_hz": 0,
                "age_s": None,
                "sequence": 0,
                "loop_ok": False,
                "connected": False,
                "control": {"owner_id": None, "label": None, "expires_in_s": 0.0},
                "sample": {"age_s": None, "sequence": 0, "loop_ok": False, "connected": False},
                "events": [dict(event) for event in self._events],
                "emergency_stop": self._emergency_stop.snapshot(),
                "cartesian": {"active": False, "motion_id": None, "direction": None, "profile": "slow",
                              "sequence": -1, "expires_in_s": 0.0, "watchdog_s": WATCHDOG_S,
                              "available": False, "spatial_available": False,
                              "orientation": "fixed_tilt",
                              "orientation_controls": {
                                  "supported": False,
                                  "tilt": {"supported": False, "available": False,
                                           "reason": "Disconnected preview has no arm connection."},
                                  "roll": {"supported": False, "available": False,
                                           "reason": "Disconnected preview has no arm connection."},
                                  "angular_rate_key": "angular_deg_s", "joint_rate_key": "joint_deg_s",
                                  "tilt_tip_compensated": False, "roll_tip_compensated": False,
                                  "tilt_view": "tip_up_down", "tilt_up_joint_sign": -1,
                                  "tilt_down_joint_sign": 1,
                                  "roll_view": "from_wrist_toward_tip", "roll_left_joint_sign": 1,
                                  "roll_right_joint_sign": -1},
                              "mode": "normal", "phase": None,
                              "stop_reason": None,
                              "stopped_motion_id": None,
                              "floor_reentry_active": False,
                              "recovery": {"supported": False, "max_soft_violation_deg": 1.0,
                                           "max_tip_drift_m": 0.002, "max_pitch_change_deg": 3.0,
                                           "phase": None, "pitch_change_deg": None, "tip_drift_m": None,
                                           "reason": "Disconnected preview has no arm connection."},
                              "joint_limits": {"software": dict(self.description.limits), "hardware": {},
                                               "effective": dict(self.description.limits)},
                              "ranges": {"status": "unavailable", "orientation": "fixed_tilt",
                                         "directions": {}, "age_s": None},
                              "reason": "Disconnected preview has no arm connection.",
                              "workspace": self._cartesian_workspace.snapshot(),
                              "profiles": {key: dict(value) for key, value in PROFILES.items()}},
            }

    def start(self) -> None:
        pass

    def close(self) -> None:
        pass

    def stop(self) -> None:
        """An inert STOP keeps the common UI usable without suggesting motion."""

    def _apply_emergency_hold(self) -> None:
        self._emergency_stop.hold_status = "unavailable"
        self._emergency_stop.hold_error = "Disconnected preview has no arm connection."

    def _healthy(self) -> bool:
        return False

    @staticmethod
    def _unavailable() -> NoReturn:
        raise PanelError("Arm control is unavailable in disconnected preview.", HTTPStatus.CONFLICT)

    def claim(self, label: str) -> dict[str, str]:
        self._unavailable()

    def _authorize(self, token: str) -> None:
        self._unavailable()

    def heartbeat(self, token: str) -> None:
        self._unavailable()

    def release(self, token: str) -> None:
        self._unavailable()

    def jog(self, joint: str, delta: float) -> None:
        self._unavailable()

    def set_target(self, joint: str, value: float) -> None:
        self._unavailable()

    def home(self) -> None:
        self._unavailable()

    def set_torque(self, on: bool) -> None:
        self._unavailable()

    def set_speed(self, value: float) -> None:
        self._unavailable()

    def start_cartesian(self, direction: str, profile: str, sequence: int,
                        observation_sequence: int, mode: str = "normal") -> dict[str, Any]:
        self._unavailable()

    def renew_cartesian(self, motion_id: str, sequence: int) -> dict[str, Any]:
        self._unavailable()

    def stop_cartesian(self, motion_id: str | None, sequence: int) -> dict[str, Any]:
        self._unavailable()
