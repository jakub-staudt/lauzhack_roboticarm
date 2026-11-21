"""Cached observations and explicitly armed sessions through the workbench's HTTP API.

This module never opens a serial port or camera device. Model calls belong outside the
control transport; a stalled notebook is stopped by the workbench's independent watchdog.
"""

from __future__ import annotations

import io
import json
import math
import threading
import time
from collections.abc import Callable, Iterable, Mapping
from dataclasses import dataclass
from typing import Any
from urllib.error import HTTPError, URLError
from urllib.parse import urlparse
from urllib.request import Request, urlopen

import numpy as np

JOINTS = ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper")
CAMERAS = ("overhead", "wrist")
ARM_CONFIRMATION = "ARM ROBOT"
DEFAULT_URL = "http://100.85.56.14:8765"


class WorkbenchError(RuntimeError):
    """A refused request, stale observation, or unavailable control feature."""


@dataclass
class Snapshot:
    state: dict[str, Any]
    images: dict[str, np.ndarray]
    ages: dict[str, float]
    captured_at: float

    @property
    def joints(self) -> dict[str, float]:
        return dict(self.state["measured"])

    @property
    def sequence(self) -> int:
        return int(self.state["sequence"])

    def require_fresh(self, max_age_s: float = 1.0) -> None:
        elapsed = max(0.0, time.monotonic() - self.captured_at)
        if not self.state.get("connected") or not self.state.get("loop_ok") or self.state.get("faults"):
            raise WorkbenchError("Robot feedback is disconnected or unhealthy.")
        age = float(self.state.get("age_s", math.inf))
        if not math.isfinite(age) or age < 0 or age + elapsed > max_age_s:
            raise WorkbenchError("Robot feedback is stale; take another snapshot.")
        for role in CAMERAS:
            age = self.ages.get(role, math.inf)
            if role not in self.images or not math.isfinite(age) or age < 0 or age + elapsed > max_age_s:
                raise WorkbenchError(f"The {role} camera is missing or stale.")
        validate_targets(self.joints)


def validate_targets(targets: Mapping[str, float], limits: Mapping | None = None) -> dict[str, float]:
    """Reject invalid actions instead of silently clipping or completing partial vectors."""
    if set(targets) != set(JOINTS):
        raise ValueError(f"An action must contain exactly these six joints: {JOINTS}.")
    result = {}
    for name in JOINTS:
        value = targets[name]
        if isinstance(value, (bool, str)) or not isinstance(value, (int, float, np.number)):
            raise ValueError(f"{name} must be a finite number.")
        value = float(value)
        if not math.isfinite(value):
            raise ValueError(f"{name} must be a finite number.")
        if limits is not None and not limits[name][0] <= value <= limits[name][1]:
            raise ValueError(f"{name}={value:.3f} is outside workbench limits {limits[name]}.")
        result[name] = value
    return result


def _require_emergency_stop_clear(state: Mapping[str, Any], expected: int | None = None) -> int | None:
    stop = state.get("emergency_stop", {})
    if stop.get("latched"):
        raise WorkbenchError("Emergency stop is latched. Explicitly reset it before starting a new task.")
    generation = stop.get("generation")
    if expected is not None and generation != expected:
        raise WorkbenchError("Emergency stop invalidated this task. Start a new task after reset.")
    return generation if type(generation) is int else None


class WorkbenchClient:
    def __init__(self, base_url: str = DEFAULT_URL, timeout_s: float = 2.0):
        parsed = urlparse(base_url)
        if parsed.scheme not in ("http", "https") or not parsed.netloc or parsed.username or parsed.password:
            raise ValueError("Use an HTTP(S) workbench URL without embedded credentials.")
        self.base_url = base_url.rstrip("/")
        self.timeout_s = timeout_s

    def _request(self, path: str, body: dict | None = None, *, raw: bool = False) -> Any:
        payload = None if body is None else json.dumps(body, allow_nan=False).encode()
        request = Request(self.base_url + path, data=payload,
                          headers={"Content-Type": "application/json"} if payload is not None else {})
        try:
            with urlopen(request, timeout=self.timeout_s) as response:
                data = response.read()
        except HTTPError as exc:
            try:
                message = json.loads(exc.read()).get("error", str(exc.code))
            except (ValueError, AttributeError):
                message = str(exc.code)
            raise WorkbenchError(f"Workbench refused {path}: {message}") from None
        except (URLError, TimeoutError, OSError) as exc:
            raise WorkbenchError(f"Workbench unavailable at {self.base_url}: {type(exc).__name__}") from None
        return data if raw else json.loads(data)

    def state(self) -> dict[str, Any]:
        return self._request("/api/state")

    def snapshot(self) -> Snapshot:
        from PIL import Image

        started = time.monotonic()
        state = self.state()
        images, ages = {}, {}
        for role in CAMERAS:
            status = state.get("cameras", {}).get(role, {})
            if status.get("status") != "live":
                error = status.get("error") or "unavailable"
                raise WorkbenchError(f"The {role} camera is not live: {error}.")
            data = self._request(f"/cameras/{role}.jpg", raw=True)
            with Image.open(io.BytesIO(data)) as image:
                images[role] = np.array(image.convert("RGB"))
            ages[role] = float(status.get("age_s", math.inf))
        # Ages refer to the start of acquisition, conservatively including all HTTP latency.
        result = Snapshot(state, images, ages, started)
        result.require_fresh()
        return result

    def preflight(self) -> dict[str, Any]:
        snapshot = self.snapshot()
        state = snapshot.state
        return {
            "robot": state.get("robot"), "connected": state.get("connected"),
            "cameras": {k: list(v.shape) for k, v in snapshot.images.items()},
            "joints": snapshot.joints, "limits": state["limits"],
            "owner": state.get("control", {}).get("label"),
            "model_control": state.get("capabilities", {}),
            "emergency_stop": state.get("emergency_stop"),
            "note": "Read-only check; no control was claimed and no target was sent.",
        }

    @staticmethod
    def show(snapshot: Snapshot) -> None:
        from IPython.display import display
        from PIL import Image

        for role, pixels in snapshot.images.items():
            print(role)
            display(Image.fromarray(pixels))

    def stop(self) -> None:
        """Latch the station's software emergency stop and revoke all current control."""
        self._request("/api/emergency-stop", {})

    def reset_emergency_stop(self, generation: int | None = None) -> None:
        """Reset the observed latch generation. Previously revoked tasks remain invalid."""
        if generation is None:
            generation = self.state().get("emergency_stop", {}).get("generation")
        if type(generation) is not int:
            raise WorkbenchError("Read the current emergency stop state before resetting.")
        self._request("/api/emergency-stop/reset", {"emergency_stop_generation": generation})

    def run_targets(self, targets: Iterable[Mapping[str, float]], *, hz: float = 20,
                    confirmation: str = "", seconds: float = 15, speed: float = 10,
                    stop_event: threading.Event | None = None) -> list[dict]:
        if confirmation != ARM_CONFIRMATION:
            raise WorkbenchError("Motion is disabled. In a supervised session, "
                                 "enter confirmation='ARM ROBOT'.")
        rows = [validate_targets(row) for row in targets]
        if not rows:
            raise ValueError("No actions to execute.")
        if not math.isfinite(hz) or hz <= 0 or len(rows) / hz > seconds:
            raise ValueError("The complete trajectory must fit in the requested episode duration.")
        snapshot = self.snapshot()
        generation = _require_emergency_stop_clear(snapshot.state)
        for row in rows:
            validate_targets(row, snapshot.state["limits"])
        if any(abs(rows[0][joint] - snapshot.joints[joint]) > 2 for joint in JOINTS):
            raise WorkbenchError("The arm moved since trajectory preparation. Review the scene and rebuild "
                                 "the trajectory from a fresh measured pose before arming.")
        iterator = iter(rows)

        def step(snapshot: Snapshot) -> dict[str, float]:
            return next(iterator)

        return self.run_policy(step, hz=hz, confirmation=confirmation, seconds=seconds,
                               speed=speed, stop_event=stop_event, expected_start=rows[0],
                               _expected_emergency_stop_generation=generation)

    def run_policy(self, step: Callable[[Snapshot], Mapping[str, float] | None], *, hz: float = 20,
                   confirmation: str = "", seconds: float = 15, speed: float = 10,
                   stop_event: threading.Event | None = None,
                   expected_start: Mapping[str, float] | None = None,
                   _expected_emergency_stop_generation: int | None = None) -> list[dict]:
        """Run a nonblocking policy callback in a bounded, explicitly armed session.

        A callback returning None holds the measured pose. Exceptions/interrupts release
        ownership. No background heartbeat can keep the robot moving after this loop stalls.
        The caller must load models and finish the inference preview before entering here.
        """
        if confirmation != ARM_CONFIRMATION:
            raise WorkbenchError("Motion is disabled. In a supervised session, "
                                 "enter confirmation='ARM ROBOT'.")
        if not math.isfinite(seconds) or not 0 < seconds <= 60:
            raise ValueError("Episode duration must be between 0 and 60 seconds.")
        if not math.isfinite(hz) or not 5 <= hz <= 30:
            raise ValueError("Use a control rate between 5 and 30 Hz.")
        if not math.isfinite(speed) or not 0 < speed <= 15:
            raise ValueError("Use a speed above 0 and at most 15 degrees/s.")
        if stop_event is not None and stop_event.is_set():
            return []
        snapshot = self.snapshot()
        generation = _require_emergency_stop_clear(snapshot.state, _expected_emergency_stop_generation)
        if expected_start is not None:
            expected_start = validate_targets(expected_start)
            if any(abs(expected_start[j] - snapshot.joints[j]) > 2 for j in JOINTS):
                raise WorkbenchError("The arm moved since trajectory preparation; rebuild it before arming.")
        capabilities = snapshot.state.get("capabilities", {})
        if not capabilities.get("model_control"):
            raise WorkbenchError("The running workbench lacks the notebook control API. Deploy the updated "
                                 "source during the supervised handoff; inference previews work now.")
        if not snapshot.state.get("torque"):
            raise WorkbenchError("Torque is off. Prepare the arm through the supervised workbench UI.")
        if stop_event is not None and stop_event.is_set():
            return []
        claim: dict[str, str | int] = {"label": "Notebook model session"}
        if generation is not None:
            claim["emergency_stop_generation"] = generation
        token = self._request("/api/control/claim", claim)["token"]
        records: list[dict] = []
        try:
            armed = self._request("/api/model/arm", {"token": token, "duration_s": seconds, "speed": speed})
            session_id = armed["session_id"]
            started = time.monotonic()
            deadline = started + seconds
            next_tick = started
            while time.monotonic() < deadline and not (stop_event and stop_event.is_set()):
                snapshot = self.snapshot()
                _require_emergency_stop_clear(snapshot.state, generation)
                if not records and expected_start is not None:
                    if any(abs(expected_start[j] - snapshot.joints[j]) > 2 for j in JOINTS):
                        raise WorkbenchError("The arm moved after arming; discard the stale trajectory.")
                try:
                    target = step(snapshot)
                except StopIteration:
                    break
                snapshot.require_fresh()
                if time.monotonic() >= deadline or (stop_event and stop_event.is_set()):
                    break
                target = validate_targets(snapshot.joints if target is None else target,
                                          snapshot.state["limits"])
                self._request("/api/control/heartbeat", {"token": token})
                self._request("/api/targets", {"token": token, "session_id": session_id,
                                              "targets": target, "observation_sequence": snapshot.sequence})
                records.append({"elapsed_s": time.monotonic() - started,
                                "measured": snapshot.joints, "target": target})
                next_tick += 1 / hz
                remaining = max(0.0, min(next_tick, deadline) - time.monotonic())
                if stop_event is not None:
                    stop_event.wait(remaining)
                else:
                    time.sleep(remaining)
                # Do not burst old actions after a slow callback or HTTP request.
                next_tick = max(next_tick, time.monotonic())
            if generation is not None:
                # A blocked callback can return after the deadline without attempting another action.
                _require_emergency_stop_clear(self.state(), generation)
        except WorkbenchError:
            if generation is not None:
                # If STOP was reset meanwhile, the revoked token still fails. Report why.
                try:
                    current = self.state()
                except WorkbenchError:
                    pass
                else:
                    _require_emergency_stop_clear(current, generation)
            raise
        finally:
            try:
                self._request("/api/control/release", {"token": token})
            except WorkbenchError:
                # STOP/watchdog can already have revoked this token. Never stop a new owner.
                pass
        return records
