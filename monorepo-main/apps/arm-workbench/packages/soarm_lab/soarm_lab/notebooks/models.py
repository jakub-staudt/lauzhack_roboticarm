"""Inference adapters for notebooks; this module never opens robot or camera devices.

SmolVLA's checkpoint units are deliberately separate from workbench degrees. Without
a task-trained checkpoint and an explicit mapping it is an inference benchmark only.
Dream reuses the repository's experimental single-arm adapter, including its dummy
left arm/camera. The pinned Dream client can disable its additional recorder with
``_record=False``; verify that private interface before sending any observations.
"""

from __future__ import annotations

import importlib
import math
import threading
import time
from collections.abc import Callable, Mapping
from dataclasses import dataclass, field
from importlib.metadata import version
from typing import Any

import numpy as np

from .credentials import require_dream_key

JOINTS = ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper")
BASE_CHECKPOINT = "lerobot/smolvla_base"
DEFAULT_DREAM_CHECKPOINT = "dreamlabs/dream-1-beta-flash"
DREAM_CLIENT_VERSION = "1.0.14"


@dataclass
class InferenceResult:
    """Targets are degrees/gripper 0..100; raw_actions retain checkpoint units."""

    actions: list[dict[str, float]]
    elapsed_s: float
    executable: bool
    note: str
    raw_actions: np.ndarray | None = None
    done: bool = False
    metadata: dict[str, Any] = field(default_factory=dict)


@dataclass(frozen=True)
class JointMapping:
    """Explicit checkpoint conversion: policy = workbench * scale + offset.

    Order is ``JOINTS``. Obtain these coefficients from the training embodiment's
    calibration and units, never from guessed joint ranges. Identity is appropriate
    only for a checkpoint actually trained in workbench degrees and gripper 0..100.
    """

    scale: tuple[float, ...]
    offset: tuple[float, ...]

    def __post_init__(self) -> None:
        if len(self.scale) != 6 or len(self.offset) != 6:
            raise ValueError("joint mapping needs six scales and offsets in JOINTS order")
        if not all(math.isfinite(v) for v in (*self.scale, *self.offset)) or 0 in self.scale:
            raise ValueError("joint mapping must be finite with nonzero scales")

    def encode(self, joints: Mapping[str, float]) -> np.ndarray:
        return _joint_vector(joints) * np.asarray(self.scale) + np.asarray(self.offset)

    def decode(self, rows: np.ndarray) -> list[dict[str, float]]:
        values = (rows - np.asarray(self.offset)) / np.asarray(self.scale)
        if values.ndim != 2 or values.shape[1] != 6 or not np.isfinite(values).all():
            raise ValueError("policy actions must be finite rows of six values")
        return [dict(zip(JOINTS, map(float, row), strict=True)) for row in values]


def _joint_vector(joints: Mapping[str, float]) -> np.ndarray:
    if any(name not in joints for name in JOINTS):
        raise ValueError("observation must contain all six SO101 joints")
    values = np.asarray([joints[name] for name in JOINTS], dtype=np.float32)
    if not np.isfinite(values).all():
        raise ValueError("joint observation must be finite")
    return values


def _check_frames(images: Mapping[str, np.ndarray], names: set[str],
                  ages: Mapping[str, float] | None, max_age_s: float = 1.0) -> None:
    for name in names:
        if name not in images:
            raise ValueError(f"missing camera {name!r}")
        image = np.asarray(images[name])
        if image.dtype != np.uint8 or image.ndim != 3 or image.shape[2] != 3 or not image.size:
            raise ValueError(f"camera {name!r} must be a nonempty uint8 RGB image")
        if ages is None or name not in ages:
            raise ValueError(f"camera {name!r} needs a measured age")
        age = float(ages[name])
        if not math.isfinite(age) or not 0 <= age <= max_age_s:
            raise ValueError(f"camera {name!r} is stale or has an invalid age")


class SmolVLAInference:
    """Load a policy once, then infer action chunks on cached workbench camera frames.

    The base checkpoint is never executable. Without ``joint_mapping``, a zero state
    in checkpoint units is used to exercise inference with the real camera images;
    returned predictions are not predictions for the robot's measured joint state.
    The two physical views occupy camera1/camera2. Any unused configured camera is
    explicitly masked by SmolVLA's ``empty_cameras`` implementation.
    """

    def __init__(self, checkpoint: str = BASE_CHECKPOINT, *, device: str = "cuda",
                 camera_map: Mapping[str, str] | None = None, joint_mapping: JointMapping | None = None,
                 trained_for_robot: bool = False):
        self.checkpoint = checkpoint
        self.device = device
        self.camera_map = dict(camera_map or {
            "observation.images.camera1": "overhead", "observation.images.camera2": "wrist",
        })
        self.joint_mapping = joint_mapping
        self.trained_for_robot = trained_for_robot
        self.policy: Any = None
        self._preprocess: Any = None
        self._postprocess: Any = None
        self._torch: Any = None

    @property
    def executable(self) -> bool:
        base = self.checkpoint.rstrip("/").split("/")[-1] == "smolvla_base"
        return not base and self.trained_for_robot and self.joint_mapping is not None

    def load(self) -> SmolVLAInference:
        """Download/load weights and saved normalization processors; opens no hardware."""
        import torch
        from lerobot.policies.factory import make_pre_post_processors
        from lerobot.policies.smolvla.configuration_smolvla import SmolVLAConfig
        from lerobot.policies.smolvla.modeling_smolvla import SmolVLAPolicy

        if self.device.startswith("cuda") and not torch.cuda.is_available():
            raise RuntimeError("CUDA is unavailable in this kernel; select the SmolVLA notebook environment")
        config = SmolVLAConfig.from_pretrained(self.checkpoint)
        config.device = self.device
        config.load_vlm_weights = False  # The complete policy checkpoint already contains these weights.
        if tuple(config.input_features["observation.state"].shape) != (6,):
            raise ValueError("this notebook adapter requires a six-joint SO101 state checkpoint")
        if tuple(config.output_features["action"].shape) != (6,):
            raise ValueError("this notebook adapter requires six-joint action outputs")
        expected_cameras = set(config.image_features)
        if not self.camera_map or not set(self.camera_map) <= expected_cameras:
            raise ValueError(f"camera_map keys must be checkpoint image features: {sorted(expected_cameras)}")
        config.empty_cameras = len(expected_cameras - set(self.camera_map))
        self.policy = SmolVLAPolicy.from_pretrained(self.checkpoint, config=config).to(self.device).eval()
        self._preprocess, self._postprocess = make_pre_post_processors(
            self.policy.config, pretrained_path=self.checkpoint,
            preprocessor_overrides={"device_processor": {"device": self.device}},
            postprocessor_overrides={"device_processor": {"device": "cpu"}},
        )
        self._torch = torch
        return self

    def predict(self, joints: Mapping[str, float], images: Mapping[str, np.ndarray], prompt: str, *,
                ages: Mapping[str, float] | None = None) -> InferenceResult:
        """Return an inference preview; callers must enforce ``result.executable``."""
        _joint_vector(joints)
        _check_frames(images, set(self.camera_map.values()), ages)
        if not prompt.strip():
            raise ValueError("task prompt cannot be empty")
        if self.policy is None:
            raise RuntimeError("call load() before predict()")
        torch = self._torch
        state = (self.joint_mapping.encode(joints) if self.joint_mapping is not None
                 else np.zeros(6, dtype=np.float32))
        batch: dict[str, Any] = {"observation.state": torch.from_numpy(state.astype(np.float32)),
                                 "task": prompt}
        for feature, camera in self.camera_map.items():
            pixels = torch.from_numpy(np.array(images[camera], copy=True)).permute(2, 0, 1)
            # The pinned LeRobot release resizes before any dtype conversion.
            batch[feature] = pixels.to(dtype=torch.float32) / 255.0
        started = time.monotonic()
        self.policy.reset()
        with torch.inference_mode():
            prepared = self._preprocess(batch)
            normalized = self.policy.predict_action_chunk(prepared)
            actions = self._postprocess(normalized).detach().cpu().numpy()
        elapsed = time.monotonic() - started
        if actions.ndim == 3 and actions.shape[0] == 1:
            actions = actions[0]
        if actions.ndim != 2 or actions.shape[1] != 6 or not np.isfinite(actions).all():
            raise ValueError("SmolVLA returned an invalid action chunk")
        targets = self.joint_mapping.decode(actions) if self.joint_mapping is not None else []
        note = "Task-trained checkpoint with explicit joint-unit mapping."
        if not self.executable:
            note = "Inference preview only; no verified task-trained checkpoint and joint-unit mapping."
        if self.joint_mapping is None:
            note += " Zero synthetic state used; camera images are real. Raw actions are checkpoint units."
        return InferenceResult(targets, elapsed, self.executable, note, raw_actions=actions,
                               metadata={"checkpoint": self.checkpoint, "device": self.device,
                                         "state_source": "mapped" if self.joint_mapping else "synthetic_zero",
                                         "masked_cameras": self.policy.config.empty_cameras})

    def close(self) -> None:
        """Release the model so another notebook may use the GPU."""
        self.policy = self._preprocess = self._postprocess = None
        if self._torch is not None and self.device.startswith("cuda"):
            self._torch.cuda.empty_cache()


class _RequestBudgetExceeded(RuntimeError):
    pass


class _TimedDreamReply(dict):
    """Local provenance on a reply; the extra data never goes to Dream's service."""

    def __init__(self, reply: Mapping, observed_at: float, received_at: float):
        super().__init__(reply)
        self.observed_at = observed_at
        self.received_at = received_at


class _BudgetedDreamClient:
    def __init__(self, client: Any, *, allow_recording: bool, max_requests: int, max_seconds: float):
        self.client = client
        self.allow_recording = allow_recording
        self.max_requests = max_requests
        self.max_seconds = max_seconds
        self.requests = 0
        self.deadline = math.inf
        self.observation_age_s = 0.0
        self._observed_at: float | None = None
        self.last_error: dict[str, Any] | None = None
        self._lock = threading.Lock()

    def __getattr__(self, name: str) -> Any:
        return getattr(self.client, name)

    def policy(self, config: dict[str, Any]) -> None:
        self.client.policy({**config, "_record": self.allow_recording})
        if not self.allow_recording:
            stats = getattr(self.client, "_recording_stats", None)
            if not isinstance(stats, Mapping) or stats.get("state") != "off":
                self.client.close()
                raise RuntimeError("Dream recorder could not be verified off; no observations were sent")
        self.deadline = time.monotonic() + self.max_seconds

    def get_action(self, **kwargs: Any) -> Any:
        with self._lock:
            if self.requests >= self.max_requests or time.monotonic() >= self.deadline:
                raise _RequestBudgetExceeded("Dream inference request/time budget exhausted")
            observed_at = self._observed_at
            if observed_at is None:
                raise RuntimeError("Dream inference needs an observation before a request")
            self.requests += 1
        try:
            reply = self.client.get_action(**kwargs)
        except Exception as exc:
            message = str(exc)
            key = getattr(self.client, "_api_key", None)

            def redact(value: Any) -> Any:
                if isinstance(value, str) and isinstance(key, str) and key:
                    return value.replace(key, "[redacted]")
                return value

            self.last_error = {
                "type": type(exc).__name__, "message": redact(message)[:600],
                "code": redact(getattr(exc, "code", None)), "reason": redact(getattr(exc, "reason", None)),
                "retryable": bool(getattr(exc, "retryable", False)),
                "retry_after_s": getattr(exc, "retry_after_s", None),
            }
            raise
        self.last_error = None
        if isinstance(reply, Mapping):
            return _TimedDreamReply(reply, observed_at, time.monotonic())
        return reply

    def set_observation(self, *args: Any, **kwargs: Any) -> None:
        if time.monotonic() >= self.deadline:
            raise _RequestBudgetExceeded("Dream inference time budget exhausted")
        self.client.set_observation(*args, **kwargs)
        # A later observation may reach the SDK before its request thread samples
        # the window. Keeping this earlier origin is deliberately conservative.
        with self._lock:
            self._observed_at = time.monotonic() - self.observation_age_s


def dream_client(api_key: str) -> Any:
    """Bind the operator-supplied key directly to the pinned SDK."""
    key = require_dream_key(api_key)
    if version("dreamlabs-client") != DREAM_CLIENT_VERSION:
        raise RuntimeError(f"Dream notebooks require dreamlabs-client=={DREAM_CLIENT_VERSION} "
                           "to verify the recorder switch")
    if importlib.util.find_spec("dream_ik") is None:
        raise RuntimeError("SO101 inference requires the dreamlabs package including dream_ik")
    return importlib.import_module("dream").DreamClient(api_key=key)


class DreamInference:
    """Nonblocking DreamStrategy ticks for the existing single-arm experiment.

    Starting opens an inference policy, never hardware. Each step sends current
    observations to Dream and returns a proposed target in repository units. A
    separate runner must decide whether to send it to the workbench. Additional
    recorder uploads are disabled and checked before sending any frames by default.
    """

    def __init__(self, prompt: str, *, checkpoint: str = DEFAULT_DREAM_CHECKPOINT,
                 allow_recording: bool = False, api_key: str | None = None,
                 max_requests: int = 3, max_seconds: float = 30,
                 max_output_age_s: float = 3.0, max_speed: float = 8.0,
                 client_factory: Callable[[], Any] | None = None):
        if isinstance(max_requests, bool) or not isinstance(max_requests, int) or max_requests < 1:
            raise ValueError("max_requests must be a positive integer")
        if not math.isfinite(max_seconds) or max_seconds <= 0:
            raise ValueError("max_seconds must be positive and finite")
        if not math.isfinite(max_output_age_s) or max_output_age_s <= 0:
            raise ValueError("max_output_age_s must be positive and finite")
        from ..strategies.dream import DreamSettings, DreamStrategy

        if not math.isfinite(max_speed) or not 0 < max_speed <= 15:
            raise ValueError("max_speed must be positive and at most 15 units/s")
        self.max_speed = float(max_speed)
        self.max_seconds = float(max_seconds)
        self.max_output_age_s = float(max_output_age_s)
        self._candidate_timing: tuple[float, float] | None = None
        self._chunk_timing: tuple[float, float] | None = None
        self._preview_timing: tuple[float, float] | None = None
        self._raw_preview: np.ndarray | None = None
        if client_factory is None:
            key = require_dream_key(api_key)

            def client_factory() -> Any:
                return dream_client(key)
        self._client_factory = client_factory
        self._budget: _BudgetedDreamClient | None = None

        def factory() -> _BudgetedDreamClient:
            self._budget = _BudgetedDreamClient(self._client_factory(), allow_recording=allow_recording,
                                               max_requests=max_requests, max_seconds=max_seconds)
            return self._budget

        owner = self

        class TimedStrategy(DreamStrategy):
            def _rows(self, reply: Any) -> list[np.ndarray]:
                # This hook runs on the control thread when the existing strategy
                # collects a completed request, before any row can become a target.
                if not isinstance(reply, _TimedDreamReply):
                    raise ValueError("Dream reply has no observation provenance")
                rows = super()._rows(reply)
                owner._raw_preview = np.asarray(reply["q_r_chunk"], dtype=float).copy()
                owner._preview_timing = (reply.observed_at, reply.received_at)
                if time.monotonic() - reply.observed_at > owner.max_output_age_s:
                    raise ValueError("Dream reply is stale relative to its originating observation")
                owner._candidate_timing = (reply.observed_at, reply.received_at)
                return rows

        self.strategy = TimedStrategy(DreamSettings(
            prompt=prompt, checkpoint=checkpoint, scene_camera="overhead", wrist_camera="wrist",
            max_wait_s=max_seconds, max_step_deg=1.0, max_grip_step=3.0,
        ), client_factory=factory)
        self._started_at: float | None = None
        self._step = 0

    def _observation(self, joints: Mapping[str, float], images: Mapping[str, np.ndarray],
                     ages: Mapping[str, float] | None,
                     command: Mapping[str, float] | None = None) -> Any:
        from machinekind_twin_api.strategy import Frame, Observation

        _joint_vector(joints)
        _check_frames(images, {"overhead", "wrist"}, ages)
        assert ages is not None
        if command is not None:
            _joint_vector(command)
        return Observation(t=0.0 if self._started_at is None else time.monotonic() - self._started_at,
                           step=self._step, joints=dict(joints),
                           command=dict(joints if command is None else command),
                           frames={name: Frame(images[name], float(ages[name]))
                                   for name in ("overhead", "wrist")})

    def start(self, joints: Mapping[str, float], images: Mapping[str, np.ndarray], *,
              ages: Mapping[str, float] | None = None,
              limits: Mapping[str, tuple[float, float]] | None = None) -> None:
        from machinekind_twin_api.strategy import RunInfo

        import soarm

        from ..robot import load_robot

        if self.strategy.started:
            raise RuntimeError("Dream inference is already started")
        observation = self._observation(joints, images, ages)
        spec = load_robot()
        effective_limits = dict(spec.limits)
        if limits is not None:
            if not set(JOINTS) <= set(limits):
                raise ValueError("workbench limits must contain all six joints")
            for name in JOINTS:
                if len(limits[name]) != 2:
                    raise ValueError(f"workbench limits for {name} must contain a lower and upper bound")
                lower, upper = map(float, limits[name])
                if not all(map(math.isfinite, (lower, upper))) or lower > upper:
                    raise ValueError(f"invalid workbench limits for {name}")
                lower = max(lower, spec.limits[name][0])
                upper = min(upper, spec.limits[name][1])
                if lower > upper:
                    raise ValueError(f"workbench limits for {name} do not overlap the robot specification")
                effective_limits[name] = (lower, upper)
        run = RunInfo(robot="so101", joints=JOINTS, gripper="gripper", limits=effective_limits,
                      max_vel={name: min(self.max_speed, 0.2 * spec.max_vel[name]) for name in JOINTS},
                      folder=soarm.ROBOT_DIR, simulated=False)
        try:
            self.strategy.start(run, observation)
        except BaseException:
            if self._budget is not None:
                self._budget.close()
            raise
        self._started_at = time.monotonic()
        self._chunk_timing = self._candidate_timing = None
        self._preview_timing = self._raw_preview = None

    def _metadata(self) -> dict[str, Any]:
        status = self.strategy.status
        now = time.monotonic()
        return {
            "chunks": status.chunks, "requests": self._budget.requests,
            "rtt_ms": status.rtt_ms, "rows_left": status.rows_left,
            "source_age_s": None if self._chunk_timing is None else now - self._chunk_timing[0],
            "chunk_age_s": None if self._chunk_timing is None else now - self._chunk_timing[1],
            "preview_source_age_s": (None if self._preview_timing is None else
                                     now - self._preview_timing[0]),
            "raw_action_units": "five arm radians, then gripper 0..1",
            "last_error": self._budget.last_error,
            "recording_enabled": self._budget.allow_recording, "experimental_single_arm": True,
        }

    def _provider_error_note(self) -> str:
        error = self._budget.last_error if self._budget is not None else None
        if not error:
            return ""
        detail = error.get("message") or error.get("reason") or error["type"]
        label = " / ".join(str(error[key]) for key in ("code", "reason") if error.get(key))
        note = f" Dream reported {label or error['type']}: {detail}."
        retry = error.get("retry_after_s")
        if (isinstance(retry, (int, float)) and not isinstance(retry, bool)
                and math.isfinite(retry) and retry > 0):
            note += f" The provider requested a {retry:g}-second retry delay; wait, then rerun the preview."
        return note

    def preview_timeout(self, timeout_s: float) -> InferenceResult:
        """End a notebook's shorter wait with diagnostics, never an executable action.

        The notebook still closes the adapter in its finally block. No extra request
        is started and neither its request budget nor the action-age limit is extended.
        """
        if self._budget is None or self._started_at is None:
            raise RuntimeError("call start() before ending a preview")
        if not math.isfinite(timeout_s) or timeout_s <= 0:
            raise ValueError("preview timeout must be positive and finite")
        note = f"No Dream action chunk arrived within the {timeout_s:g}-second preview deadline."
        return InferenceResult([], 0.0, False, note + self._provider_error_note(), done=True,
                               raw_actions=self._raw_preview, metadata=self._metadata())

    def step(self, joints: Mapping[str, float], images: Mapping[str, np.ndarray], *,
             ages: Mapping[str, float] | None = None,
             command: Mapping[str, float] | None = None) -> InferenceResult:
        if self._started_at is None or not self.strategy.started:
            raise RuntimeError("call start() before step()")
        if time.monotonic() - self._started_at >= self.max_seconds:
            return InferenceResult([], 0.0, False, "Dream inference time budget exhausted", done=True,
                                   raw_actions=self._raw_preview, metadata=self._metadata())
        if self._chunk_timing is not None:
            if any(time.monotonic() - stamp > self.max_output_age_s for stamp in self._chunk_timing):
                return InferenceResult([], 0.0, False,
                                       "Dream output expired; acquire a fresh prediction", done=True,
                                       raw_actions=self._raw_preview, metadata=self._metadata())
        self._step += 1
        started = time.monotonic()
        before_chunks = self.strategy.status.chunks
        try:
            observation = self._observation(joints, images, ages, command)
            # Anchor planning to the command actually sent by the workbench. During
            # a no-motion preview, measured joints are the only honest anchor.
            self.strategy._q = self.strategy._vector(observation.command)
            self._budget.observation_age_s = max(float(value) for value in ages.values())
            action = self.strategy.act(observation)
        except _RequestBudgetExceeded as exc:
            return InferenceResult([], time.monotonic() - started, False, str(exc), done=True,
                                   raw_actions=self._raw_preview, metadata=self._metadata())
        status = self.strategy.status
        if (status.chunks == 0 and self._budget.last_error
                and self._budget.requests >= self._budget.max_requests):
            error = self._budget.last_error
            note = (f"Dream request budget exhausted after {error['type']}: "
                    f"{error.get('reason') or error['message']}")
            return InferenceResult([], time.monotonic() - started, False, note, done=True,
                                   raw_actions=self._raw_preview, metadata=self._metadata())
        if (status.chunks == 0 and self._budget.last_error
                and self.strategy._retry_at >= self._budget.deadline):
            note = "Dream's scheduled retry falls outside this preview's remaining time budget."
            return InferenceResult([], time.monotonic() - started, False,
                                   note + self._provider_error_note(), done=True,
                                   raw_actions=self._raw_preview, metadata=self._metadata())
        if status.chunks != before_chunks:
            self._chunk_timing = self._candidate_timing
        executable = status.chunks > 0 and not action.done
        return InferenceResult([dict(action.target)] if executable else [], time.monotonic() - started,
                               executable, action.note, done=action.done, raw_actions=self._raw_preview,
                               metadata=self._metadata())

    def close(self) -> None:
        self.strategy.stop("notebook finished")
        self._started_at = None
