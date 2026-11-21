"""Notebook inference boundaries: joint units, stale frames, recorder and request budgets."""

from __future__ import annotations

import time

import numpy as np
import pytest

from soarm_lab.notebooks.models import (
    JOINTS,
    DreamInference,
    JointMapping,
    SmolVLAInference,
    _BudgetedDreamClient,
    _RequestBudgetExceeded,
)
from soarm_lab.robot import load_robot

JOINT_STATE = dict(load_robot().home)
IMAGES = {name: np.zeros((10, 10, 3), dtype=np.uint8) for name in ("overhead", "wrist")}
AGES = {name: 0.1 for name in IMAGES}


class FakeDreamClient:
    mode = "sync"

    def __init__(self, recorder_state="off"):
        self.config = None
        self.observations = []
        self.calls = 0
        self.closed = False
        self._recording_stats = {"state": recorder_state}

    def policy(self, config):
        self.config = config

    def set_observation(self, frames, observation, **kwargs):
        self.observations.append((frames, observation))

    def get_action(self, **kwargs):
        self.calls += 1
        row = np.array([*np.radians([JOINT_STATE[name] for name in JOINTS[:5]]), 0.5])
        return {"q_r_chunk": np.tile(row, (48, 1))}

    def refill_due(self, executed, remaining):
        return remaining == 0

    def close(self):
        self.closed = True


def test_explicit_joint_mapping_roundtrip_preserves_gripper_units():
    mapping = JointMapping((2, -2, 0.5, 3, 1, 0.01), (10, 20, -30, 0, 5, 0))
    encoded = mapping.encode(JOINT_STATE)
    decoded = mapping.decode(encoded[None])[0]
    assert decoded == pytest.approx(JOINT_STATE)
    assert encoded[5] == pytest.approx(JOINT_STATE["gripper"] / 100)


@pytest.mark.parametrize("scale,offset", [((1,) * 5, (0,) * 6), ((0,) * 6, (0,) * 6),
                                        ((1,) * 6, (float("nan"),) * 6)])
def test_mapping_rejects_ambiguous_or_invalid_coefficients(scale, offset):
    with pytest.raises(ValueError):
        JointMapping(scale, offset)


def test_base_weights_cannot_be_armed_even_with_operator_mapping():
    identity = JointMapping((1,) * 6, (0,) * 6)
    assert not SmolVLAInference(joint_mapping=identity, trained_for_robot=True).executable
    assert not SmolVLAInference("someone/fitted-model", trained_for_robot=True).executable
    assert SmolVLAInference("someone/fitted-model", joint_mapping=identity,
                            trained_for_robot=True).executable


@pytest.mark.parametrize("ages", [None, {"overhead": 0.1}, {"overhead": float("nan"), "wrist": 0.1},
                                  {"overhead": 0.1, "wrist": 2.0}])
def test_stale_or_unknown_camera_age_is_rejected_before_model_load(ages):
    with pytest.raises(ValueError, match="age|stale"):
        SmolVLAInference().predict(JOINT_STATE, IMAGES, "move the mouse", ages=ages)


def test_dream_disables_recorder_before_observations():
    client = FakeDreamClient()
    adapter = DreamInference("move the mouse onto the left gray area", client_factory=lambda: client)
    adapter.start(JOINT_STATE, IMAGES, ages=AGES)
    try:
        assert client.config["_record"] is False
        assert client.observations == []
        waiting = adapter.step(JOINT_STATE, IMAGES, ages=AGES)
        assert not waiting.executable and waiting.actions == [] and not waiting.done
        adapter.strategy.wait_request(2)
        result = adapter.step(JOINT_STATE, IMAGES, ages=AGES)
        assert result.metadata["chunks"] == 1
        assert result.metadata["requests"] == 1
        assert result.executable
        assert set(result.actions[0]) == set(JOINTS)
        frames, state = client.observations[0]
        assert set(frames) == {"video.scene", "video.wrist_r", "video.wrist_l"}
        assert frames["video.wrist_l"] == frames["video.scene"]  # Documented experimental fill.
        assert state["state.right_arm"] == pytest.approx(np.radians([
            JOINT_STATE[name] for name in JOINTS[:5]]))
    finally:
        adapter.close()
    assert client.closed


def test_unverified_dream_recorder_refuses_before_camera_upload():
    client = FakeDreamClient(recorder_state="connecting")
    adapter = DreamInference("move the mouse", client_factory=lambda: client)
    with pytest.raises(RuntimeError, match="recorder"):
        adapter.start(JOINT_STATE, IMAGES, ages=AGES)
    assert client.closed
    assert client.observations == []
    assert client.calls == 0


def test_dream_budget_counts_failed_attempts_and_stops_before_next_request():
    client = FakeDreamClient()
    wrapped = _BudgetedDreamClient(client, allow_recording=False, max_requests=1, max_seconds=30)
    wrapped.policy({})
    wrapped.set_observation({}, {})
    wrapped.get_action()
    with pytest.raises(_RequestBudgetExceeded):
        wrapped.get_action()
    assert client.calls == 1
    wrapped.deadline = time.monotonic() - 1
    with pytest.raises(_RequestBudgetExceeded):
        wrapped.set_observation({}, {})
    assert len(client.observations) == 1


def test_dream_time_limit_returns_done_without_sending_another_frame():
    client = FakeDreamClient()
    adapter = DreamInference("move the mouse", client_factory=lambda: client)
    adapter.start(JOINT_STATE, IMAGES, ages=AGES)
    try:
        adapter._started_at = time.monotonic() - 31
        result = adapter.step(JOINT_STATE, IMAGES, ages=AGES)
        assert result.done and not result.executable and result.actions == []
        assert client.observations == []
    finally:
        adapter.close()


def test_late_dream_reply_is_rejected_before_any_target_is_executable():
    client = FakeDreamClient()
    adapter = DreamInference("move the mouse", max_output_age_s=0.01, client_factory=lambda: client)
    adapter.start(JOINT_STATE, IMAGES, ages=AGES)
    try:
        # Camera data already 0.1s old: the 0.01s provenance budget cannot accept it.
        adapter.step(JOINT_STATE, IMAGES, ages=AGES)
        adapter.strategy.wait_request(2)
        result = adapter.step(JOINT_STATE, IMAGES, ages=AGES)
        assert result.done and not result.executable
        assert "stale" in result.note
        assert result.actions == []
        assert result.raw_actions.shape == (48, 6)
        assert np.isfinite(result.raw_actions).all()
        assert result.metadata["preview_source_age_s"] >= 0.1
        assert result.metadata["raw_action_units"] == "five arm radians, then gripper 0..1"
    finally:
        adapter.close()


def test_dream_proposals_intersect_server_eeprom_limits_with_robot_limits():
    client = FakeDreamClient()
    adapter = DreamInference("move the mouse", client_factory=lambda: client)
    spec = load_robot()
    limits = dict(spec.limits)
    limits["shoulder_pan"] = (-1, 2)
    limits["gripper"] = (-200, 200)  # Wider server limits cannot widen the robot specification.
    adapter.start(JOINT_STATE, IMAGES, ages=AGES, limits=limits)
    try:
        assert adapter.strategy._lo[0] == -1
        assert adapter.strategy._hi[0] == 2
        assert (adapter.strategy._lo[5], adapter.strategy._hi[5]) == spec.limits["gripper"]
    finally:
        adapter.close()


def test_invalid_server_limits_fail_before_opening_dream_policy():
    client = FakeDreamClient()
    adapter = DreamInference("move the mouse", client_factory=lambda: client)
    limits = dict(load_robot().limits)
    limits["shoulder_pan"] = (500, 600)
    with pytest.raises(ValueError, match="do not overlap"):
        adapter.start(JOINT_STATE, IMAGES, ages=AGES, limits=limits)
    assert client.config is None


def test_retryable_service_error_is_preserved_when_request_budget_is_exhausted():
    class RetryableError(RuntimeError):
        code = "DA-2000"
        reason = "pool_warming"
        retryable = True
        retry_after_s = 30

    class WarmingClient(FakeDreamClient):
        _api_key = "secret-do-not-print"

        def get_action(self, **kwargs):
            raise RetryableError(f"pool warming; sensitive example {self._api_key}")

    client = WarmingClient()
    adapter = DreamInference("move the mouse", max_requests=1, client_factory=lambda: client)
    adapter.start(JOINT_STATE, IMAGES, ages=AGES)
    try:
        result = adapter.step(JOINT_STATE, IMAGES, ages=AGES)
        adapter.strategy.wait_request(2)
        if not result.done:
            result = adapter.step(JOINT_STATE, IMAGES, ages=AGES)
        assert result.done and not result.executable
        error = result.metadata["last_error"]
        assert error["reason"] == "pool_warming"
        assert error["code"] == "DA-2000"
        assert error["retry_after_s"] == 30
        assert "secret-do-not-print" not in error["message"]
        assert "[redacted]" in error["message"]
        assert "pool_warming" in result.note
    finally:
        adapter.close()


def test_provider_retry_that_cannot_fit_time_budget_returns_capacity_reason_promptly():
    class WarmingError(RuntimeError):
        code = "DA-2000"
        reason = "pool_warming"
        retryable = True
        retry_after_s = 30

    class WarmingClient(FakeDreamClient):
        def get_action(self, **kwargs):
            self.calls += 1
            raise WarmingError("NO_CAPACITY: no live serve yet")

    client = WarmingClient()
    # These are the notebook's actual defaults: request budget is not exhausted.
    adapter = DreamInference("move the mouse", max_requests=3, max_seconds=30,
                             client_factory=lambda: client)
    adapter.start(JOINT_STATE, IMAGES, ages=AGES)
    try:
        adapter.step(JOINT_STATE, IMAGES, ages=AGES)
        adapter.strategy.wait_request(2)
        result = adapter.step(JOINT_STATE, IMAGES, ages=AGES)
        assert result.done and not result.executable and result.actions == []
        assert result.metadata["last_error"]["reason"] == "pool_warming"
        assert result.metadata["last_error"]["retry_after_s"] == 30
        assert "pool_warming" in result.note
        assert "30" in result.note
        assert client.calls == 1
        assert result.metadata["requests"] < adapter._budget.max_requests
    finally:
        adapter.close()


def test_short_provider_backoff_within_budget_can_recover_without_extending_deadline(monkeypatch):
    clock = [100.0]
    monkeypatch.setattr(time, "monotonic", lambda: clock[0])

    class WarmingError(RuntimeError):
        code = "DA-2000"
        reason = "pool_warming"
        retryable = True
        retry_after_s = 0.5

    class WarmingOnceClient(FakeDreamClient):
        def get_action(self, **kwargs):
            if self.calls == 0:
                self.calls += 1
                raise WarmingError("NO_CAPACITY: no live serve yet")
            return super().get_action(**kwargs)

    client = WarmingOnceClient()
    adapter = DreamInference("move the mouse", max_requests=3, max_seconds=30,
                             client_factory=lambda: client)
    adapter.start(JOINT_STATE, IMAGES, ages=AGES)
    original_deadline = adapter._budget.deadline
    try:
        adapter.step(JOINT_STATE, IMAGES, ages=AGES)
        adapter.strategy.wait_request(2)
        waiting = adapter.step(JOINT_STATE, IMAGES, ages=AGES)
        assert not waiting.done and not waiting.executable and waiting.actions == []
        assert waiting.metadata["last_error"]["reason"] == "pool_warming"
        assert client.calls == 1
        clock[0] += 1
        adapter.step(JOINT_STATE, IMAGES, ages=AGES)
        adapter.strategy.wait_request(2)
        recovered = adapter.step(JOINT_STATE, IMAGES, ages=AGES)
        assert recovered.executable and not recovered.done
        assert recovered.metadata["chunks"] == 1
        assert recovered.metadata["last_error"] is None
        assert client.calls == 2
        assert adapter._budget.deadline == original_deadline
        assert adapter.max_output_age_s == 3
    finally:
        adapter.close()


@pytest.mark.parametrize("provider_error", [False, True])
def test_preview_timeout_keeps_provider_diagnostics_without_requests_or_executable_actions(provider_error):
    class WarmingError(RuntimeError):
        code = "DA-2000"
        reason = "pool_warming"
        retryable = True
        retry_after_s = 10

    class WarmingClient(FakeDreamClient):
        def get_action(self, **kwargs):
            self.calls += 1
            raise WarmingError("NO_CAPACITY: no live serve yet")

    client = WarmingClient() if provider_error else FakeDreamClient()
    adapter = DreamInference("move the mouse", max_requests=3, max_seconds=30,
                             client_factory=lambda: client)
    adapter.start(JOINT_STATE, IMAGES, ages=AGES)
    try:
        if provider_error:
            adapter.step(JOINT_STATE, IMAGES, ages=AGES)
            adapter.strategy.wait_request(2)
            waiting = adapter.step(JOINT_STATE, IMAGES, ages=AGES)
            assert not waiting.done
        calls, observations = client.calls, len(client.observations)
        result = adapter.preview_timeout(25)
        assert result.done and not result.executable and result.actions == []
        assert "25" in result.note
        assert client.calls == calls and len(client.observations) == observations
        if provider_error:
            assert result.metadata["last_error"]["code"] == "DA-2000"
            assert result.metadata["last_error"]["reason"] == "pool_warming"
            assert result.metadata["last_error"]["retry_after_s"] == 10
            assert "pool_warming" in result.note
        else:
            assert result.metadata["last_error"] is None
    finally:
        adapter.close()


def test_fresh_camera_frames_cannot_keep_an_expired_dream_target_alive():
    client = FakeDreamClient()
    adapter = DreamInference("move the mouse", client_factory=lambda: client)
    adapter.start(JOINT_STATE, IMAGES, ages=AGES)
    try:
        adapter.step(JOINT_STATE, IMAGES, ages=AGES)
        adapter.strategy.wait_request(2)
        result = adapter.step(JOINT_STATE, IMAGES, ages=AGES)
        assert result.metadata["chunks"] == 1
        assert result.metadata["source_age_s"] >= 0.1
        adapter._chunk_timing = (time.monotonic() - 4, time.monotonic() - 3)
        previous_frames = len(client.observations)
        result = adapter.step(JOINT_STATE, IMAGES, ages={name: 0.0 for name in IMAGES})
        assert result.done and not result.executable and result.actions == []
        assert len(client.observations) == previous_frames
    finally:
        adapter.close()

@pytest.mark.parametrize('use_command', [False, True])
def test_notebook_planning_does_not_accumulate_motion_the_workbench_never_sent(use_command):
    class MovingClient(FakeDreamClient):
        def get_action(self, **kwargs):
            reply = super().get_action(**kwargs)
            reply['q_r_chunk'][:, 0] += np.radians(20)
            return reply

    client = MovingClient()
    adapter = DreamInference('move the mouse', client_factory=lambda: client, max_speed=8)
    adapter.start(JOINT_STATE, IMAGES, ages=AGES)
    anchor = dict(JOINT_STATE)
    if use_command:
        anchor['shoulder_pan'] += 2
    kwargs = {'command': anchor} if use_command else {}
    try:
        adapter.step(JOINT_STATE, IMAGES, ages=AGES, **kwargs)
        adapter.strategy.wait_request(2)
        targets = [adapter.step(JOINT_STATE, IMAGES, ages=AGES, **kwargs).actions[0]['shoulder_pan']
                   for _ in range(10)]
        # Neither preview ticks nor rate-limited hardware may advance the planning
        # anchor beyond feedback. Each proposal is at most one 8/20-degree step away.
        assert targets == pytest.approx([anchor['shoulder_pan'] + 8 / 20] * 10)
    finally:
        adapter.close()


def test_dream_sdk_receives_explicit_key_only(monkeypatch):
    from types import SimpleNamespace

    from soarm_lab.notebooks import models

    monkeypatch.setenv("DREAM_API_KEY", "ambient-must-not-be-used")
    seen = []

    def sdk_client(*, api_key):
        seen.append(api_key)
        return object()

    monkeypatch.setattr(models, "version", lambda name: models.DREAM_CLIENT_VERSION)
    monkeypatch.setattr(models.importlib.util, "find_spec", lambda name: object())
    fake_sdk = SimpleNamespace(DreamClient=sdk_client)
    monkeypatch.setattr(models.importlib, "import_module", lambda name: fake_sdk)
    with pytest.raises(ValueError, match="hidden prompt"):
        models.dream_client("")
    with pytest.raises(ValueError, match="hidden prompt"):
        DreamInference("move the mouse")
    assert not seen
    models.dream_client("operator-test-key")
    assert seen == ["operator-test-key"]
