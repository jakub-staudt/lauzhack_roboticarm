"""Deterministic line/boundary and worker-mailbox tests; no hardware."""
import threading
import time

import pytest

from soarm.panel_cartesian import Workspace
from soarm.panel_cartesian_ranges import CartesianRangeEstimator, estimate_ranges


class BoxSolver:
    def __init__(self):
        self.pitches = []

    def position(self, joints):
        return joints["r"], 0.0, joints["z"]

    def heading(self, joints):
        return 1.0, 0.0

    def pitch(self, joints):
        return 0.3

    def solve(self, joints, position, *, pitch=None):
        self.pitches.append(pitch)
        x, _, z = position
        if not -0.035 <= x <= 0.027 or not -1.0 <= z <= 1.0:
            raise ValueError("boundary")
        return dict(joints, r=x, z=z)


LIMITS = {"r": (-1.0, 1.0), "z": (-1.0, 1.0)}
WORKSPACE = Workspace(table_z_m=0.0, tcp_margin_m=0.02)
POSE = {"r": 0.0, "z": 0.05}


def test_connected_ranges_floor_horizon_and_fixed_pitch():
    solver = BoxSolver()
    result = estimate_ranges(solver, POSE, LIMITS, WORKSPACE)
    assert result["status"] == "ready"
    directions = result["directions"]
    for name, expected in (("forward", 0.027), ("back", 0.035), ("down", 0.03)):
        assert 0 <= expected - directions[name]["distance_m"] <= 0.0005
        assert directions[name]["allowed"]
        assert not directions[name]["at_least"]
        assert directions[name]["limiting_joint"] is None
    assert directions["down"]["limiting_reason"] == "Configured TCP floor."
    assert directions["up"]["distance_m"] == 0.4
    assert directions["up"]["at_least"]
    assert set(solver.pitches) == {0.3}


def test_outside_limit_requires_recovery_without_clipping_or_solving():
    solver = BoxSolver()
    result = estimate_ranges(solver, dict(POSE, r=-1.001), LIMITS, WORKSPACE)
    assert result["status"] == "unavailable"
    assert "recovery is required" in result["directions"]["forward"]["limiting_reason"]
    assert result["directions"]["forward"]["distance_m"] is None
    assert not solver.pitches


def test_budget_exhaustion_is_unknown():
    result = estimate_ranges(BoxSolver(), POSE, LIMITS, WORKSPACE, budget_s=0.0)
    assert result["status"] == "unavailable"
    assert all(item["distance_m"] is None for item in result["directions"].values())


def wait_until(predicate):
    deadline = time.monotonic() + 2.0
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(0.005)
    pytest.fail("Worker did not complete within two seconds")


def test_worker_never_publishes_changed_pose_as_fresh():
    entered, release = threading.Event(), threading.Event()
    worker_threads = []

    def factory():
        worker_threads.append(threading.current_thread().name)
        entered.set()
        assert release.wait(1.0)
        return BoxSolver()

    estimator = CartesianRangeEstimator(factory, LIMITS, WORKSPACE, interval_s=10.0)
    try:
        estimator.observe(POSE, 1, time.monotonic())
        assert entered.wait(1.0)
        assert estimator.snapshot()["status"] == "pending"
        estimator.observe(dict(POSE, r=0.001), 2, time.monotonic())
        release.set()
        wait_until(lambda: estimator.snapshot()["status"] != "pending")
        snapshot = estimator.snapshot()
        assert snapshot["status"] == "stale"
        assert snapshot["stale_reason"] == "stale_pose"
        assert snapshot["observation_sequence"] == 1
        assert snapshot["source_observation_sequence"] == 1
        assert snapshot["source_pose"] == POSE
        assert snapshot["latest_pose_delta_deg"] == pytest.approx(0.001)
        assert snapshot["computed_age_s"] >= 0
        assert worker_threads == ["cartesian-ranges"]
        snapshot["directions"]["forward"]["distance_m"] = 42
        assert estimator.snapshot()["directions"]["forward"]["distance_m"] != 42
    finally:
        release.set()
        estimator.close()


def test_unchanged_pose_refreshes_identity_and_stale_feedback_is_hidden():
    estimator = CartesianRangeEstimator(BoxSolver, LIMITS, WORKSPACE)
    try:
        estimator.observe(POSE, 3, time.monotonic())
        wait_until(lambda: estimator.snapshot()["status"] == "ready")
        estimator.observe(POSE, 4, time.monotonic())
        assert estimator.snapshot()["observation_sequence"] == 4
        assert estimator.snapshot()["source_observation_sequence"] == 3
        assert estimator.snapshot()["stale_reason"] is None
        estimator.observe(POSE, 5, time.monotonic() - 1.0)
        assert estimator.snapshot()["status"] == "stale"
        assert estimator.snapshot()["stale_reason"] == "stale_feedback"
        # Changed pose cannot override stale feedback: previous estimates must
        # never be shown as usable while observations are interrupted.
        estimator.observe(dict(POSE, r=0.001), 6, time.monotonic() - 1.0)
        assert estimator.snapshot()["stale_reason"] == "stale_feedback"
    finally:
        estimator.close()


def test_unconfigured_workspace_does_not_start_worker_or_factory():
    def forbidden():
        raise AssertionError("Factory must not run")

    estimator = CartesianRangeEstimator(forbidden, LIMITS, Workspace())
    estimator.observe(POSE, 1, time.monotonic())
    assert estimator.snapshot()["status"] == "unavailable"
    assert estimator._thread is None
    estimator.close()
