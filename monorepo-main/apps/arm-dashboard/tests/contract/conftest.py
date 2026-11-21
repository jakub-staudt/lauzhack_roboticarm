"""Fixtures for the black-box contract suite. Every test runs once per target in ``TARGETS``."""

from collections.abc import Callable, Iterator
from contextlib import ExitStack
from pathlib import Path

import pytest

from fakes import Address, FakeController, FakeViewer
from harness import Dashboard
from targets import TARGETS, TargetConfig, start_target

StartDashboard = Callable[..., Dashboard]


@pytest.fixture(params=TARGETS)
def target(request: pytest.FixtureRequest) -> str:
    return request.param


@pytest.fixture
def scan_storage(tmp_path: Path) -> Path:
    return tmp_path / 'scans'


@pytest.fixture
def start_dashboard(
    target: str, scan_storage: Path, tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> Iterator[StartDashboard]:
    """Factory: tests that seed scan sessions or kill an upstream do so before calling it."""
    with ExitStack() as stack:

        def start(
            *, controller_address: Address | None = None, viewer_address: Address | None = None
        ) -> Dashboard:
            controller, viewer = FakeController(), FakeViewer()
            config = TargetConfig(
                controller=controller_address or stack.enter_context(controller.serve()),
                viewer=viewer_address or stack.enter_context(viewer.serve()),
                scan_storage=scan_storage,
                scan_runtime_root=tmp_path / 'no-scanner-runtime',
            )
            host, port = stack.enter_context(start_target(target, config, monkeypatch))
            return Dashboard(target, host, port, controller, viewer, scan_storage)

        yield start


@pytest.fixture
def dashboard(start_dashboard: StartDashboard) -> Dashboard:
    return start_dashboard()
