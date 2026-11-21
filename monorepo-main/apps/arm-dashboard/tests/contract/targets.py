"""How each implementation under test is booted against the fakes.

The contract tests talk HTTP/WebSocket to a ``Dashboard`` and never import the implementation.
Only this module knows how to start one. Adding a target means adding one ``start_*``
function here; no test changes.
"""

import socket
from collections.abc import Iterator
from contextlib import contextmanager
from dataclasses import dataclass
from functools import partial
from http.server import ThreadingHTTPServer
from pathlib import Path

import pytest
from pydantic import AnyHttpUrl

import legacy
from arm_dashboard.api.v1.dependencies import build_services
from arm_dashboard.core.settings import ControllerSettings, ScanSettings, Settings, ViewerSettings
from arm_dashboard.main import create_app
from arm_dashboard.scan import ScanWorkerRunner
from fakes import Address, running, running_asgi

STATIC_DIR = legacy.STATIC_DIR
LEGACY = 'legacy'
FASTAPI = 'fastapi'
TARGETS = (LEGACY, FASTAPI)


@dataclass(frozen=True)
class TargetConfig:
    controller: Address
    viewer: Address
    scan_storage: Path
    # A directory with no scanner runtime in it, so "start" deterministically reports it missing.
    scan_runtime_root: Path


@contextmanager
def start_target(name: str, config: TargetConfig, monkeypatch: pytest.MonkeyPatch) -> Iterator[Address]:
    if name == LEGACY:
        with _start_legacy(config, monkeypatch) as address:
            yield address
    elif name == FASTAPI:
        with _start_fastapi(config) as address:
            yield address
    else:
        raise KeyError(f'Unknown target {name!r}; available: {TARGETS}')


@contextmanager
def _start_legacy(config: TargetConfig, monkeypatch: pytest.MonkeyPatch) -> Iterator[Address]:
    server, viewer_proxy, scan_manager = (
        legacy.load('server'),
        legacy.load('viewer_proxy'),
        legacy.load('scan_manager'),
    )
    # Legacy has no settings: its upstream addresses are module globals, and `direct_hosts`
    # is computed at import time from the hard-coded upstream. Point all of them at the fakes.
    monkeypatch.setattr(server, 'UPSTREAM', config.controller)
    monkeypatch.setitem(
        server.CAMERA_DELIVERY,
        'direct_hosts',
        list(dict.fromkeys((config.controller[0], socket.gethostname()))),
    )
    monkeypatch.setattr(viewer_proxy, 'VIEWER_UPSTREAM', config.viewer)

    httpd = ThreadingHTTPServer(('127.0.0.1', 0), server.Handler)
    httpd.daemon_threads = True
    controller_url = f'http://{config.controller[0]}:{config.controller[1]}'
    httpd.scan_manager = scan_manager.ScanManager(
        config.scan_runtime_root, config.scan_storage, controller_url
    )
    with running(httpd) as address:
        yield address


def _url(address: Address) -> AnyHttpUrl:
    return AnyHttpUrl(f'http://{address[0]}:{address[1]}')


@contextmanager
def _start_fastapi(config: TargetConfig) -> Iterator[Address]:
    settings = Settings(
        _env_file=None,
        controller=ControllerSettings(base_url=_url(config.controller)),
        viewer=ViewerSettings(base_url=_url(config.viewer)),
        scan=ScanSettings(storage_dir=config.scan_storage),
    )
    # The scanner runtime is reported missing, so "start" answers 409 deterministically,
    # whether or not OpenCV is installed in the test environment.
    runner = ScanWorkerRunner(runtime_installed=lambda: False)
    app = create_app(settings, services_factory=partial(build_services, scan_runner=runner))
    with running_asgi(app) as address:
        yield address
