"""Shared types must describe what the legacy implementation actually does. Deleted with ``legacy/``;
after that the contract suite and the explicit values in test_typing.py are the reference.
"""

import json
from pathlib import Path

import pytest

import legacy
from arm_dashboard.core.typing import (
    CameraDelivery,
    CameraName,
    ControllerCommand,
    ScanArtifact,
    ScanLimits,
    ScanPhase,
    ScanStatus,
)
from fakes import CannedResponse, FakeController

cv2 = pytest.importorskip('cv2', reason='legacy collector parity needs the scan extra')
np = pytest.importorskip('numpy')

COLLECT_SECONDS = 1.2
COLLECT_INTERVAL_S = 0.4


def test_commands_are_exactly_the_legacy_allowlist() -> None:
    assert {command.value for command in ControllerCommand} == legacy.load('server').POST_ROUTES


def test_camera_delivery_matches_legacy_payload() -> None:
    server = legacy.load('server')
    legacy_payload = dict(server.CAMERA_DELIVERY)
    current = CameraDelivery(
        default_fps=legacy_payload['default_fps'],
        direct_default_fps=legacy_payload['direct_default_fps'],
        direct_hosts=tuple(legacy_payload['direct_hosts']),
        available_fps=server.CAMERA_FPS,
    )

    assert current.model_dump(mode='json') == legacy_payload


def test_artifacts_are_exactly_the_legacy_whitelist(tmp_path: Path) -> None:
    scan_manager = legacy.load('scan_manager')
    session = tmp_path / 'data' / 'scan-20260101-000000-aaaaaa'
    session.mkdir(parents=True)
    candidates = [artifact.value for artifact in ScanArtifact] + [
        'frames.jsonl',
        'status.json',
        'collector.log',
    ]
    for name in candidates:
        (session / name).write_bytes(b'x')
    manager = scan_manager.ScanManager(tmp_path / 'app', tmp_path / 'data', 'http://127.0.0.1:8765')

    exposed = set()
    for name in candidates:
        try:
            manager.image(name)
            exposed.add(name)
        except FileNotFoundError:
            pass

    assert exposed == {artifact.value for artifact in ScanArtifact}


def test_status_written_by_legacy_collector_round_trips(tmp_path: Path) -> None:
    scan_worker = legacy.load('scan_worker')
    controller = FakeController()
    ok, jpeg = cv2.imencode('.jpg', np.full((48, 64, 3), 128, np.uint8))
    assert ok
    for camera in CameraName:
        controller.respond(
            'GET', f'/cameras/{camera}.jpg', CannedResponse(content_type='image/jpeg', body=jpeg.tobytes())
        )

    with controller.serve() as (host, port):
        scan_worker.collect(
            f'http://{host}:{port}', tmp_path, seconds=COLLECT_SECONDS, interval=COLLECT_INTERVAL_S
        )
    written = json.loads((tmp_path / 'status.json').read_text())

    status = ScanStatus.model_validate(written)

    assert status.phase is ScanPhase.COMPLETE
    assert status.model_dump(mode='json', exclude_unset=True) == written


def test_duration_limits_match_legacy_clamp() -> None:
    # scan_worker.main(): max(1, min(120, args.seconds))
    source = (legacy.LEGACY_DIR / 'scan_worker.py').read_text()

    assert f'max({ScanLimits.DURATION_MIN_S}, min({ScanLimits.DURATION_MAX_S}, args.seconds))' in source
