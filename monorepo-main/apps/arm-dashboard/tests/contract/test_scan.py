"""Automatic collection endpoints as seen by ``scan.html`` and ``scripts/deploy.sh``.

Collector process lifecycle (concurrent start, terminate) is unit-level (tests/unit/test_scan_*): it needs
the OpenCV runtime and is not observable through HTTP alone.
"""

import json
from pathlib import Path

import pytest

from harness import Dashboard

STATUS = '/api/scan/status'
ARTIFACTS = ['overhead.jpg', 'wrist.jpg', 'mosaic.jpg']
INTERRUPTED_NOTE = 'A previous collector was interrupted by service restart.'


def seed_session(
    storage: Path, name: str, status: dict[str, object] | None, files: dict[str, bytes] | None = None
) -> Path:
    session = storage / name
    session.mkdir(parents=True)
    if status is not None:
        (session / 'status.json').write_text(json.dumps(status))
    for filename, content in (files or {}).items():
        (session / filename).write_bytes(content)
    return session


def test_status_is_idle_without_sessions(dashboard: Dashboard) -> None:
    reply = dashboard.request('GET', STATUS)

    assert reply.status == 200
    assert reply.headers['Content-Type'] == 'application/json'
    assert reply.headers['Cache-Control'] == 'no-store'
    assert reply.json() == {
        'phase': 'idle',
        'captured_frames': 0,
        'units': 'pixels',
        'metric_calibrated': False,
    }


def test_status_reports_latest_session_with_worker_fields_intact(scan_storage: Path, start_dashboard) -> None:
    seed_session(scan_storage, 'scan-20260101-000000-aaaaaa', {'phase': 'complete', 'captured_frames': 1})
    latest = {
        'phase': 'complete',
        'captured_frames': 5,
        'coverage': {'observed_output_pixels': 10},
        'errors': 0,
    }
    seed_session(scan_storage, 'scan-20260102-000000-bbbbbb', latest)

    status = start_dashboard().request('GET', STATUS).json()

    assert status == latest | {'session': 'scan-20260102-000000-bbbbbb'}


@pytest.mark.parametrize('phase', ['starting', 'collecting'])
def test_abandoned_running_session_is_reported_interrupted(
    scan_storage: Path, start_dashboard, phase: str
) -> None:
    seed_session(scan_storage, 'scan-20260101-000000-aaaaaa', {'phase': phase, 'captured_frames': 3})

    status = start_dashboard().request('GET', STATUS).json()

    assert status['phase'] == 'interrupted'
    assert status['note'] == INTERRUPTED_NOTE
    assert status['captured_frames'] == 3


@pytest.mark.parametrize('content', [None, 'not json'])
def test_unreadable_status_file_is_reported_interrupted(
    scan_storage: Path, start_dashboard, content: str | None
) -> None:
    session = seed_session(scan_storage, 'scan-20260101-000000-aaaaaa', None)
    if content is not None:
        (session / 'status.json').write_text(content)

    status = start_dashboard().request('GET', STATUS).json()

    assert status['phase'] == 'interrupted'
    assert status['session'] == session.name


@pytest.mark.parametrize('artifact', ARTIFACTS)
def test_current_session_artifacts_are_served(scan_storage: Path, start_dashboard, artifact: str) -> None:
    seed_session(scan_storage, 'scan-20260101-000000-aaaaaa', {'phase': 'complete'}, {artifact: b'old'})
    seed_session(scan_storage, 'scan-20260102-000000-bbbbbb', {'phase': 'complete'}, {artifact: b'current'})

    reply = start_dashboard().request('GET', f'/api/scan/images/{artifact}')

    assert reply.status == 200
    assert reply.body == b'current'
    assert reply.headers['Content-Type'] == 'image/jpeg'
    assert reply.headers['Cache-Control'] == 'no-store'


@pytest.mark.parametrize(
    'name',
    [
        'frames.jsonl',
        'status.json',
        'collector.log',
        '00000-overhead.jpg',
        '..%2Fsecret',
        '%2E%2E',
        'missing',
    ],
)
def test_only_whitelisted_artifacts_are_exposed(scan_storage: Path, start_dashboard, name: str) -> None:
    files = {'frames.jsonl': b'private', 'collector.log': b'log', '00000-overhead.jpg': b'raw'}
    seed_session(scan_storage, 'scan-20260101-000000-aaaaaa', {'phase': 'complete'}, files)
    (scan_storage / 'secret').write_bytes(b'secret')

    assert start_dashboard().request('GET', f'/api/scan/images/{name}').status == 404


def test_whitelisted_artifact_missing_on_disk_is_404(scan_storage: Path, start_dashboard) -> None:
    seed_session(scan_storage, 'scan-20260101-000000-aaaaaa', {'phase': 'starting'})

    assert start_dashboard().request('GET', '/api/scan/images/mosaic.jpg').status == 404


def test_artifact_without_any_session_is_404(dashboard: Dashboard) -> None:
    assert dashboard.request('GET', '/api/scan/images/mosaic.jpg').status == 404


def test_start_without_scanner_runtime_is_409_with_error_message(dashboard: Dashboard) -> None:
    # scan.html shows `body.error` to the operator.
    reply = dashboard.request(
        'POST', '/api/scan/start', headers={'Content-Type': 'application/json'}, body=b'{}'
    )

    assert reply.status == 409
    assert isinstance(reply.json()['error'], str)
    assert reply.json()['error']
    assert dashboard.controller.requests == []


def test_stop_without_running_collector_returns_status(dashboard: Dashboard) -> None:
    reply = dashboard.request('POST', '/api/scan/stop', body=b'{}')

    assert reply.status == 200
    assert reply.json()['phase'] == 'idle'


@pytest.mark.parametrize('action', ['start', 'stop'])
def test_cross_origin_scan_action_is_403(dashboard: Dashboard, action: str) -> None:
    headers = {'Origin': 'http://untrusted.invalid'}

    assert dashboard.request('POST', f'/api/scan/{action}', headers=headers, body=b'{}').status == 403


def test_scan_action_body_over_limit_is_413(dashboard: Dashboard) -> None:
    assert dashboard.request('POST', '/api/scan/start', body=b' ' * 8193).status == 413


@pytest.mark.parametrize('action', ['start', 'stop'])
def test_scan_actions_do_not_answer_get(dashboard: Dashboard, action: str) -> None:
    assert dashboard.request('GET', f'/api/scan/{action}').status == 404
