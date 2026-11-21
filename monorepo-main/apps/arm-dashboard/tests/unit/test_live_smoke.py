import json
import sys

import pytest
from manual import live_smoke

CONTROLLER = ('127.0.0.1', 8765)
DASHBOARD = ('127.0.0.1', 8780)
PREVIEW = {'preview_only': True, 'connected': False, 'port': None}


@pytest.fixture(autouse=True)
def smoke_args(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setattr(
        sys,
        'argv',
        [
            'live_smoke.py',
            '--controller',
            'http://127.0.0.1:8765',
            '--dashboard',
            'http://127.0.0.1:8780',
        ],
    )


@pytest.mark.parametrize(
    'proxy_state',
    [
        (200, b'{"preview_only":true,"connected":true}'),
        (200, b'{"preview_only":true}'),
        (200, b'{"preview_only":true,"connected":false,"port":"/dev/ttyUSB0"}'),
        (200, b'not JSON'),
        (502, json.dumps(PREVIEW).encode()),
    ],
)
def test_smoke_refuses_unverified_proxy_before_running_checks(
    monkeypatch: pytest.MonkeyPatch, proxy_state: tuple[int, bytes]
) -> None:
    def request(target: live_smoke.Address, method: str, path: str) -> tuple[int, None, bytes]:
        assert (method, path) == ('GET', '/api/state')
        if target == CONTROLLER:
            return 200, None, json.dumps(PREVIEW).encode()
        assert target == DASHBOARD
        status, body = proxy_state
        return status, None, body

    monkeypatch.setattr(live_smoke, 'request', request)
    assert live_smoke.main() == 2


def test_smoke_only_posts_to_the_dashboard_scan_api(monkeypatch: pytest.MonkeyPatch) -> None:
    scan_start_statuses = iter((200, 409))
    scan_phases = iter(('collecting', 'stopped'))

    def request(
        target: live_smoke.Address, method: str, path: str, body: dict | None = None
    ) -> tuple[int, str | None, bytes]:
        if method == 'POST':
            assert target == DASHBOARD
            if path == '/api/scan/start':
                return next(scan_start_statuses), None, b'{}'
            assert path == '/api/scan/stop'
            return 200, None, b'{}'

        assert method == 'GET'
        if path == '/api/state':
            state = PREVIEW if target == CONTROLLER else {**PREVIEW, 'camera_delivery': {}}
            return 200, None, json.dumps(state).encode()
        if path == '/api/scan/status':
            return 200, None, json.dumps({'phase': next(scan_phases)}).encode()

        assets = {'/': 'text/html', '/panel.css': 'text/css', '/panel.js': 'text/javascript'}
        if path in assets:
            assert target == DASHBOARD
            return 200, assets[path], b'asset'
        status = {
            '/cameras/overhead.jpg': 503,
            '/cameras/overhead.mjpg': 502,
            '/cameras/overhead.mjpg?fps=3': 400,
            '/viewer/': 502,
        }.get(path, 200)
        return status, None, b''

    monkeypatch.setattr(live_smoke, 'request', request)
    monkeypatch.setattr(live_smoke.time, 'sleep', lambda _: None)
    assert live_smoke.main() == 0
