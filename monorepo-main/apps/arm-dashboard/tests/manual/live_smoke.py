"""End-to-end smoke check of a running dashboard in front of a *preview* soarm-panel.

Checks dashboard assets, enriched controller state, cameras, viewer, map and scan pages,
and a real scan collector start/stop. Not collected by pytest:

    uv run python tests/manual/live_smoke.py \\
        --dashboard http://127.0.0.1:8780 --controller http://127.0.0.1:8765

Both the direct and proxied controller must report disconnected preview. Controller commands
are covered separately by tests/contract/test_commands.py against a fake controller.
"""

import argparse
import http.client
import json
import sys
import time
from urllib.parse import urlsplit

Address = tuple[str, int]
TIMEOUT_S = 5
SCAN_RUN_S = 3.0
SCAN_STOP_S = 2.5


def address(url: str) -> Address:
    parts = urlsplit(url)
    return parts.hostname or '127.0.0.1', parts.port or 80


def request(
    target: Address,
    method: str,
    path: str,
    body: dict[str, object] | None = None,
) -> tuple[int, str | None, bytes]:
    connection = http.client.HTTPConnection(*target, timeout=TIMEOUT_S)
    payload = json.dumps(body).encode() if body is not None else None
    headers = {'Content-Type': 'application/json'} if payload is not None else {}
    try:
        connection.request(method, path, body=payload, headers=headers)
        response = connection.getresponse()
        return response.status, response.getheader('Content-Type'), response.read()
    finally:
        connection.close()


def preview_state(target: Address) -> dict | None:
    try:
        status, _, body = request(target, 'GET', '/api/state')
        state = json.loads(body) if status == 200 else None
    except (OSError, http.client.HTTPException, ValueError):
        return None
    if (
        isinstance(state, dict)
        and state.get('preview_only') is True
        and state.get('connected') is False
        and state.get('port') is None
    ):
        return state
    return None


class Report:
    def __init__(self) -> None:
        self.failures = 0

    def check(self, name: str, ok: bool, detail: str = '') -> None:
        self.failures += not ok
        print(f'{"OK  " if ok else "FAIL"} {name:50} {detail}')


def main() -> int:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument('--dashboard', required=True)
    parser.add_argument('--controller', required=True, help='the same controller, reached directly')
    args = parser.parse_args()
    dash, ctrl = address(args.dashboard), address(args.controller)

    controller_state = preview_state(ctrl)
    if controller_state is None:
        print('Refusing: the direct controller did not report disconnected preview.')
        return 2
    state = preview_state(dash)
    if state is None:
        print('Refusing: the controller reached through the dashboard did not report disconnected preview.')
        return 2

    report = Report()
    for path, media_type in (
        ('/', 'text/html'),
        ('/panel.css', 'text/css'),
        ('/panel.js', 'text/javascript'),
    ):
        status, content_type, body = request(dash, 'GET', path)
        report.check(
            f'GET {path} from dashboard',
            status == 200 and (content_type or '').split(';')[0] == media_type and bool(body),
            f'{len(body)} B',
        )

    report.check(
        'state enriched with camera_delivery',
        'camera_delivery' in state,
        str(state.get('camera_delivery', {}).get('direct_hosts')),
    )
    report.check(
        'state keeps every controller key', set(controller_state) <= set(state), f'{len(state)} keys'
    )

    cameras = controller_state.get('cameras', {})
    live = [name for name, camera in cameras.items() if camera.get('status') == 'live']
    if live:
        report.check(
            f'snapshot of live camera {live[0]}', request(dash, 'GET', f'/cameras/{live[0]}.jpg')[0] == 200
        )
    else:
        report.check(
            'snapshot without cameras relays the 503', request(dash, 'GET', '/cameras/overhead.jpg')[0] == 503
        )
        report.check('MJPEG without cameras is 502', request(dash, 'GET', '/cameras/overhead.mjpg')[0] == 502)
    report.check(
        'MJPEG with an invalid fps is 400', request(dash, 'GET', '/cameras/overhead.mjpg?fps=3')[0] == 400
    )
    viewer_expected = 200 if controller_state.get('viewer', {}).get('available') else 502
    report.check(f'GET /viewer/ is {viewer_expected}', request(dash, 'GET', '/viewer/')[0] == viewer_expected)
    for path in ('/map', '/map/manual', '/board-map.js', '/scan', '/healthz'):
        report.check(f'GET {path}', request(dash, 'GET', path)[0] == 200)

    start = request(dash, 'POST', '/api/scan/start', {})
    if start[0] == 409:
        report.check('scan start: runtime missing or already running', True, start[2][:80].decode())
    else:
        report.check('POST /api/scan/start', start[0] == 200, start[2][:80].decode())
        time.sleep(SCAN_RUN_S)
        running = json.loads(request(dash, 'GET', '/api/scan/status')[2])
        report.check(
            'collector is collecting',
            running['phase'] == 'collecting',
            f'cycles={running.get("capture_cycles")}',
        )
        report.check('second start refused', request(dash, 'POST', '/api/scan/start', {})[0] == 409)
        request(dash, 'POST', '/api/scan/stop', {})
        time.sleep(SCAN_STOP_S)
        final = json.loads(request(dash, 'GET', '/api/scan/status')[2])
        report.check(
            'collector ended after stop', final['phase'] not in ('starting', 'collecting'), final['phase']
        )

    print(f'\n{report.failures} failed')
    return 1 if report.failures else 0


if __name__ == '__main__':
    sys.exit(main())
