"""Camera snapshots (proxied) and MJPEG streams (paced, one fresh snapshot per part)."""

import http.client
import time

import pytest

from fakes import CAMERAS, CannedResponse, camera_frame, unused_address
from harness import Dashboard

MJPEG_CONTENT_TYPE = 'multipart/x-mixed-replace; boundary=frame'
FRAME_LIMIT = 2_000_000
DEFAULT_PERIOD_S = 1 / 2
# Generous lower bound on the gap between parts at the default cadence; the upstream is local.
DEFAULT_MIN_GAP_S = 0.4


def read_part(response: http.client.HTTPResponse) -> bytes:
    """Read one ``--frame`` part and return its JPEG payload."""
    while (line := response.readline()) and line.strip() != b'--frame':
        pass
    headers: dict[str, str] = {}
    while line := response.readline().strip():
        name, value = line.decode().split(':', 1)
        headers[name.lower()] = value.strip()
    assert headers['content-type'] == 'image/jpeg'
    return response.read(int(headers['content-length']))


@pytest.mark.parametrize('camera', CAMERAS)
def test_snapshot_is_proxied(dashboard: Dashboard, camera: str) -> None:
    reply = dashboard.request('GET', f'/cameras/{camera}.jpg')

    assert reply.status == 200
    assert reply.body == camera_frame(camera)
    assert reply.headers['Content-Type'] == 'image/jpeg'
    assert reply.headers['Cache-Control'] == 'no-store'


def test_snapshot_upstream_status_is_passed_through(dashboard: Dashboard) -> None:
    dashboard.controller.respond('GET', '/cameras/observer.jpg', CannedResponse(status=503, body=b'{}'))

    assert dashboard.request('GET', '/cameras/observer.jpg').status == 503


@pytest.mark.parametrize('route', ['/cameras/unknown.jpg', '/cameras/unknown.mjpg', '/cameras/overhead.png'])
def test_unknown_camera_routes_are_404(dashboard: Dashboard, route: str) -> None:
    assert dashboard.request('GET', route).status == 404
    assert dashboard.controller.requests == []


@pytest.mark.parametrize('camera', CAMERAS)
def test_mjpeg_stream_sends_fresh_snapshots(dashboard: Dashboard, camera: str) -> None:
    connection = dashboard.connect()
    try:
        connection.request('GET', f'/cameras/{camera}.mjpg')
        response = connection.getresponse()

        assert response.status == 200
        assert response.getheader('Content-Type') == MJPEG_CONTENT_TYPE
        assert response.getheader('Cache-Control') == 'no-store'
        assert read_part(response) == camera_frame(camera)
        assert read_part(response) == camera_frame(camera)
    finally:
        connection.close()
    # Each part is a new upstream snapshot, not a relayed upstream stream. Filtered by camera:
    # legacy reads its upstream from a module global, so a previous test's stream thread can
    # fetch once more from this test's fake before it notices its client is gone.
    assert dashboard.controller.paths('GET').count(f'/cameras/{camera}.jpg') >= 2


def test_mjpeg_default_cadence_is_two_fps(dashboard: Dashboard) -> None:
    connection = dashboard.connect()
    try:
        connection.request('GET', '/cameras/overhead.mjpg')
        response = connection.getresponse()
        read_part(response)
        first = time.monotonic()
        read_part(response)
        gap = time.monotonic() - first
    finally:
        connection.close()

    assert DEFAULT_MIN_GAP_S <= gap < DEFAULT_PERIOD_S * 2


def test_mjpeg_opt_in_cadence_is_faster(dashboard: Dashboard) -> None:
    connection = dashboard.connect()
    try:
        connection.request('GET', '/cameras/overhead.mjpg?fps=15')
        response = connection.getresponse()
        read_part(response)
        first = time.monotonic()
        for _ in range(3):
            read_part(response)
        elapsed = time.monotonic() - first
    finally:
        connection.close()

    assert elapsed < DEFAULT_MIN_GAP_S


@pytest.mark.parametrize('query', ['fps=2', 'fps=10', 'fps=15'])
def test_mjpeg_accepts_allowed_fps(dashboard: Dashboard, query: str) -> None:
    connection = dashboard.connect()
    try:
        connection.request('GET', f'/cameras/wrist.mjpg?{query}')
        assert connection.getresponse().status == 200
    finally:
        connection.close()


@pytest.mark.parametrize(
    'query',
    [
        'fps=3',
        'fps=02',
        'fps=2.0',
        'fps=+2',
        'fps=',
        'fps=two',
        'fps=2&fps=2',
        'fps=2&quality=1',
        'quality=1',
    ],
)
def test_mjpeg_rejects_invalid_options_before_contacting_controller(dashboard: Dashboard, query: str) -> None:
    assert dashboard.request('GET', f'/cameras/overhead.mjpg?{query}').status == 400
    assert dashboard.controller.requests == []


def test_mjpeg_is_502_when_first_frame_is_unavailable(dashboard: Dashboard) -> None:
    dashboard.controller.respond('GET', '/cameras/overhead.jpg', CannedResponse(status=503, body=b''))

    assert dashboard.request('GET', '/cameras/overhead.mjpg').status == 502


def test_mjpeg_is_502_when_frame_exceeds_limit(dashboard: Dashboard) -> None:
    oversized = CannedResponse(content_type='image/jpeg', body=b'\xff' * (FRAME_LIMIT + 1))
    dashboard.controller.respond('GET', '/cameras/overhead.jpg', oversized)

    assert dashboard.request('GET', '/cameras/overhead.mjpg').status == 502


def test_mjpeg_is_502_when_controller_is_down(start_dashboard) -> None:
    dashboard = start_dashboard(controller_address=unused_address())

    assert dashboard.request('GET', '/cameras/overhead.mjpg').status == 502
