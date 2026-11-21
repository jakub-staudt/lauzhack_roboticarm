"""``GET /api/state``: proxied controller state enriched with camera delivery and viewer URL."""

import json
import socket

from arm_dashboard.core.typing import ControllerCommand
from fakes import CannedResponse, unused_address
from harness import Dashboard
from targets import FASTAPI

STATE = '/api/state'


def expected_camera_delivery(controller_host: str) -> dict[str, object]:
    """Verbatim: panel.js reads these keys and shows the notes."""
    return {
        'default_fps': 2,
        'direct_default_fps': 10,
        'direct_hosts': list(dict.fromkeys((controller_host, socket.gethostname()))),
        'available_fps': [2, 10, 15],
        'max_fps': 15,
        'selection_query': '?fps=<2|10|15>',
        'buffering': 'Each fresh snapshot is fetched after the previous part is written.',
        'latency_note': 'Output cadence is not a sensor-to-screen latency measurement.',
    }


def test_state_is_enriched_with_camera_delivery_and_viewer_proxy_url(dashboard: Dashboard) -> None:
    dashboard.controller.respond_json(
        'GET', STATE, {'connected': True, 'viewer': {'available': True, 'port': 8766}}
    )

    reply = dashboard.request('GET', STATE)

    assert reply.status == 200
    assert reply.headers['Content-Type'] == 'application/json'
    assert reply.headers['Cache-Control'] == 'no-store'
    expected = {
        'connected': True,
        'viewer': {'available': True, 'port': 8766, 'proxy_url': '/viewer/'},
        'camera_delivery': expected_camera_delivery('127.0.0.1'),
    }
    if dashboard.target == FASTAPI:
        expected['operator_commands'] = [command.value for command in ControllerCommand]
    assert reply.json() == expected


def test_state_without_viewer_gets_no_viewer_key(dashboard: Dashboard) -> None:
    dashboard.controller.respond_json('GET', STATE, {'connected': False})

    state = dashboard.request('GET', STATE).json()

    assert 'viewer' not in state
    assert state['camera_delivery']['default_fps'] == 2


def test_non_object_viewer_is_left_untouched(dashboard: Dashboard) -> None:
    dashboard.controller.respond_json('GET', STATE, {'connected': True, 'viewer': None})

    assert dashboard.request('GET', STATE).json()['viewer'] is None


def test_query_string_is_not_forwarded(dashboard: Dashboard) -> None:
    assert dashboard.request('GET', STATE + '?cache=bust').status == 200
    assert dashboard.controller.paths('GET') == [STATE]


def test_non_200_state_is_passed_through(dashboard: Dashboard) -> None:
    body = json.dumps({'error': 'starting'}).encode()
    dashboard.controller.respond('GET', STATE, CannedResponse(status=503, body=body))

    reply = dashboard.request('GET', STATE)

    assert reply.status == 503
    assert reply.body == body


def test_invalid_json_state_is_502(dashboard: Dashboard) -> None:
    dashboard.controller.respond('GET', STATE, CannedResponse(body=b'not json'))

    assert dashboard.request('GET', STATE).status == 502


def test_non_object_state_is_502(dashboard: Dashboard) -> None:
    # Legacy crashes on `list.get` and reports 502. Kept for parity.
    dashboard.controller.respond_json('GET', STATE, [1, 2, 3])

    assert dashboard.request('GET', STATE).status == 502


def test_state_is_502_when_controller_is_down(start_dashboard) -> None:
    dashboard = start_dashboard(controller_address=unused_address())

    assert dashboard.request('GET', STATE).status == 502
