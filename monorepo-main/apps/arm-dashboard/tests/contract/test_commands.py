"""POST commands forwarded to the controller. This is the safety path: STOP, claim, heartbeat.

Check order (legacy ``do_POST``): cross-origin → 403, then unknown path → 404, then body over the
limit → 413, then forward.
"""

import json

import pytest

from fakes import CannedResponse, unused_address
from harness import Dashboard

COMMANDS = [
    '/api/control/claim',
    '/api/control/heartbeat',
    '/api/control/release',
    '/api/stop',
    '/api/emergency-stop',
    '/api/emergency-stop/reset',
    '/api/jog',
    '/api/target',
    '/api/home',
    '/api/speed',
    '/api/torque',
    '/api/cameras',
    '/api/view',
    '/api/cartesian/start',
    '/api/cartesian/renew',
    '/api/cartesian/stop',
]
BODY_LIMIT = 8192
UNTRUSTED_ORIGIN = 'http://untrusted.invalid'


@pytest.mark.parametrize('command', COMMANDS)
def test_command_is_forwarded_with_body_unchanged(dashboard: Dashboard, command: str) -> None:
    body = json.dumps({'token': 'test-owner', 'sequence': 7, 'path': command}).encode()

    reply = dashboard.request('POST', command, headers={'Content-Type': 'application/json'}, body=body)

    assert reply.status == 200
    assert reply.json() == {'forwarded': command}
    [forwarded] = dashboard.controller.requests
    assert (forwarded.method, forwarded.path, forwarded.body) == ('POST', command, body)


def test_forwarded_content_type_is_always_json(dashboard: Dashboard) -> None:
    dashboard.request('POST', '/api/stop', headers={'Content-Type': 'text/plain'}, body=b'{}')

    [forwarded] = dashboard.controller.requests
    assert forwarded.headers['Content-Type'] == 'application/json'


def test_controller_status_and_body_are_passed_through(dashboard: Dashboard) -> None:
    # panel.js drops control on 401/403/409 from the controller; those must reach the browser.
    body = b'{"error":"stale control generation"}'
    dashboard.controller.respond('POST', '/api/control/heartbeat', CannedResponse(status=409, body=body))

    reply = dashboard.request('POST', '/api/control/heartbeat', body=b'{}')

    assert reply.status == 409
    assert reply.body == body
    assert reply.headers['Content-Type'] == 'application/json'


def test_same_origin_browser_request_is_allowed(dashboard: Dashboard) -> None:
    assert (
        dashboard.request('POST', '/api/stop', headers={'Origin': dashboard.origin}, body=b'{}').status == 200
    )


@pytest.mark.parametrize(
    'command', ['/api/stop', '/api/cartesian/start', '/api/cartesian/renew', '/api/cartesian/stop']
)
def test_cross_origin_command_never_reaches_controller(dashboard: Dashboard, command: str) -> None:
    assert dashboard.request('POST', command, headers={'Origin': UNTRUSTED_ORIGIN}, body=b'{}').status == 403
    assert dashboard.controller.requests == []


def test_cross_origin_is_checked_before_route(dashboard: Dashboard) -> None:
    assert (
        dashboard.request('POST', '/api/arbitrary', headers={'Origin': UNTRUSTED_ORIGIN}, body=b'{}').status
        == 403
    )


def test_unknown_command_is_404(dashboard: Dashboard) -> None:
    assert dashboard.request('POST', '/api/arbitrary', body=b'{}').status == 404
    assert dashboard.controller.requests == []


def test_body_at_limit_is_forwarded(dashboard: Dashboard) -> None:
    assert dashboard.request('POST', '/api/target', body=b' ' * BODY_LIMIT).status == 200


def test_body_over_limit_is_413_and_not_forwarded(dashboard: Dashboard) -> None:
    assert dashboard.request('POST', '/api/target', body=b' ' * (BODY_LIMIT + 1)).status == 413
    assert dashboard.controller.requests == []


def test_command_route_does_not_answer_get(dashboard: Dashboard) -> None:
    assert dashboard.request('GET', '/api/stop').status == 404
    assert dashboard.controller.requests == []


def test_command_is_502_when_controller_is_down(start_dashboard) -> None:
    dashboard = start_dashboard(controller_address=unused_address())

    assert dashboard.request('POST', '/api/emergency-stop', body=b'{}').status == 502
