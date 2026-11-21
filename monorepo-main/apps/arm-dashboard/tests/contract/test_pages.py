"""Dashboard assets, legacy proxy assets, and the 404 fallback."""

import pytest

from fakes import unused_address
from harness import Dashboard
from targets import LEGACY, STATIC_DIR

NO_STORE = 'no-store'
WORKBENCH_ASSETS = {
    '/': ('text/html; charset=utf-8', b'<html>workbench</html>'),
    '/panel.css': ('text/css; charset=utf-8', b'body{}'),
    '/panel.js': ('text/javascript; charset=utf-8', b'poll()'),
}
OPERATOR_FILES = {'/': 'panel.html', '/panel.css': 'panel.css', '/panel.js': 'panel.js'}
LOCAL_PAGES = {
    '/map': 'scan.html',
    '/map/': 'scan.html',
    '/scan': 'scan.html',
    '/scan/': 'scan.html',
    '/map/manual': 'board-map.html',
    '/board-map.js': 'board-map.js',
}
HTML = 'text/html; charset=utf-8'
JAVASCRIPT = 'text/javascript; charset=utf-8'


@pytest.mark.parametrize('route', list(WORKBENCH_ASSETS))
def test_operator_assets_are_served(dashboard: Dashboard, route: str) -> None:
    content_type, body = WORKBENCH_ASSETS[route]

    reply = dashboard.request('GET', route)

    assert reply.status == 200
    assert reply.headers['Content-Type'] == content_type
    assert reply.headers['Cache-Control'] == NO_STORE
    if dashboard.target == LEGACY:
        assert reply.body == body
        assert dashboard.controller.paths('GET') == [route]
    else:
        assert reply.body == (STATIC_DIR / OPERATOR_FILES[route]).read_bytes()
        assert dashboard.controller.requests == []


def test_operator_page_loads_with_the_controller_down(start_dashboard) -> None:
    dashboard = start_dashboard(controller_address=unused_address())

    if dashboard.target == LEGACY:
        assert dashboard.request('GET', '/').status == 502
    else:
        for route in OPERATOR_FILES:
            assert dashboard.request('GET', route).status == 200
        assert dashboard.request('GET', '/api/state').status == 502


@pytest.mark.parametrize('route', list(LOCAL_PAGES))
def test_local_pages_are_served_without_the_controller(dashboard: Dashboard, route: str) -> None:
    filename = LOCAL_PAGES[route]

    reply = dashboard.request('GET', route)

    assert reply.status == 200
    assert reply.body == (STATIC_DIR / filename).read_bytes()
    assert reply.headers['Content-Type'] == (JAVASCRIPT if filename.endswith('.js') else HTML)
    assert reply.headers['Cache-Control'] == NO_STORE
    assert dashboard.controller.requests == []


@pytest.mark.parametrize(
    'route', ['/private', '/index.html', '/map/manual/', '/scan.html', '/api', '/api/unknown']
)
def test_unknown_get_routes_are_404(dashboard: Dashboard, route: str) -> None:
    assert dashboard.request('GET', route).status == 404
    assert dashboard.controller.requests == []
