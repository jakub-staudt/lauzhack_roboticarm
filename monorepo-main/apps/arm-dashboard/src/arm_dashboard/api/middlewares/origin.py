"""Same-origin guard for state-changing requests: every POST and every WebSocket upgrade.

Pure ASGI on purpose: a ``BaseHTTPMiddleware`` never sees WebSocket scopes. It runs before
routing, so a cross-origin POST is refused with 403 even on an unknown path (legacy order).
"""

from http import HTTPStatus

from starlette.datastructures import Headers as ScopeHeaders
from starlette.responses import JSONResponse
from starlette.types import ASGIApp, Receive, Scope, Send
from starlette.websockets import WebSocket

from arm_dashboard.api.constants import Headers
from arm_dashboard.api.middlewares.rules import Decision, same_origin

GUARDED_HTTP_METHODS = frozenset({'POST'})
DENIAL_DETAIL = 'Cross-origin request refused'


class SameOriginMiddleware:
    def __init__(self, app: ASGIApp) -> None:
        self._app = app

    async def __call__(self, scope: Scope, receive: Receive, send: Send) -> None:
        if not _is_guarded(scope):
            await self._app(scope, receive, send)
            return
        headers = ScopeHeaders(scope=scope)
        if same_origin(headers.get(Headers.ORIGIN), headers.get(Headers.HOST)) is Decision.ALLOW:
            await self._app(scope, receive, send)
            return
        denial = JSONResponse({'detail': DENIAL_DETAIL}, status_code=HTTPStatus.FORBIDDEN)
        if scope['type'] == 'websocket':
            await WebSocket(scope, receive, send).send_denial_response(denial)
        else:
            await denial(scope, receive, send)


def _is_guarded(scope: Scope) -> bool:
    if scope['type'] == 'websocket':
        return True
    return scope['type'] == 'http' and scope.get('method') in GUARDED_HTTP_METHODS
