"""The read-only Viser viewer under ``/viewer/``: the page over HTTP, the scene over WebSocket.

The WebSocket route is transport by nature: a refused handshake or an unreachable Viser must be
answered *before* accepting (400 / 502, as the legacy relay), which only the route can do.
"""

from fastapi import APIRouter, Request, Response, WebSocket

from arm_dashboard.api.constants import ViewerPaths
from arm_dashboard.api.errors import error_response
from arm_dashboard.api.v1 import handlers, responses
from arm_dashboard.api.v1.dependencies import ServicesDep
from arm_dashboard.api.v1.websocket import StarletteClientSocket
from arm_dashboard.core.exceptions import DashboardError

router = APIRouter(tags=['viewer'])


async def page(request: Request, services: ServicesDep) -> Response:
    return responses.html(await handlers.viewer_page(services.viewer_page, request.headers))


async def scene(websocket: WebSocket, services: ServicesDep) -> None:
    try:
        upstream = await handlers.open_viewer_upstream(services.viewer_relay, websocket.headers)
    except DashboardError as exc:
        await websocket.send_denial_response(error_response(exc))
        return
    await websocket.accept(subprotocol=upstream.subprotocol)
    await handlers.relay_viewer(services.viewer_relay, StarletteClientSocket(websocket), upstream)


for _path in ViewerPaths.ALL:
    router.add_api_route(_path, page, methods=['GET'], include_in_schema=False)
    router.add_api_websocket_route(_path, scene)
