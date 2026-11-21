"""Composition of services for the endpoints that need more than one call.

No ``Request``/``WebSocket`` and no status codes here: routers translate, handlers compose.
Endpoints whose whole job is one service call call that service directly.
"""

from collections.abc import AsyncGenerator, AsyncIterator, Mapping

from websockets.asyncio.client import ClientConnection

from arm_dashboard.api.constants import Headers, HeaderValues
from arm_dashboard.cameras import CameraStreamService, parse_fps
from arm_dashboard.core.settings import CameraSettings
from arm_dashboard.core.typing import CameraName, ViewerHandshake
from arm_dashboard.viewer import (
    InvalidViewerHandshakeError,
    ViewerPageProcessor,
    ViewerSocketRelay,
    connection_tokens,
    validate_handshake,
)
from arm_dashboard.viewer.base import ClientSocketProtocol


async def open_camera_stream(
    stream: CameraStreamService, settings: CameraSettings, camera: CameraName, query: str
) -> AsyncIterator[bytes]:
    """Validate the options, then wait for the first part, so a dead camera still gets a 502."""
    fps = parse_fps(query, allowed=settings.available_fps, default=settings.default_fps)
    parts = stream.frames(camera, fps)
    first = await anext(parts)
    return _prepend(first, parts)


async def _prepend(first: bytes, rest: AsyncGenerator[bytes]) -> AsyncGenerator[bytes]:
    try:
        yield first
        async for part in rest:
            yield part
    finally:
        await rest.aclose()


async def viewer_page(page: ViewerPageProcessor, headers: Mapping[str, str]) -> bytes:
    """The Viser page, unless this is an upgrade uvicorn did not recognise (no ``Connection: upgrade``)."""
    if headers.get(Headers.UPGRADE, '').lower() == HeaderValues.WEBSOCKET:
        raise InvalidViewerHandshakeError('WebSocket upgrade without "Connection: upgrade"')
    return await page.fetch()


async def open_viewer_upstream(relay: ViewerSocketRelay, headers: Mapping[str, str]) -> ClientConnection:
    """Check the browser's handshake, then connect to Viser — both before the browser is accepted."""
    handshake = ViewerHandshake(
        key=headers.get(Headers.WS_KEY, ''),
        version=headers.get(Headers.WS_VERSION),
        protocol=headers.get(Headers.WS_PROTOCOL, ''),
        connection_tokens=connection_tokens(headers.get(Headers.CONNECTION.lower(), '')),
    )
    validate_handshake(handshake)
    return await relay.connect(handshake.protocol)


async def relay_viewer(
    relay: ViewerSocketRelay, client: ClientSocketProtocol, upstream: ClientConnection
) -> None:
    await relay.pump(client, upstream)
