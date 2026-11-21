"""Starlette's WebSocket as the viewer relay's ``ClientSocketProtocol``."""

import logging

from starlette.websockets import WebSocket, WebSocketDisconnect

from arm_dashboard.viewer.base import ClientMessage, SocketClosed
from arm_dashboard.viewer.relay import CloseCodes

logger = logging.getLogger(__name__)

DISCONNECT = 'websocket.disconnect'


class StarletteClientSocket:
    def __init__(self, websocket: WebSocket) -> None:
        self._websocket = websocket
        self._closed = False

    async def receive_message(self) -> ClientMessage:
        message = await self._websocket.receive()
        if message['type'] == DISCONNECT:
            self._closed = True
            return SocketClosed(code=message.get('code', CloseCodes.NO_STATUS))
        if message.get('bytes') is not None:
            return message['bytes']
        return message.get('text') or ''

    async def send_message(self, message: str | bytes) -> None:
        if self._closed:
            raise ConnectionError('Browser WebSocket is closed')
        try:
            if isinstance(message, bytes):
                await self._websocket.send_bytes(message)
            else:
                await self._websocket.send_text(message)
        except (WebSocketDisconnect, RuntimeError, OSError) as exc:
            self._closed = True
            raise ConnectionError('Browser WebSocket is closed') from exc

    async def close(self, code: int) -> None:
        if self._closed:
            return
        self._closed = True
        try:
            await self._websocket.close(code)
        except (RuntimeError, OSError) as exc:
            # The browser went away between the last message and this close; nothing left to tell it.
            logger.debug('Browser WebSocket already gone on close: %s', exc)
