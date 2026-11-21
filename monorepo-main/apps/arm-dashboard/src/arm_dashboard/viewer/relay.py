"""WebSocket relay between the browser and Viser.

The legacy proxy copied raw TCP bytes after the upgrade. ASGI never hands over the socket, so
this relays *messages* instead: each side does its own framing and ping/pong, and close codes
are carried across explicitly. Viser's scene messages (meshes) can be large, so the upstream
side has no message size limit, as the byte relay had none.
"""

import asyncio
import logging
import time
from collections.abc import Callable
from typing import Final, final

from websockets.asyncio.client import ClientConnection, connect
from websockets.exceptions import ConnectionClosed, InvalidHandshake
from websockets.typing import Subprotocol

from arm_dashboard.viewer.base import ClientSocketProtocol, SocketClosed
from arm_dashboard.viewer.exceptions import ViewerUnavailableError

logger = logging.getLogger(__name__)

Clock = Callable[[], float]


@final
class CloseCodes:
    """RFC 6455 close codes the relay sends or has to translate."""

    NORMAL: Final = 1000
    # Reported when a peer closed without a code / the connection dropped. Never sent on the wire.
    NO_STATUS: Final = 1005
    ABNORMAL: Final = 1006
    INTERNAL_ERROR: Final = 1011
    RESERVED: Final = frozenset({1005, 1006, 1015})


def sendable(code: int | None, *, fallback: int) -> int:
    """A code that may be sent in a close frame; reserved or missing codes become ``fallback``."""
    if code is None or code in CloseCodes.RESERVED:
        return fallback
    return code


class _Activity:
    def __init__(self, clock: Clock) -> None:
        self._clock = clock
        self.last = clock()

    def touch(self) -> None:
        self.last = self._clock()

    def idle_for(self) -> float:
        return self._clock() - self.last


class ViewerSocketRelay:
    """Opens the Viser socket for one browser and relays until either side ends or both go quiet."""

    def __init__(
        self,
        url: str,
        *,
        open_timeout_s: float,
        idle_timeout_s: float,
        clock: Clock = time.monotonic,
    ) -> None:
        self._url = url
        self._open_timeout_s = open_timeout_s
        self._idle_timeout_s = idle_timeout_s
        self._clock = clock

    async def connect(self, protocol: str) -> ClientConnection:
        """Open the upstream socket offering the browser's protocol. Done *before* accepting the
        browser, so an unreachable Viser can still be answered with 502."""
        try:
            return await connect(
                self._url,
                subprotocols=[Subprotocol(protocol)],
                open_timeout=self._open_timeout_s,
                max_size=None,
                # Pings are each side's own business; the legacy byte relay sent none upstream.
                ping_interval=None,
                # The legacy relay negotiated no extensions with Viser.
                compression=None,
            )
        except (OSError, TimeoutError, InvalidHandshake) as exc:
            logger.warning('Viser WebSocket unavailable: %s: %s', type(exc).__name__, exc)
            raise ViewerUnavailableError(f'Viser WebSocket: {type(exc).__name__}') from exc

    async def pump(self, client: ClientSocketProtocol, upstream: ClientConnection) -> None:
        """Relay both ways. Returns once either side closed or nothing moved for the idle timeout;
        both sides are closed on return."""
        activity = _Activity(self._clock)
        tasks = {
            asyncio.create_task(self._from_client(client, upstream, activity)),
            asyncio.create_task(self._from_upstream(upstream, client, activity)),
            asyncio.create_task(self._idle_watch(activity)),
        }
        try:
            done, _ = await asyncio.wait(tasks, return_when=asyncio.FIRST_COMPLETED)
        finally:
            for task in tasks:
                task.cancel()
            await asyncio.gather(*tasks, return_exceptions=True)
            await upstream.close()
            await client.close(sendable(upstream.close_code, fallback=CloseCodes.NORMAL))
        for task in done:
            task.result()  # surface an unexpected failure to the caller

    async def _from_client(
        self, client: ClientSocketProtocol, upstream: ClientConnection, activity: _Activity
    ) -> None:
        while True:
            message = await client.receive_message()
            if isinstance(message, SocketClosed):
                await upstream.close(code=sendable(message.code, fallback=CloseCodes.NORMAL))
                return
            activity.touch()
            try:
                await upstream.send(message)
            except ConnectionClosed:
                return

    async def _from_upstream(
        self, upstream: ClientConnection, client: ClientSocketProtocol, activity: _Activity
    ) -> None:
        try:
            async for message in upstream:
                activity.touch()
                await client.send_message(message)
        except ConnectionClosed as exc:
            logger.info('Viser WebSocket dropped: %s', exc)
        except ConnectionError:
            return  # the browser is gone; the client task sees its close
        await client.close(sendable(upstream.close_code, fallback=CloseCodes.INTERNAL_ERROR))

    async def _idle_watch(self, activity: _Activity) -> None:
        while (remaining := self._idle_timeout_s - activity.idle_for()) > 0:
            await asyncio.sleep(remaining)
        logger.info('Viewer relay idle for %.0f s; closing', self._idle_timeout_s)
