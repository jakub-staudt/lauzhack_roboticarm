import asyncio
from collections.abc import AsyncGenerator
from contextlib import asynccontextmanager
from dataclasses import dataclass, field

import pytest
from websockets.asyncio.server import ServerConnection, serve
from websockets.exceptions import ConnectionClosed
from websockets.typing import Subprotocol

from arm_dashboard.viewer import ViewerSocketRelay, ViewerUnavailableError
from arm_dashboard.viewer.base import ClientMessage, SocketClosed
from fakes import unused_address
from fakes.websocket import VISER_PROTOCOL

pytestmark = pytest.mark.anyio

# Above the websockets default max_size (1 MiB): a scene with meshes must still get through.
LARGE_SCENE_BYTES = 2 * 1024 * 1024
WAIT_S = 3.0


@dataclass
class FakeViser:
    """A real WebSocket server: sends ``scene`` on connect, echoes, and obeys two commands."""

    url: str = ''
    subprotocols: list[str | None] = field(default_factory=list)
    close_codes: list[int | None] = field(default_factory=list)

    async def handle(self, websocket: ServerConnection) -> None:
        self.subprotocols.append(websocket.subprotocol)
        await websocket.send(b'scene')
        try:
            async for message in websocket:
                if message == 'large':
                    await websocket.send(b'x' * LARGE_SCENE_BYTES)
                elif message == 'close-4000':
                    await websocket.close(4000, 'scene reset')
                else:
                    await websocket.send(message)
        except ConnectionClosed:
            pass
        self.close_codes.append(websocket.close_code)


@dataclass
class FakeBrowser:
    """ClientSocketProtocol over a queue: the test types, the relay reads."""

    inbox: asyncio.Queue[ClientMessage] = field(default_factory=asyncio.Queue)
    sent: list[str | bytes] = field(default_factory=list)
    close_codes: list[int] = field(default_factory=list)

    async def receive_message(self) -> ClientMessage:
        return await self.inbox.get()

    async def send_message(self, message: str | bytes) -> None:
        if self.close_codes:
            raise ConnectionError('browser gone')
        self.sent.append(message)

    async def close(self, code: int) -> None:
        if not self.close_codes:
            self.close_codes.append(code)

    async def wait_for(self, count: int) -> None:
        async with asyncio.timeout(WAIT_S):
            while len(self.sent) < count:
                await asyncio.sleep(0.01)


@asynccontextmanager
async def running_viser() -> AsyncGenerator[FakeViser]:
    viser = FakeViser()
    async with serve(viser.handle, '127.0.0.1', 0, subprotocols=[Subprotocol(VISER_PROTOCOL)]) as server:
        host, port = next(iter(server.sockets)).getsockname()[:2]
        viser.url = f'ws://{host}:{port}/'
        yield viser


def relay_to(viser: FakeViser, *, idle_timeout_s: float = 30.0) -> ViewerSocketRelay:
    return ViewerSocketRelay(viser.url, open_timeout_s=WAIT_S, idle_timeout_s=idle_timeout_s)


@asynccontextmanager
async def relaying(
    viser: FakeViser, **options: float
) -> AsyncGenerator[tuple[FakeBrowser, asyncio.Task[None]]]:
    relay = relay_to(viser, **options)
    browser = FakeBrowser()
    upstream = await relay.connect(VISER_PROTOCOL)
    pump = asyncio.create_task(relay.pump(browser, upstream))
    try:
        yield browser, pump
    finally:
        if not pump.done():
            browser.inbox.put_nowait(SocketClosed(code=1000))
        async with asyncio.timeout(WAIT_S):
            await pump


async def test_connect_offers_the_browser_protocol() -> None:
    async with running_viser() as viser:
        upstream = await relay_to(viser).connect(VISER_PROTOCOL)
        await upstream.close()

    assert upstream.subprotocol == VISER_PROTOCOL
    assert viser.subprotocols == [VISER_PROTOCOL]


async def test_unreachable_viser_is_unavailable() -> None:
    host, port = unused_address()
    relay = ViewerSocketRelay(f'ws://{host}:{port}/', open_timeout_s=WAIT_S, idle_timeout_s=30)

    with pytest.raises(ViewerUnavailableError):
        await relay.connect(VISER_PROTOCOL)


async def test_messages_flow_both_ways_with_their_type() -> None:
    async with running_viser() as viser, relaying(viser) as (browser, _pump):
        await browser.wait_for(1)
        await browser.inbox.put(b'\x00\xff')
        await browser.inbox.put('text frame')
        await browser.wait_for(3)

    assert browser.sent == [b'scene', b'\x00\xff', 'text frame']


async def test_large_scene_message_is_not_truncated_or_refused() -> None:
    async with running_viser() as viser, relaying(viser) as (browser, _pump):
        await browser.inbox.put('large')
        await browser.wait_for(2)

    assert len(browser.sent[1]) == LARGE_SCENE_BYTES


@pytest.mark.parametrize(('browser_code', 'upstream_code'), [(1001, 1001), (1005, 1000)])
async def test_browser_close_is_carried_to_viser(browser_code: int, upstream_code: int) -> None:
    async with running_viser() as viser:
        async with relaying(viser) as (browser, pump):
            await browser.wait_for(1)
            await browser.inbox.put(SocketClosed(code=browser_code))
            async with asyncio.timeout(WAIT_S):
                await pump
        async with asyncio.timeout(WAIT_S):
            while not viser.close_codes:
                await asyncio.sleep(0.01)

    assert viser.close_codes == [upstream_code]


async def test_viser_close_is_carried_to_the_browser() -> None:
    async with running_viser() as viser, relaying(viser) as (browser, pump):
        await browser.inbox.put('close-4000')
        async with asyncio.timeout(WAIT_S):
            await pump

    assert browser.close_codes == [4000]


async def test_idle_relay_closes_both_sides() -> None:
    async with running_viser() as viser, relaying(viser, idle_timeout_s=0.2) as (browser, pump):
        async with asyncio.timeout(WAIT_S):
            await pump

    assert browser.close_codes == [1000]
    assert viser.close_codes == [1000]
