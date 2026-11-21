import logging
from collections.abc import AsyncIterator, Callable

import httpx
import pytest

from arm_dashboard.controller import (
    ControllerClient,
    ControllerPayloadTooLargeError,
    ControllerUnavailableError,
)
from arm_dashboard.core.exceptions import UpstreamError
from arm_dashboard.core.typing import ControllerCommand
from fakes import FakeController

pytestmark = pytest.mark.anyio

BASE_URL = 'http://controller.test:8765'
Handler = Callable[[httpx.Request], httpx.Response]


def client_for(handler: Handler, *, timeout_s: float = 8.0) -> ControllerClient:
    transport = httpx.MockTransport(handler)
    return ControllerClient(httpx.AsyncClient(transport=transport, base_url=BASE_URL, timeout=timeout_s))


def refuse(request: httpx.Request) -> httpx.Response:
    raise httpx.ConnectError('connection refused', request=request)


async def chunked(chunks: list[bytes]) -> AsyncIterator[bytes]:
    for chunk in chunks:
        yield chunk


async def body_then_failure(chunks: list[bytes]) -> AsyncIterator[bytes]:
    for chunk in chunks:
        yield chunk
    raise httpx.ReadError('connection reset')


async def test_get_relays_status_content_type_and_body() -> None:
    def handler(request: httpx.Request) -> httpx.Response:
        assert (request.method, request.url.path) == ('GET', '/panel.css')
        return httpx.Response(200, headers={'Content-Type': 'text/css; charset=utf-8'}, content=b'body{}')

    response = await client_for(handler).get('/panel.css')

    assert (response.status_code, response.content_type, response.body) == (
        200,
        'text/css; charset=utf-8',
        b'body{}',
    )


async def test_missing_content_type_defaults_to_json() -> None:
    response = await client_for(lambda _request: httpx.Response(200, content=b'{}')).get('/api/state')

    assert response.content_type == 'application/json'


async def test_non_200_is_a_response_not_an_error() -> None:
    response = await client_for(lambda _request: httpx.Response(503, content=b'{"error":"x"}')).get(
        '/api/state'
    )

    assert (response.status_code, response.body) == (503, b'{"error":"x"}')


async def test_get_within_limit_is_read_whole() -> None:
    response = await client_for(lambda _request: httpx.Response(200, content=b'x' * 10)).get(
        '/f', limit_bytes=10
    )

    assert response.body == b'x' * 10


async def test_get_over_limit_raises_and_is_an_upstream_error() -> None:
    client = client_for(lambda _request: httpx.Response(200, content=b'x' * 11))

    with pytest.raises(ControllerPayloadTooLargeError) as raised:
        await client.get('/cameras/overhead.jpg', limit_bytes=10)

    assert isinstance(raised.value, UpstreamError)


async def test_per_request_timeout_overrides_client_default() -> None:
    seen: list[dict[str, float | None]] = []

    def handler(request: httpx.Request) -> httpx.Response:
        seen.append(request.extensions['timeout'])
        return httpx.Response(200)

    client = client_for(handler, timeout_s=8.0)
    await client.get('/cameras/wrist.jpg', timeout_s=3.0)
    await client.get('/api/state')

    assert [timeouts['read'] for timeouts in seen] == [3.0, 8.0]


@pytest.mark.parametrize('command', list(ControllerCommand))
async def test_post_forwards_command_body_unchanged_as_json(command: ControllerCommand) -> None:
    seen: list[httpx.Request] = []

    def handler(request: httpx.Request) -> httpx.Response:
        seen.append(request)
        return httpx.Response(200, json={'forwarded': request.url.path})

    body = b'{"token": "owner", "sequence": 7}'
    response = await client_for(handler).post(command, body)

    [request] = seen
    assert (request.method, request.url.path, request.content) == ('POST', command.value, body)
    assert request.headers['Content-Type'] == 'application/json'
    assert response.status_code == 200


async def test_post_relays_controller_refusal() -> None:
    body = b'{"error":"stale control generation"}'
    client = client_for(lambda _request: httpx.Response(409, content=body))

    response = await client.post(ControllerCommand.CONTROL_HEARTBEAT, b'{}')

    assert (response.status_code, response.body) == (409, body)


@pytest.mark.parametrize(
    'failure',
    [
        httpx.ConnectError('refused'),
        httpx.ConnectTimeout('slow'),
        httpx.ReadTimeout('slow'),
        httpx.RemoteProtocolError('garbage'),
    ],
    ids=type,
)
async def test_transport_failures_become_unavailable_and_are_logged(
    failure: httpx.TransportError, caplog: pytest.LogCaptureFixture
) -> None:
    def handler(request: httpx.Request) -> httpx.Response:
        raise failure

    client = client_for(handler)

    with caplog.at_level(logging.WARNING), pytest.raises(ControllerUnavailableError):
        await client.post(ControllerCommand.EMERGENCY_STOP, b'{}')
    with pytest.raises(ControllerUnavailableError):
        await client.get('/api/state')

    assert 'POST /api/emergency-stop' in caplog.text


async def test_failure_while_reading_body_is_unavailable() -> None:
    client = client_for(lambda _request: httpx.Response(200, content=body_then_failure([b'part'])))

    with pytest.raises(ControllerUnavailableError):
        await client.get('/api/state')


async def test_stream_relays_chunks_then_closes() -> None:
    client = client_for(
        lambda _request: httpx.Response(
            200, headers={'Content-Type': 'image/jpeg'}, content=chunked([b'ab', b'cd'])
        )
    )

    stream = await client.open('/cameras/overhead.jpg')
    received = [chunk async for chunk in stream.chunks()]

    assert (stream.status_code, stream.content_type) == (200, 'image/jpeg')
    assert b''.join(received) == b'abcd'
    assert stream._response.is_closed


async def test_stream_failure_mid_body_ends_relay_and_logs(caplog: pytest.LogCaptureFixture) -> None:
    client = client_for(lambda _request: httpx.Response(200, content=body_then_failure([b'ab'])))

    stream = await client.open('/cameras/overhead.jpg')
    with caplog.at_level(logging.WARNING):
        received = [chunk async for chunk in stream.chunks()]

    assert received == [b'ab']
    assert 'ended mid-body' in caplog.text
    assert stream._response.is_closed


async def test_open_reports_unreachable_controller() -> None:
    with pytest.raises(ControllerUnavailableError):
        await client_for(refuse).open('/')


async def test_close_is_idempotent() -> None:
    stream = await client_for(lambda _request: httpx.Response(200, content=b'x')).open('/')

    await stream.aclose()
    await stream.aclose()


async def test_against_a_real_http10_controller() -> None:
    # soarm-panel answers HTTP/1.0 and closes each connection; the pooled client must cope.
    fake = FakeController()
    with fake.serve() as (host, port):
        async with httpx.AsyncClient(base_url=f'http://{host}:{port}', timeout=3) as http:
            client = ControllerClient(http)
            replies = [await client.post(ControllerCommand.STOP, b'{}') for _ in range(3)]
            state = await client.get('/api/state')

    assert [reply.status_code for reply in replies] == [200, 200, 200]
    assert state.status_code == 200
    assert fake.paths() == ['/api/stop'] * 3 + ['/api/state']
