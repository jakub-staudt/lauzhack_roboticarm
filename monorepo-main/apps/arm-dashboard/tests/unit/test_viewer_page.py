from collections.abc import Callable

import httpx
import pytest

from arm_dashboard.core.exceptions import UpstreamError
from arm_dashboard.viewer import ViewerPageProcessor, ViewerPayloadTooLargeError, ViewerUnavailableError

pytestmark = pytest.mark.anyio

HTML = b'<html><title>Viser</title></html>'


def page(handler: Callable[[httpx.Request], httpx.Response], limit_bytes: int = 1024) -> ViewerPageProcessor:
    http = httpx.AsyncClient(transport=httpx.MockTransport(handler), base_url='http://viewer.test:8766')
    return ViewerPageProcessor(http, limit_bytes=limit_bytes)


async def test_page_is_fetched_from_viser_root() -> None:
    paths: list[str] = []

    def handler(request: httpx.Request) -> httpx.Response:
        paths.append(request.url.path)
        return httpx.Response(200, content=HTML)

    assert await page(handler).fetch() == HTML
    assert paths == ['/']


async def test_non_200_page_is_unavailable() -> None:
    with pytest.raises(ViewerUnavailableError, match='500'):
        await page(lambda _request: httpx.Response(500, content=b'oops')).fetch()


async def test_page_at_limit_is_accepted_and_over_limit_is_refused() -> None:
    assert await page(lambda _request: httpx.Response(200, content=b'x' * 10), limit_bytes=10).fetch()
    with pytest.raises(ViewerPayloadTooLargeError) as raised:
        await page(lambda _request: httpx.Response(200, content=b'x' * 11), limit_bytes=10).fetch()

    assert isinstance(raised.value, UpstreamError)


async def test_unreachable_viser_is_unavailable() -> None:
    def refuse(request: httpx.Request) -> httpx.Response:
        raise httpx.ConnectError('refused', request=request)

    with pytest.raises(ViewerUnavailableError):
        await page(refuse).fetch()
