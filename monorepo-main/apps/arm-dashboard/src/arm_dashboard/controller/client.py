"""HTTP transport to soarm-panel. A processor: it moves bytes and reports transport failures,
and knows nothing about what the state means or which commands are allowed.

soarm-panel answers HTTP/1.0 and closes every connection, so the httpx pool never reuses one;
each request is a fresh connection, exactly as in the legacy proxy.
"""

import logging
from collections.abc import AsyncIterator
from typing import Final, final

import httpx

from arm_dashboard.controller.exceptions import ControllerPayloadTooLargeError, ControllerUnavailableError
from arm_dashboard.core.typing.controller import ControllerCommand, UpstreamResponse

logger = logging.getLogger(__name__)


@final
class ControllerHeaders:
    CONTENT_TYPE: Final = 'Content-Type'
    # soarm-panel rejects any POST that is not application/json (its CSRF guard), so commands are
    # always forwarded as JSON whatever the browser sent. Also the fallback for a missing header.
    JSON: Final = 'application/json'


def _unavailable(method: str, path: str, exc: httpx.HTTPError) -> ControllerUnavailableError:
    logger.warning('Controller %s %s failed: %s: %s', method, path, type(exc).__name__, exc)
    return ControllerUnavailableError(f'{method} {path}: {type(exc).__name__}')


class UpstreamStream:
    """An open controller response whose headers have arrived and whose body has not.

    Not data: it holds a live connection until ``aclose()``. Read it once, either whole with
    ``read()`` (before anything was sent downstream, so failures raise) or piecewise with
    ``chunks()`` (while relaying, when headers are already sent and failures can only end it).
    """

    def __init__(self, response: httpx.Response) -> None:
        self._response = response

    @property
    def status_code(self) -> int:
        return self._response.status_code

    @property
    def content_type(self) -> str:
        return self._response.headers.get(ControllerHeaders.CONTENT_TYPE, ControllerHeaders.JSON)

    async def read(self, limit_bytes: int | None = None) -> bytes:
        body = bytearray()
        try:
            async for chunk in self._response.aiter_bytes():
                body += chunk
                if limit_bytes is not None and len(body) > limit_bytes:
                    raise ControllerPayloadTooLargeError(f'GET {self._path} exceeded {limit_bytes} bytes')
        except httpx.TransportError as exc:
            raise _unavailable('GET', self._path, exc) from exc
        return bytes(body)

    async def chunks(self) -> AsyncIterator[bytes]:
        """Relay the body, then close. A transport failure mid-body is logged and ends the relay."""
        try:
            async for chunk in self._response.aiter_bytes():
                yield chunk
        except httpx.TransportError as exc:
            logger.warning('Controller GET %s ended mid-body: %s: %s', self._path, type(exc).__name__, exc)
        finally:
            await self.aclose()

    async def aclose(self) -> None:
        """Idempotent."""
        await self._response.aclose()

    @property
    def _path(self) -> str:
        return self._response.request.url.path


class ControllerClient:
    """GET and POST to soarm-panel through an ``httpx.AsyncClient`` owned by the app lifespan."""

    def __init__(self, http: httpx.AsyncClient) -> None:
        self._http = http

    async def open(self, path: str, *, timeout_s: float | None = None) -> UpstreamStream:
        """Send a GET and return as soon as the headers arrive. The caller must close the stream."""
        if timeout_s is None:
            request = self._http.build_request('GET', path)
        else:
            request = self._http.build_request('GET', path, timeout=timeout_s)
        return UpstreamStream(await self._send(request, stream=True))

    async def get(
        self, path: str, *, limit_bytes: int | None = None, timeout_s: float | None = None
    ) -> UpstreamResponse:
        stream = await self.open(path, timeout_s=timeout_s)
        try:
            body = await stream.read(limit_bytes)
        finally:
            await stream.aclose()
        return UpstreamResponse(status_code=stream.status_code, content_type=stream.content_type, body=body)

    async def post(self, command: ControllerCommand, body: bytes) -> UpstreamResponse:
        request = self._http.build_request(
            'POST',
            command.value,
            content=body,
            headers={ControllerHeaders.CONTENT_TYPE: ControllerHeaders.JSON},
        )
        response = await self._send(request)
        return UpstreamResponse(
            status_code=response.status_code,
            content_type=response.headers.get(ControllerHeaders.CONTENT_TYPE, ControllerHeaders.JSON),
            body=response.content,
        )

    async def _send(self, request: httpx.Request, *, stream: bool = False) -> httpx.Response:
        try:
            return await self._http.send(request, stream=stream)
        except httpx.TransportError as exc:
            raise _unavailable(request.method, request.url.path, exc) from exc
