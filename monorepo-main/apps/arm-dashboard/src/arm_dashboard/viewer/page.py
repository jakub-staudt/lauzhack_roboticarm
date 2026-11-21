import logging
from http import HTTPStatus
from typing import Final, final

import httpx

from arm_dashboard.viewer.exceptions import ViewerPayloadTooLargeError, ViewerUnavailableError

logger = logging.getLogger(__name__)


@final
class ViewerPaths:
    PAGE: Final = '/'


class ViewerPageProcessor:
    """Fetches the Viser page, bounded in size. The relay rewrites nothing in it.

    Reads the body itself rather than through ``ControllerClient``: Viser is a different upstream
    with its own errors, and the two capability packages do not import each other. If a third
    upstream appears, the bounded read moves to a shared processor.
    """

    def __init__(self, http: httpx.AsyncClient, *, limit_bytes: int) -> None:
        self._http = http
        self._limit_bytes = limit_bytes

    async def fetch(self) -> bytes:
        try:
            async with self._http.stream('GET', ViewerPaths.PAGE) as response:
                if response.status_code != HTTPStatus.OK:
                    raise ViewerUnavailableError(f'Viser page answered {response.status_code}')
                return await self._read_bounded(response)
        except httpx.TransportError as exc:
            logger.warning('Viser page failed: %s: %s', type(exc).__name__, exc)
            raise ViewerUnavailableError(f'Viser page: {type(exc).__name__}') from exc

    async def _read_bounded(self, response: httpx.Response) -> bytes:
        body = bytearray()
        async for chunk in response.aiter_bytes():
            body += chunk
            if len(body) > self._limit_bytes:
                raise ViewerPayloadTooLargeError(f'Viser page exceeded {self._limit_bytes} bytes')
        return bytes(body)
