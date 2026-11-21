import time
from dataclasses import dataclass
from http import HTTPStatus

import httpx

from arm_dashboard.scan.worker.constants import CollectionSettings
from arm_dashboard.scan.worker.exceptions import FrameFetchError


@dataclass(frozen=True)
class Fetched:
    data: bytes
    request_ms: float


class FrameFetcher:
    """GETs one cached controller resource (state or snapshot), bounded and timed.

    Synchronous on purpose: the collector is a plain process with a small thread pool, and
    ``httpx.Client`` is safe to share across those threads.
    """

    def __init__(
        self, http: httpx.Client, *, limit_bytes: int = CollectionSettings.FRAME_LIMIT_BYTES
    ) -> None:
        self._http = http
        self._limit_bytes = limit_bytes

    def fetch(self, path: str) -> Fetched:
        start = time.monotonic()
        with self._http.stream('GET', path) as response:
            if response.status_code != HTTPStatus.OK:
                raise FrameFetchError(f'{path} answered {response.status_code}')
            data = bytearray()
            for chunk in response.iter_bytes():
                data += chunk
                if len(data) > self._limit_bytes:
                    raise FrameFetchError('Capture too large')
        elapsed_ms = (time.monotonic() - start) * 1000
        return Fetched(bytes(data), round(elapsed_ms, CollectionSettings.REQUEST_MS_DIGITS))
