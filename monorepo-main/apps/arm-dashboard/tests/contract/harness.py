"""What a contract test holds: a running dashboard plus the fakes behind it."""

import http.client
import json
from dataclasses import dataclass
from pathlib import Path

from fakes import Address, FakeController, FakeViewer

IO_TIMEOUT_S = 3


@dataclass(frozen=True)
class Reply:
    status: int
    headers: http.client.HTTPMessage
    body: bytes

    def json(self) -> object:
        return json.loads(self.body)


@dataclass(frozen=True)
class Dashboard:
    target: str
    host: str
    port: int
    controller: FakeController
    viewer: FakeViewer
    scan_storage: Path

    @property
    def address(self) -> Address:
        return self.host, self.port

    @property
    def origin(self) -> str:
        """The value a browser sends as ``Origin`` for a page served by this dashboard."""
        return f'http://{self.host}:{self.port}'

    def connect(self) -> http.client.HTTPConnection:
        return http.client.HTTPConnection(self.host, self.port, timeout=IO_TIMEOUT_S)

    def request(
        self, method: str, path: str, *, headers: dict[str, str] | None = None, body: bytes | None = None
    ) -> Reply:
        connection = self.connect()
        try:
            connection.request(method, path, body=body, headers=headers or {})
            response = connection.getresponse()
            return Reply(response.status, response.headers, response.read())
        finally:
            connection.close()
