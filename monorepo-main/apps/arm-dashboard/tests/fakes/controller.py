"""In-process stand-in for soarm-panel :8765.

Records every request and answers from a route table that a test can override. It never
touches hardware and knows nothing about the dashboard under test.
"""

import json
import threading
from collections.abc import Iterator
from contextlib import contextmanager
from dataclasses import dataclass, field
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

from fakes.servers import Address, running

CAMERAS = ('overhead', 'wrist', 'observer')
JSON = 'application/json'
DEFAULT_STATE = {'connected': True, 'viewer': {'available': True, 'port': 8766}}


@dataclass(frozen=True)
class RecordedRequest:
    method: str
    path: str
    headers: dict[str, str]
    body: bytes


@dataclass(frozen=True)
class CannedResponse:
    status: int = 200
    content_type: str = JSON
    body: bytes = b'{}'


def camera_frame(camera: str) -> bytes:
    return f'fake-jpeg-{camera}'.encode()


def _default_routes() -> dict[tuple[str, str], CannedResponse]:
    routes = {
        ('GET', '/'): CannedResponse(content_type='text/html; charset=utf-8', body=b'<html>workbench</html>'),
        ('GET', '/panel.css'): CannedResponse(content_type='text/css; charset=utf-8', body=b'body{}'),
        ('GET', '/panel.js'): CannedResponse(content_type='text/javascript; charset=utf-8', body=b'poll()'),
        ('GET', '/api/state'): CannedResponse(body=json.dumps(DEFAULT_STATE).encode()),
    }
    for camera in CAMERAS:
        routes['GET', f'/cameras/{camera}.jpg'] = CannedResponse(
            content_type='image/jpeg', body=camera_frame(camera)
        )
    return routes


@dataclass
class FakeController:
    """Unknown GETs answer 404; any POST answers 200 with ``{"forwarded": <path>}`` unless overridden."""

    routes: dict[tuple[str, str], CannedResponse] = field(default_factory=_default_routes)
    requests: list[RecordedRequest] = field(default_factory=list)
    _lock: threading.Lock = field(default_factory=threading.Lock)

    def respond(self, method: str, path: str, response: CannedResponse) -> None:
        self.routes[method, path] = response

    def respond_json(self, method: str, path: str, payload: object, status: int = 200) -> None:
        self.respond(method, path, CannedResponse(status=status, body=json.dumps(payload).encode()))

    def paths(self, method: str | None = None) -> list[str]:
        with self._lock:
            return [request.path for request in self.requests if method in (None, request.method)]

    def record(self, request: RecordedRequest) -> None:
        with self._lock:
            self.requests.append(request)

    def answer(self, method: str, path: str) -> CannedResponse:
        if (method, path) in self.routes:
            return self.routes[method, path]
        if method == 'POST':
            return CannedResponse(body=json.dumps({'forwarded': path}).encode())
        return CannedResponse(status=404, body=b'{"error":"not found"}')

    @contextmanager
    def serve(self) -> Iterator[Address]:
        server = ThreadingHTTPServer(('127.0.0.1', 0), _handler_for(self))
        server.daemon_threads = True
        with running(server) as address:
            yield address


def _handler_for(fake: FakeController) -> type[BaseHTTPRequestHandler]:
    class Handler(BaseHTTPRequestHandler):
        def do_GET(self) -> None:
            self._reply('GET', b'')

        def do_POST(self) -> None:
            self._reply('POST', self.rfile.read(int(self.headers.get('Content-Length', '0'))))

        def _reply(self, method: str, body: bytes) -> None:
            fake.record(RecordedRequest(method, self.path, dict(self.headers.items()), body))
            response = fake.answer(method, self.path)
            self.send_response(response.status)
            self.send_header('Content-Type', response.content_type)
            self.send_header('Content-Length', str(len(response.body)))
            self.end_headers()
            self.wfile.write(response.body)

        def log_message(self, *_args: object) -> None:
            pass

    return Handler
