"""In-process stand-in for the read-only Viser viewer on :8766.

Plain GET answers HTML. A WebSocket upgrade answers 101 with the requested subprotocol, sends
an initial ``scene`` frame in the same write, then echoes every client frame back unmasked
until it echoes a close frame.
"""

import http.client
import socketserver
import threading
from collections.abc import Iterator
from contextlib import contextmanager
from dataclasses import dataclass, field
from email.message import Message

from fakes.servers import Address, running
from fakes.websocket import OPCODE_BINARY, OPCODE_CLOSE, accept_key, encode_frame, read_frame

VIEWER_HTML = b'<html><title>Viser</title></html>'
INITIAL_FRAME = encode_frame(OPCODE_BINARY, b'scene')
IO_TIMEOUT_S = 3


@dataclass
class FakeViewer:
    html_status: int = 200
    html_body: bytes = VIEWER_HTML
    requests: list[tuple[str, Message]] = field(default_factory=list)
    _lock: threading.Lock = field(default_factory=threading.Lock)

    def record(self, request_line: str, headers: Message) -> None:
        with self._lock:
            self.requests.append((request_line, headers))

    def request_lines(self) -> list[str]:
        with self._lock:
            return [line for line, _ in self.requests]

    @contextmanager
    def serve(self) -> Iterator[Address]:
        server = socketserver.ThreadingTCPServer(('127.0.0.1', 0), _handler_for(self))
        server.daemon_threads = True
        with running(server) as address:
            yield address


def _handler_for(fake: FakeViewer) -> type[socketserver.StreamRequestHandler]:
    class Handler(socketserver.StreamRequestHandler):
        def handle(self) -> None:
            self.connection.settimeout(IO_TIMEOUT_S)
            request_line = self.rfile.readline().decode().strip()
            headers = http.client.parse_headers(self.rfile)
            fake.record(request_line, headers)
            if headers.get('Upgrade', '').lower() == 'websocket':
                self._echo_websocket(headers)
            else:
                self._send_html()

        def _send_html(self) -> None:
            self.connection.sendall(
                f'HTTP/1.1 {fake.html_status} X\r\nContent-Type: text/html\r\n'
                f'Content-Length: {len(fake.html_body)}\r\nConnection: close\r\n\r\n'.encode()
                + fake.html_body
            )

        def _echo_websocket(self, headers: Message) -> None:
            self.connection.sendall(
                b'HTTP/1.1 101 Switching Protocols\r\nUpgrade: websocket\r\nConnection: Upgrade\r\n'
                + f'Sec-WebSocket-Accept: {accept_key(headers["Sec-WebSocket-Key"])}\r\n'.encode()
                + f'Sec-WebSocket-Protocol: {headers["Sec-WebSocket-Protocol"]}\r\n\r\n'.encode()
                + INITIAL_FRAME
            )
            while True:
                try:
                    opcode, payload = read_frame(self.rfile)
                except (EOFError, OSError):
                    return
                self.connection.sendall(encode_frame(opcode, payload))
                if opcode == OPCODE_CLOSE:
                    return

    return Handler
