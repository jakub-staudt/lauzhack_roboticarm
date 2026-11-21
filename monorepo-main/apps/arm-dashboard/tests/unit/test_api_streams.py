"""A browser that disconnects mid-stream must release the upstream: no orphaned controller work."""

import socket
import socketserver
import threading
import time
from collections.abc import Iterator
from contextlib import contextmanager
from pathlib import Path

from pydantic import AnyHttpUrl

from arm_dashboard.core.settings import ControllerSettings, ScanSettings, Settings, ViewerSettings
from arm_dashboard.main import create_app
from fakes import Address, FakeController, running, running_asgi, unused_address

WAIT_S = 3.0


class EndlessUpstream(socketserver.StreamRequestHandler):
    """Answers any GET with one chunk of an endless chunked body, then waits for the peer to hang up."""

    def handle(self) -> None:
        self.rfile.readline()
        self.wfile.write(
            b'HTTP/1.1 200 OK\r\nContent-Type: image/jpeg\r\nTransfer-Encoding: chunked\r\n\r\n4\r\npart\r\n'
        )
        self.wfile.flush()
        self.connection.settimeout(10)
        try:
            while self.connection.recv(4096):
                pass
        except OSError:
            pass
        self.server.closed.set()  # type: ignore[attr-defined]


def settings_for(controller: Address, scan_dir: Path) -> Settings:
    return Settings(
        _env_file=None,
        controller=ControllerSettings(base_url=AnyHttpUrl(f'http://{controller[0]}:{controller[1]}')),
        viewer=ViewerSettings(base_url=AnyHttpUrl(f'http://{unused_address()[0]}:{unused_address()[1]}')),
        scan=ScanSettings(storage_dir=scan_dir),
    )


@contextmanager
def browser(address: Address, path: str) -> Iterator[socket.socket]:
    client = socket.create_connection(address, timeout=WAIT_S)
    try:
        client.sendall(f'GET {path} HTTP/1.1\r\nHost: {address[0]}:{address[1]}\r\n\r\n'.encode())
        yield client
    finally:
        client.close()


def read_until(client: socket.socket, marker: bytes) -> None:
    received = b''
    while marker not in received:
        chunk = client.recv(4096)
        assert chunk, 'connection closed before the marker arrived'
        received += chunk


def test_relayed_stream_releases_the_controller_when_the_browser_leaves(tmp_path: Path) -> None:
    upstream = socketserver.ThreadingTCPServer(('127.0.0.1', 0), EndlessUpstream)
    upstream.daemon_threads = True
    upstream.closed = threading.Event()  # type: ignore[attr-defined]
    with running(upstream) as controller, running_asgi(create_app(settings_for(controller, tmp_path))) as app:
        with browser(app, '/cameras/overhead.jpg') as client:
            read_until(client, b'part')

        assert upstream.closed.wait(WAIT_S), 'upstream connection still open after the browser left'  # type: ignore[attr-defined]


def test_mjpeg_stops_fetching_snapshots_when_the_browser_leaves(tmp_path: Path) -> None:
    fake = FakeController()
    with fake.serve() as controller, running_asgi(create_app(settings_for(controller, tmp_path))) as app:
        with browser(app, '/cameras/wrist.mjpg?fps=15') as client:
            read_until(client, b'--frame')
        time.sleep(0.3)  # let the server notice the disconnect
        after_leaving = len(fake.paths())
        time.sleep(0.5)  # at 15 fps a live stream would fetch ~7 more snapshots here

    assert len(fake.paths()) == after_leaving
