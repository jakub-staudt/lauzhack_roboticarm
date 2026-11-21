"""Run a server (stdlib or ASGI) on a background thread for the duration of a test."""

import asyncio
import socket
import socketserver
import threading
import time
from collections.abc import Iterator
from contextlib import contextmanager

import uvicorn
from starlette.types import ASGIApp

Address = tuple[str, int]

SHUTDOWN_JOIN_TIMEOUT_S = 2
# serve_forever() notices shutdown() only between polls; the 0.5 s default dominates test time.
POLL_INTERVAL_S = 0.01
ASGI_STARTUP_TIMEOUT_S = 5.0


@contextmanager
def running(server: socketserver.BaseServer) -> Iterator[Address]:
    """Serve until the block exits, then shut down and close the listening socket."""
    worker = threading.Thread(target=server.serve_forever, args=(POLL_INTERVAL_S,), daemon=True)
    worker.start()
    try:
        host, port = server.server_address[:2]
        yield str(host), int(port)
    finally:
        server.shutdown()
        server.server_close()
        worker.join(SHUTDOWN_JOIN_TIMEOUT_S)


def unused_address() -> Address:
    """A loopback address nothing listens on, for "upstream is down" cases."""
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as probe:
        probe.bind(('127.0.0.1', 0))
        host, port = probe.getsockname()
    return host, port


@contextmanager
def running_asgi(app: ASGIApp) -> Iterator[Address]:
    """A real uvicorn on a loopback port, lifespan included. Stops gracefully on exit."""
    listener = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    listener.bind(('127.0.0.1', 0))
    server = uvicorn.Server(uvicorn.Config(app, log_level='warning', timeout_graceful_shutdown=1))
    worker = threading.Thread(target=lambda: asyncio.run(server.serve(sockets=[listener])), daemon=True)
    worker.start()
    deadline = time.monotonic() + ASGI_STARTUP_TIMEOUT_S
    while not server.started:
        if time.monotonic() > deadline or not worker.is_alive():
            raise RuntimeError('ASGI server did not start')
        time.sleep(POLL_INTERVAL_S)
    try:
        host, port = listener.getsockname()[:2]
        yield str(host), int(port)
    finally:
        server.should_exit = True
        worker.join(SHUTDOWN_JOIN_TIMEOUT_S * 2)
        listener.close()
