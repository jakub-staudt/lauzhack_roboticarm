"""Slow-reader staleness: legacy ThreadingHTTPServer vs uvicorn + CameraStreamService.

Upstream serves 100 KB frames whose first bytes are the wall-clock time they were produced.
A client reads one MJPEG part per second at fps=15 and records how old each frame is when read.
Queued frames show up as growing staleness. Not collected by pytest; run by hand:

    ../../.venv/bin/python tests/manual/backpressure.py .

Measured 2026-11-10 (macOS, loopback): the frame age grows the same way for both, e.g.
legacy 0.0 → 6.53 s and FastAPI 0.0 → 6.55 s over 8 reads. The legacy half goes away with legacy/.
"""

import socket
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

APP = Path(sys.argv[1])
sys.path[:0] = [str(APP / 'legacy'), str(APP / 'src'), str(APP / 'tests')]


FRAME_BYTES = 100_000
READS = 8
READ_INTERVAL_S = 1.0


class Upstream(BaseHTTPRequestHandler):
    def do_GET(self) -> None:
        body = f'{time.time():.6f}'.encode().ljust(FRAME_BYTES, b'\0')
        self.send_response(200)
        self.send_header('Content-Type', 'image/jpeg')
        self.send_header('Content-Length', str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def log_message(self, *_: object) -> None:
        pass


def serve(server) -> tuple[str, int]:
    threading.Thread(target=server.serve_forever, daemon=True).start()
    return server.server_address[:2]


def slow_read(host: str, port: int, path: str) -> list[float]:
    sock = socket.create_connection((host, port), timeout=10)
    sock.sendall(f'GET {path} HTTP/1.1\r\nHost: {host}:{port}\r\n\r\n'.encode())
    reader = sock.makefile('rb')
    while reader.readline() not in (b'\r\n', b''):
        pass
    ages = []
    for _ in range(READS):
        while reader.readline().strip() != b'--frame':
            pass
        length = 0
        while line := reader.readline().strip():
            if line.lower().startswith(b'content-length'):
                length = int(line.split(b':')[1])
        frame = reader.read(length)
        ages.append(round(time.time() - float(frame[:17].rstrip(b'\0')), 2))
        time.sleep(READ_INTERVAL_S)
    sock.close()
    return ages


def legacy(upstream: tuple[str, int]) -> list[float]:
    import server

    server.UPSTREAM = upstream
    address = serve(ThreadingHTTPServer(('127.0.0.1', 0), server.Handler))
    return slow_read(*address, '/cameras/overhead.mjpg?fps=15')


def fastapi(upstream: tuple[str, int]) -> list[float]:
    """The real application, as deployed: settings → create_app → uvicorn."""
    import tempfile

    from pydantic import AnyHttpUrl

    from arm_dashboard.core.settings import ControllerSettings, ScanSettings, Settings
    from arm_dashboard.main import create_app
    from fakes import running_asgi

    with tempfile.TemporaryDirectory() as scratch:
        settings = Settings(
            _env_file=None,
            controller=ControllerSettings(base_url=AnyHttpUrl(f'http://{upstream[0]}:{upstream[1]}')),
            scan=ScanSettings(storage_dir=Path(scratch)),
        )
        with running_asgi(create_app(settings)) as (host, port):
            return slow_read(host, port, '/cameras/overhead.mjpg?fps=15')


if __name__ == '__main__':
    upstream = serve(ThreadingHTTPServer(('127.0.0.1', 0), Upstream))
    print('legacy  staleness s:', legacy(upstream))
    print('fastapi staleness s:', fastapi(upstream))
