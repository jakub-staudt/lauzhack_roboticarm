"""Latency and footprint: legacy stdlib proxy vs the FastAPI app, each in its own process.

Both run against the same in-process fake controller (100 KB camera frames). Not collected by
pytest; run by hand from apps/arm-dashboard:

    ../../.venv/bin/python tests/manual/latency.py

Measured 2026-11-10 (macOS, loopback, 300 requests): heartbeat p50/p95 legacy 0.30/0.39 ms vs
FastAPI 0.96/1.11 ms; RSS idle 24 vs 63 MB; OS threads with 3 MJPEG 4 vs 8. The legacy half goes
away with legacy/.
"""

import http.client
import os
import socket
import statistics
import subprocess
import sys
import tempfile
import threading
import time
from collections.abc import Callable, Generator
from contextlib import contextmanager
from pathlib import Path

APP_DIR = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(APP_DIR / 'tests'))

from fakes import CannedResponse, FakeController  # noqa: E402

REQUESTS = 300
FRAME_BYTES = 100_000
MJPEG_CLIENTS = 3
MJPEG_SECONDS = 3.0
STARTUP_TIMEOUT_S = 10.0
HEARTBEAT_BODY = b'{"token": "owner", "generation": 1}'

LEGACY_LAUNCHER = """
import sys
from http.server import ThreadingHTTPServer
from pathlib import Path
sys.path.insert(0, sys.argv[1])
import server, scan_manager, viewer_proxy
server.UPSTREAM = ('127.0.0.1', int(sys.argv[2]))
viewer_proxy.VIEWER_UPSTREAM = ('127.0.0.1', 1)
httpd = ThreadingHTTPServer(('127.0.0.1', int(sys.argv[3])), server.Handler)
httpd.daemon_threads = True
httpd.scan_manager = scan_manager.ScanManager(Path(sys.argv[4]), Path(sys.argv[4]), 'http://127.0.0.1:1')
httpd.serve_forever()
"""


def free_port() -> int:
    with socket.socket() as probe:
        probe.bind(('127.0.0.1', 0))
        return probe.getsockname()[1]


def wait_until_listening(port: int) -> None:
    deadline = time.monotonic() + STARTUP_TIMEOUT_S
    while time.monotonic() < deadline:
        try:
            socket.create_connection(('127.0.0.1', port), timeout=0.2).close()
            return
        except OSError:
            time.sleep(0.05)
    raise RuntimeError(f'nothing listens on {port}')


@contextmanager
def launched(
    kind: str, controller_port: int, scratch: Path
) -> Generator[tuple[subprocess.Popen[bytes], int]]:
    port = free_port()
    if kind == 'legacy':
        command = [
            sys.executable,
            '-c',
            LEGACY_LAUNCHER,
            str(APP_DIR / 'legacy'),
            str(controller_port),
            str(port),
            str(scratch),
        ]
        env = os.environ.copy()
    else:
        command = [str(Path(sys.executable).with_name('arm-dashboard'))]
        env = os.environ | {
            'RAMION_DASHBOARD_SERVER__PORT': str(port),
            'RAMION_DASHBOARD_SERVER__LOG_LEVEL': 'WARNING',
            'RAMION_DASHBOARD_CONTROLLER__BASE_URL': f'http://127.0.0.1:{controller_port}',
            'RAMION_DASHBOARD_VIEWER__BASE_URL': 'http://127.0.0.1:1',
            'RAMION_DASHBOARD_SCAN__STORAGE_DIR': str(scratch),
        }
    process = subprocess.Popen(command, env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    try:
        wait_until_listening(port)
        yield process, port
    finally:
        process.terminate()
        process.wait(5)


def timed(port: int, method: str, path: str, body: bytes | None) -> float:
    start = time.perf_counter()
    connection = http.client.HTTPConnection('127.0.0.1', port, timeout=5)
    connection.request(method, path, body=body, headers={'Content-Type': 'application/json'} if body else {})
    response = connection.getresponse()
    response.read()
    connection.close()
    assert response.status == 200, (path, response.status)
    return (time.perf_counter() - start) * 1000


def percentiles(samples: list[float]) -> str:
    ordered = sorted(samples)
    p95 = ordered[int(len(ordered) * 0.95) - 1]
    return f'{statistics.median(ordered):.2f} / {p95:.2f}'


def rss_mb(pid: int) -> float:
    return int(subprocess.check_output(['ps', '-o', 'rss=', '-p', str(pid)]).strip()) / 1024


def thread_count(pid: int) -> int:
    if Path(f'/proc/{pid}/task').exists():
        return len(list(Path(f'/proc/{pid}/task').iterdir()))
    return len(subprocess.check_output(['ps', '-M', '-p', str(pid)]).decode().splitlines()) - 1


def read_mjpeg(port: int, stop: threading.Event) -> None:
    connection = socket.create_connection(('127.0.0.1', port), timeout=5)
    connection.sendall(
        f'GET /cameras/overhead.mjpg?fps=15 HTTP/1.1\r\nHost: 127.0.0.1:{port}\r\n\r\n'.encode()
    )
    while not stop.is_set() and connection.recv(65536):
        pass
    connection.close()


def measure(kind: str, controller_port: int) -> dict[str, str]:
    with (
        tempfile.TemporaryDirectory() as scratch,
        launched(kind, controller_port, Path(scratch)) as (process, port),
    ):
        for _ in range(20):  # warm up imports, pools and caches
            timed(port, 'GET', '/api/state', None)
        heartbeat = [timed(port, 'POST', '/api/control/heartbeat', HEARTBEAT_BODY) for _ in range(REQUESTS)]
        state = [timed(port, 'GET', '/api/state', None) for _ in range(REQUESTS)]
        idle_rss, idle_threads = rss_mb(process.pid), thread_count(process.pid)
        stop = threading.Event()
        readers = [threading.Thread(target=read_mjpeg, args=(port, stop)) for _ in range(MJPEG_CLIENTS)]
        for reader in readers:
            reader.start()
        time.sleep(MJPEG_SECONDS)
        busy_rss, busy_threads = rss_mb(process.pid), thread_count(process.pid)
        stop.set()
        for reader in readers:
            reader.join(5)
    return {
        'heartbeat p50 / p95, ms': percentiles(heartbeat),
        'state p50 / p95, ms': percentiles(state),
        'RSS idle, MB': f'{idle_rss:.1f}',
        f'RSS with {MJPEG_CLIENTS} MJPEG, MB': f'{busy_rss:.1f}',
        'OS threads idle': str(idle_threads),
        f'OS threads with {MJPEG_CLIENTS} MJPEG': str(busy_threads),
    }


def main(kinds: Callable[[], tuple[str, ...]] = lambda: ('legacy', 'fastapi')) -> None:
    fake = FakeController()
    fake.respond(
        'GET', '/cameras/overhead.jpg', CannedResponse(content_type='image/jpeg', body=b'\xff' * FRAME_BYTES)
    )
    with fake.serve() as (_, controller_port):
        results = {kind: measure(kind, controller_port) for kind in kinds()}
    print('| Metric | ' + ' | '.join(results) + ' |')
    print('|---|' + '---|' * len(results))
    for metric in next(iter(results.values())):
        print(f'| {metric} | ' + ' | '.join(result[metric] for result in results.values()) + ' |')


if __name__ == '__main__':
    main()
