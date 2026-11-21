import http.client
import json
import sys
import threading
import unittest
from contextlib import contextmanager
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import server


@contextmanager
def serve(httpd):
    worker = threading.Thread(target=httpd.serve_forever, daemon=True)
    worker.start()
    try:
        yield httpd.server_address
    finally:
        httpd.shutdown()
        httpd.server_close()
        worker.join(2)


class FakeUpstream(BaseHTTPRequestHandler):
    def do_GET(self):
        self.server.paths.append(self.path)
        body = b'{"connected":true}' if self.path == '/api/state' else b'jpeg'
        self.send_response(200)
        self.send_header('Content-Type', 'application/json' if self.path == '/api/state' else 'image/jpeg')
        self.send_header('Content-Length', str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def log_message(self, *_args):
        pass


class FakeClock:
    def __init__(self):
        self.now = 0.0
        self.sleeps = []

    def monotonic(self):
        return self.now

    def sleep(self, seconds):
        self.sleeps.append(seconds)
        self.now += seconds


class FakeResponse:
    status = 200

    def __init__(self, clock):
        self.clock = clock

    def read(self, _limit=None):
        self.clock.now += 0.01
        return b'jpeg'


class FakeConnection:
    def __init__(self, clock, _address, timeout):
        self.clock = clock
        self.timeout = timeout
        self.paths = []

    def request(self, method, path):
        self.paths.append((method, path))
        self.clock.now += 0.02

    def getresponse(self):
        return FakeResponse(self.clock)

    def close(self):
        pass


class FakeSocket:
    def __init__(self):
        self.options = []
        self.timeout = None

    def settimeout(self, timeout):
        self.timeout = timeout

    def setsockopt(self, level, option, value):
        self.options.append((level, option, value))


class StopAfterFrames:
    def __init__(self, clock, completed_frames=2):
        self.clock = clock
        self.completed_frames = completed_frames
        self.frames = 0

    def write(self, payload):
        if self.frames >= self.completed_frames:
            raise BrokenPipeError()
        self.frames += 1
        self.clock.now += 0.02
        return len(payload)

    def flush(self):
        pass


class CameraDeliveryTest(unittest.TestCase):
    def test_state_advertises_relay_safe_and_opt_in_fps(self):
        upstream = ThreadingHTTPServer(('127.0.0.1', 0), FakeUpstream)
        upstream.daemon_threads = True
        upstream.paths = []
        with serve(upstream) as address, patch.object(server, 'UPSTREAM', address):
            with serve(ThreadingHTTPServer(('127.0.0.1', 0), server.Handler)) as dashboard:
                connection = http.client.HTTPConnection(*dashboard, timeout=3)
                try:
                    connection.request('GET', '/api/state?cache=bust')
                    response = connection.getresponse()
                    self.assertEqual(response.status, 200)
                    payload = json.loads(response.read())
                finally:
                    connection.close()

        self.assertEqual(payload['camera_delivery']['default_fps'], 2)
        self.assertEqual(payload['camera_delivery']['direct_default_fps'], 10)
        self.assertEqual(payload['camera_delivery']['direct_hosts'], list(dict.fromkeys((
            server.UPSTREAM[0], server.socket.gethostname(),
        ))))
        self.assertEqual(payload['camera_delivery']['available_fps'], [2, 10, 15])
        self.assertEqual(payload['camera_delivery']['max_fps'], 15)
        self.assertIn('not a sensor-to-screen latency', payload['camera_delivery']['latency_note'])
        self.assertEqual(upstream.paths, ['/api/state'])

    def test_invalid_fps_is_rejected_before_upstream_request(self):
        upstream = ThreadingHTTPServer(('127.0.0.1', 0), FakeUpstream)
        upstream.daemon_threads = True
        upstream.paths = []
        with serve(upstream) as address, patch.object(server, 'UPSTREAM', address):
            with serve(ThreadingHTTPServer(('127.0.0.1', 0), server.Handler)) as dashboard:
                connection = http.client.HTTPConnection(*dashboard, timeout=3)
                try:
                    connection.request('GET', '/cameras/overhead.mjpg?fps=3')
                    response = connection.getresponse()
                    self.assertEqual(response.status, 400)
                    response.read()
                finally:
                    connection.close()
        self.assertEqual(upstream.paths, [])

    def run_stream(self, query):
        clock = FakeClock()
        downstream = FakeSocket()
        handler = server.Handler.__new__(server.Handler)
        handler.connection = downstream
        handler.wfile = StopAfterFrames(clock)
        handler.close_connection = False
        handler.send_response = lambda _status: None
        handler.send_header = lambda _name, _value: None
        handler.end_headers = lambda: None
        handler.send_error = lambda *_args: self.fail('Valid camera fps rejected')
        fake_connection = FakeConnection(clock, server.UPSTREAM, 3)
        with patch.object(server, 'time', SimpleNamespace(monotonic=clock.monotonic, sleep=clock.sleep)), \
                patch.object(server.http.client, 'HTTPConnection', return_value=fake_connection):
            handler.stream_camera('/cameras/overhead.mjpg', query)
        return clock, fake_connection, downstream

    def test_default_and_higher_cadence_use_controlled_latest_frame_pacing(self):
        default_clock, default_upstream, _ = self.run_stream('')
        fast_clock, fast_upstream, _ = self.run_stream('fps=10')
        self.assertEqual(default_upstream.paths, [
            ('GET', '/cameras/overhead.jpg'),
            ('GET', '/cameras/overhead.jpg'),
            ('GET', '/cameras/overhead.jpg'),
        ])
        self.assertEqual(fast_upstream.paths, default_upstream.paths)
        self.assertEqual(len(default_clock.sleeps), 2)
        self.assertEqual(len(fast_clock.sleeps), 2)
        for duration in default_clock.sleeps:
            self.assertAlmostEqual(duration, 0.45)
        for duration in fast_clock.sleeps:
            self.assertAlmostEqual(duration, 0.05)

    def test_stream_applies_small_send_buffer(self):
        _clock, _upstream, downstream = self.run_stream('fps=15')
        self.assertEqual(downstream.timeout, 3)
        self.assertIn((server.socket.SOL_SOCKET, server.socket.SO_SNDBUF, 64 * 1024), downstream.options)


if __name__ == '__main__':
    unittest.main()
