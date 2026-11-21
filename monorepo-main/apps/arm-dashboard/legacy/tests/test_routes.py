import http.client
import importlib.util
import json
from pathlib import Path
import sys
import threading
import time
import unittest
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

APP = Path(__file__).resolve().parents[1]
STATIC = APP.parent / 'src' / 'arm_dashboard' / 'static'
sys.path.insert(0, str(APP))
spec = importlib.util.spec_from_file_location('dashboard', APP / 'server.py')
dashboard = importlib.util.module_from_spec(spec)
spec.loader.exec_module(dashboard)


class FakeController(BaseHTTPRequestHandler):
    gets = []
    posted = []
    posted_bodies = []

    def do_GET(self):
        self.gets.append(self.path)
        assets = {'/': b'<html>workbench</html>', '/panel.css': b'body{}', '/panel.js': b'poll()'}
        state = b'{"connected":true,"viewer":{"available":true,"port":8766}}'
        body = b'fake-jpeg' if self.path.endswith(".jpg") else assets.get(self.path, state if self.path == '/api/state' else b'{"connected":true}')
        self.send_response(200)
        content_type = ('image/jpeg' if self.path.endswith('.jpg') else
                        'text/html; charset=utf-8' if self.path == '/' else
                        'text/css; charset=utf-8' if self.path.endswith('.css') else
                        'text/javascript; charset=utf-8' if self.path.endswith('.js') else
                        'application/json')
        self.send_header('Content-Type', content_type)
        self.send_header('Content-Length', str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_POST(self):
        self.posted.append(self.path)
        size = int(self.headers.get('Content-Length', '0'))
        self.posted_bodies.append((self.path, self.rfile.read(size)))
        body = json.dumps({'forwarded': self.path}).encode()
        self.send_response(200)
        self.send_header('Content-Type', 'application/json')
        self.send_header('Content-Length', str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def log_message(self, *_):
        pass


class DashboardTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.upstream = ThreadingHTTPServer(('127.0.0.1', 0), FakeController)
        dashboard.UPSTREAM = cls.upstream.server_address
        cls.server = ThreadingHTTPServer(('127.0.0.1', 0), dashboard.Handler)
        for server in [cls.upstream, cls.server]:
            threading.Thread(target=server.serve_forever, daemon=True).start()

    @classmethod
    def tearDownClass(cls):
        for server in [cls.server, cls.upstream]:
            server.shutdown()
            server.server_close()

    def request(self, method, path, headers=None, body=None):
        conn = http.client.HTTPConnection(*self.server.server_address, timeout=3)
        try:
            conn.request(method, path, body=body, headers=headers or {})
            response = conn.getresponse()
            return response.status, response.read()
        finally:
            conn.close()

    def test_proxies_workbench_shell_and_assets(self):
        FakeController.gets.clear()
        expected = {'/': b'<html>workbench</html>', '/panel.css': b'body{}', '/panel.js': b'poll()'}
        for route, content in expected.items():
            with self.subTest(route=route):
                status, body = self.request('GET', route)
                self.assertEqual(status, 200)
                self.assertEqual(body, content)
        proxied_assets = [route for route in FakeController.gets if route in expected]
        self.assertEqual(proxied_assets, list(expected))

    def test_proxies_controller_state(self):
        status, body = self.request('GET', '/api/state')
        self.assertEqual(status, 200)
        state = json.loads(body)
        self.assertTrue(state['connected'])
        self.assertEqual(state['viewer']['proxy_url'], '/viewer/')

    def test_camera_stream_is_rate_limited(self):
        conn = http.client.HTTPConnection(*self.server.server_address, timeout=3)
        try:
            conn.request('GET', '/cameras/overhead.mjpg')
            response = conn.getresponse()
            self.assertEqual(response.status, 200)
            self.assertIn('multipart/x-mixed-replace', response.getheader('Content-Type'))
            arrivals = []
            for _ in range(2):
                while response.readline().strip() != b'--frame':
                    pass
                headers = {}
                while True:
                    line = response.readline().strip()
                    if not line:
                        break
                    key, value = line.decode().split(':', 1)
                    headers[key.lower()] = value.strip()
                self.assertEqual(response.read(int(headers['content-length'])), b'fake-jpeg')
                arrivals.append(time.monotonic())
            self.assertGreaterEqual(arrivals[1] - arrivals[0], .4)
        finally:
            conn.close()

    def test_serves_board_mapping_assets(self):
        for route, name in [('/map', 'scan.html'), ('/map/manual', 'board-map.html'), ('/board-map.js', 'board-map.js')]:
            with self.subTest(route=route):
                status, body = self.request('GET', route)
                self.assertEqual(status, 200)
                self.assertEqual(body, (STATIC / name).read_bytes())

    def test_serves_cached_camera_snapshot(self):
        status, body = self.request('GET', '/cameras/overhead.jpg')
        self.assertEqual(status, 200)
        self.assertEqual(body, b'fake-jpeg')

    def test_observer_snapshot_is_allowlisted(self):
        status, body = self.request('GET', '/cameras/observer.jpg')
        self.assertEqual(status, 200)
        self.assertEqual(body, b'fake-jpeg')

    def test_observer_mjpeg_is_allowlisted(self):
        conn = http.client.HTTPConnection(*self.server.server_address, timeout=3)
        try:
            conn.request('GET', '/cameras/observer.mjpg')
            response = conn.getresponse()
            self.assertEqual(response.status, 200)
            self.assertIn('multipart/x-mixed-replace', response.getheader('Content-Type'))
        finally:
            conn.close()

    def test_emergency_stop_and_reset_are_forwarded(self):
        FakeController.posted.clear()
        for route in ['/api/emergency-stop', '/api/emergency-stop/reset']:
            with self.subTest(route=route):
                status, body = self.request('POST', route, {'Content-Type': 'application/json'}, b'{}')
                self.assertEqual(status, 200)
                self.assertEqual(json.loads(body)['forwarded'], route)
        self.assertEqual(FakeController.posted, ['/api/emergency-stop', '/api/emergency-stop/reset'])

    def test_manual_workbench_actions_are_allowlisted(self):
        FakeController.posted.clear()
        routes = ['/api/control/claim', '/api/control/heartbeat', '/api/control/release',
                  '/api/jog', '/api/target', '/api/home', '/api/speed', '/api/torque',
                  '/api/cameras', '/api/view']
        for route in routes:
            with self.subTest(route=route):
                status, body = self.request('POST', route, {'Content-Type': 'application/json'}, b'{}')
                self.assertEqual(status, 200)
                self.assertEqual(json.loads(body)['forwarded'], route)
        self.assertEqual(FakeController.posted, routes)

    def test_unknown_route_rejected(self):
        self.assertEqual(self.request('GET', '/private')[0], 404)

    def test_cartesian_lifecycle_preserves_request_body_through_proxy(self):
        FakeController.posted_bodies.clear()
        commands = [
            ('/api/cartesian/start', {'token': 'test-owner', 'direction': 'forward',
                                     'profile': 'slow', 'sequence': 1, 'observation_sequence': 20}),
            ('/api/cartesian/renew', {'token': 'test-owner', 'motion_id': 'test-motion', 'sequence': 2}),
            ('/api/cartesian/stop', {'token': 'test-owner', 'sequence': 3}),
        ]
        for route, payload in commands:
            body = json.dumps(payload).encode()
            with self.subTest(route=route):
                status, response = self.request('POST', route, {'Content-Type': 'application/json'}, body)
                self.assertEqual(status, 200)
                self.assertEqual(json.loads(response)['forwarded'], route)
        self.assertEqual(FakeController.posted_bodies,
                         [(route, json.dumps(payload).encode()) for route, payload in commands])

    def test_cross_origin_cartesian_commands_never_reach_controller(self):
        FakeController.posted.clear()
        for command in ('start', 'renew', 'stop'):
            status, _ = self.request('POST', '/api/cartesian/' + command,
                                     {'Origin': 'http://untrusted.invalid'})
            self.assertEqual(status, 403)
        self.assertEqual(FakeController.posted, [])

    def test_cross_origin_control_rejected(self):
        self.assertEqual(self.request('POST', '/api/stop', {'Origin': 'http://untrusted.invalid'})[0], 403)

    def test_unsupported_command_rejected(self):
        self.assertEqual(self.request('POST', '/api/arbitrary')[0], 404)


if __name__ == '__main__':
    unittest.main()
