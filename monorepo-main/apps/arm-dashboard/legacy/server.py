import http.client
import json
import socket
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import parse_qs, urlsplit

from viewer_proxy import proxy_viewer

UPSTREAM = ('100.85.56.14', 8765)
# The map/scan pages moved into the FastAPI package; this legacy copy serves the same files.
STATIC_DIR = Path(__file__).resolve().parents[1] / 'src' / 'arm_dashboard' / 'static'
CAMERA_FPS = (2, 10, 15)
CAMERA_DELIVERY = {
    'default_fps': 2,
    'direct_default_fps': 10,
    'direct_hosts': list(dict.fromkeys((UPSTREAM[0], socket.gethostname()))),
    'available_fps': list(CAMERA_FPS),
    'max_fps': max(CAMERA_FPS),
    'selection_query': '?fps=<2|10|15>',
    'buffering': 'Each fresh snapshot is fetched after the previous part is written.',
    'latency_note': 'Output cadence is not a sensor-to-screen latency measurement.',
}
POST_ROUTES = {'/api/control/claim', '/api/control/heartbeat', '/api/control/release', '/api/stop', '/api/emergency-stop', '/api/emergency-stop/reset', '/api/jog', '/api/target', '/api/home', '/api/speed', '/api/torque', '/api/cameras', '/api/view', '/api/cartesian/start', '/api/cartesian/renew', '/api/cartesian/stop'}

class Handler(BaseHTTPRequestHandler):
    protocol_version = 'HTTP/1.1'

    def do_GET(self):
        parsed = urlsplit(self.path)
        route = parsed.path
        if route in {'/viewer', '/viewer/'}:
            return proxy_viewer(self)
        if route in {'/', '/panel.css', '/panel.js'}:
            self.proxy(route)
        elif route in {'/map', '/map/', '/scan', '/scan/', '/map/manual', '/board-map.js'}:
            filename = 'board-map.js' if route.endswith('.js') else 'board-map.html' if route == '/map/manual' else 'scan.html'
            body = (STATIC_DIR / filename).read_bytes()
            self.send_response(200)
            self.send_header('Content-Type', 'text/javascript; charset=utf-8' if filename.endswith('.js') else 'text/html; charset=utf-8')
            self.send_header('Content-Length', str(len(body)))
            self.send_header('Cache-Control', 'no-store')
            self.end_headers()
            self.wfile.write(body)
        elif route == '/api/scan/status':
            self.answer_json(self.server.scan_manager.status())
        elif route.startswith('/api/scan/images/'):
            try:
                body = self.server.scan_manager.image(route.rsplit('/', 1)[-1])
                self.send_response(200)
                self.send_header('Content-Type', 'image/jpeg')
                self.send_header('Content-Length', str(len(body)))
                self.send_header('Cache-Control', 'no-store')
                self.end_headers()
                self.wfile.write(body)
            except FileNotFoundError:
                self.send_error(404)
        elif route in {'/api/state', '/cameras/overhead.jpg', '/cameras/wrist.jpg', '/cameras/observer.jpg'}:
            self.proxy(route)
        elif route in {'/cameras/overhead.mjpg', '/cameras/wrist.mjpg', '/cameras/observer.mjpg'}:
            self.stream_camera(route, parsed.query)
        else:
            self.send_error(404)

    def do_POST(self):
        # Only our own page may submit controls from a browser.
        origin = self.headers.get('Origin')
        if origin and origin != 'http://' + self.headers.get('Host', ''):
            return self.send_error(403)
        if self.path not in POST_ROUTES | {'/api/scan/start', '/api/scan/stop'}:
            return self.send_error(404)
        size = int(self.headers.get('Content-Length', '0'))
        if size > 8192:
            return self.send_error(413)
        body = self.rfile.read(size)
        if self.path in {'/api/scan/start', '/api/scan/stop'}:
            try:
                result = self.server.scan_manager.start() if self.path.endswith('/start') else self.server.scan_manager.stop()
                self.answer_json(result)
            except (ValueError, RuntimeError) as exc:
                self.answer_json({'error': str(exc)}, 409)
            return
        self.proxy(self.path, body)

    def answer_json(self, data, status=200):
        body = json.dumps(data).encode()
        self.send_response(status)
        self.send_header('Content-Type', 'application/json')
        self.send_header('Content-Length', str(len(body)))
        self.send_header('Cache-Control', 'no-store')
        self.end_headers()
        self.wfile.write(body)

    def stream_camera(self, route, query=''):
        params = parse_qs(query, keep_blank_values=True)
        fps_values = params.get('fps', [])
        if set(params) - {'fps'} or len(fps_values) > 1:
            return self.send_error(400, 'Invalid camera stream options')
        fps = CAMERA_DELIVERY['default_fps']
        if fps_values:
            try:
                fps = int(fps_values[0])
            except ValueError:
                return self.send_error(400, 'fps must be one of 2, 10, or 15')
            if str(fps) != fps_values[0] or fps not in CAMERA_FPS:
                return self.send_error(400, 'fps must be one of 2, 10, or 15')

        # Pull fresh snapshots instead of forwarding every full-rate upstream frame.
        # Two 100 KB streams at 25 fps can starve control traffic in the SSH relay.
        # The query can opt into a higher cap; snapshots are fetched serially, after
        # each previous part is written, so a slow client cannot build an app queue.
        self.connection.settimeout(3)
        try:
            # Bound queued kernel data so a slow reader is less likely to see seconds
            # of already-sent frames. The platform may round this value upward.
            self.connection.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 64 * 1024)
        except (AttributeError, OSError):
            pass
        started = False
        try:
            while True:
                tick = time.monotonic()
                conn = http.client.HTTPConnection(*UPSTREAM, timeout=3)
                try:
                    conn.request('GET', route.replace('.mjpg', '.jpg'))
                    response = conn.getresponse()
                    if response.status != 200:
                        raise ConnectionError('Camera frame unavailable')
                    frame = response.read(2_000_001)
                    if len(frame) > 2_000_000:
                        raise ValueError('Camera frame too large')
                finally:
                    conn.close()
                if not started:
                    self.send_response(200)
                    self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=frame')
                    self.send_header('Cache-Control', 'no-store')
                    self.send_header('Connection', 'close')
                    self.end_headers()
                    self.close_connection = True
                    started = True
                self.wfile.write((f'--frame\r\nContent-Type: image/jpeg\r\n'
                                  f'Content-Length: {len(frame)}\r\n\r\n').encode() + frame + b'\r\n')
                self.wfile.flush()
                time.sleep(max(0, (1.0 / fps) - (time.monotonic() - tick)))
        except (OSError, ValueError, http.client.HTTPException):
            if not started:
                self.send_error(502, 'Camera unavailable')
        finally:
            self.close_connection = True

    def proxy(self, route, body=None):
        conn = http.client.HTTPConnection(*UPSTREAM, timeout=8)
        started = False
        try:
            conn.request('POST' if body is not None else 'GET', route, body=body,
                         headers={'Content-Type': 'application/json'} if body is not None else {})
            response = conn.getresponse()
            if route == '/api/state' and response.status == 200:
                state = json.loads(response.read())
                if isinstance(state, dict):
                    state['camera_delivery'] = dict(CAMERA_DELIVERY)
                if isinstance(state.get('viewer'), dict):
                    state['viewer']['proxy_url'] = '/viewer/'
                return self.answer_json(state)
            self.send_response(response.status)
            self.send_header('Content-Type', response.getheader('Content-Type', 'application/json'))
            self.send_header('Cache-Control', 'no-store')
            self.send_header('Connection', 'close')
            self.end_headers()
            started = True
            self.close_connection = True
            if route.startswith('/cameras/'):
                while True:
                    chunk = response.read1(65536)
                    if not chunk:
                        break
                    self.wfile.write(chunk)
                    self.wfile.flush()
            else:
                self.wfile.write(response.read())
        except (BrokenPipeError, ConnectionResetError):
            pass
        except Exception:
            if not started:
                self.send_error(502, 'Arm controller unavailable')
        finally:
            conn.close()

if __name__ == '__main__':
    from scan_manager import ScanManager
    server = ThreadingHTTPServer(('100.85.56.14', 8780), Handler)
    server.scan_manager = ScanManager(Path(__file__).parent, Path.home()/'ramion-scans', 'http://100.85.56.14:8765')
    server.serve_forever()
