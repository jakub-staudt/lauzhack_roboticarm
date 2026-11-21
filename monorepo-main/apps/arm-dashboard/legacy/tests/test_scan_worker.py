"""Offline scanner checks. Run with the isolated OpenCV dependencies."""
import importlib.util
import json
from pathlib import Path
import tempfile
import threading
import unittest
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

try:
    import cv2
    import numpy as np
except ImportError:
    cv2 = None
    np = None

APP = Path(__file__).resolve().parents[1]
worker = None
if cv2 is not None:
    spec = importlib.util.spec_from_file_location('scan_worker', APP/'scan_worker.py')
    worker = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(worker)


def texture(seed):
    rng = np.random.default_rng(seed)
    image = np.full((480, 640, 3), 210, np.uint8)
    for _ in range(300):
        x, y = rng.integers([10, 10], [630, 470])
        cv2.circle(image, (int(x), int(y)), int(rng.integers(2, 10)), tuple(int(x) for x in rng.integers(0, 180, 3)), -1)
    return image


@unittest.skipIf(cv2 is None, 'Scanner OpenCV dependencies are not installed')
class RegistrationTests(unittest.TestCase):
    def test_recovers_known_planar_transform(self):
        reference = texture(7)
        forward = np.array([[.9, .02, 15], [-.01, .92, 12], [.00002, .00001, 1]])
        candidate = cv2.warpPerspective(reference, forward, (640, 480))
        h, result = worker.register_plane(reference, candidate)
        self.assertTrue(result['accepted'], result)
        points = np.float32([[[100, 100]], [[300, 200]], [[500, 300]]])
        actual = cv2.perspectiveTransform(cv2.perspectiveTransform(points, forward), h)
        self.assertLess(float(np.max(np.linalg.norm(actual-points, axis=2))), 1.)

    def test_blank_frames_rejected(self):
        h, result = worker.register_plane(np.zeros((480, 640, 3), np.uint8), texture(3))
        self.assertIsNone(h)
        self.assertFalse(result['accepted'])

    def test_mosaic_retains_unobserved_holes(self):
        anchor = np.full((80, 80, 3), 80, np.uint8)
        other = np.full((80, 80, 3), 140, np.uint8)
        image, coverage = worker.render_mosaic(anchor, [(other, np.array([[1., 0., 160.], [0., 1., 0.], [0., 0., 1.]]))])
        self.assertEqual(image.shape, (80, 240, 3))
        self.assertEqual(coverage['observed_output_pixels'], 12800)
        self.assertEqual(coverage['unobserved_output_pixels'], 6400)
        self.assertTrue(np.all(image[:, 80:160] == 0))

    def test_collects_both_cameras_without_commands_or_tokens(self):
        ok, jpeg = cv2.imencode('.jpg', texture(9))
        self.assertTrue(ok)
        class Fake(BaseHTTPRequestHandler):
            def do_GET(self):
                if self.path == '/api/state':
                    body = json.dumps({'connected': True, 'loop_ok': True, 'sequence': 1, 'age_s': .01, 'measured': {'joint': 0}, 'token': 'NEVER_SAVE_THIS'}).encode()
                else:
                    body = jpeg.tobytes()
                self.server.requests.append(('GET', self.path))
                self.send_response(200)
                self.send_header('Content-Length', str(len(body)))
                self.end_headers()
                self.wfile.write(body)
            def log_message(self, *_):
                pass
        server = ThreadingHTTPServer(('127.0.0.1', 0), Fake)
        server.requests = []
        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()
        try:
            with tempfile.TemporaryDirectory() as directory:
                result = worker.collect('http://127.0.0.1:'+str(server.server_port), directory, seconds=.15, interval=.01)
                self.assertGreaterEqual(result['captured_frames'], 2)
                self.assertEqual(result['robot_motion_commands'], 0)
                self.assertFalse(result['metric_calibrated'])
                log = (Path(directory)/'frames.jsonl').read_text()
                self.assertNotIn('NEVER_SAVE_THIS', log)
                rows = [json.loads(line) for line in log.splitlines()]
                self.assertEqual({r['camera'] for r in rows}, {'overhead', 'wrist'})
                self.assertTrue((Path(directory)/'mosaic.jpg').exists())
                self.assertEqual(json.loads((Path(directory)/'status.json').read_text())['phase'], 'complete')
                self.assertTrue(all(method == 'GET' for method, _ in server.requests))
        finally:
            server.shutdown()
            server.server_close()


if __name__ == '__main__':
    unittest.main()
