"""Own a bounded read-only camera collection subprocess; never owns the robot."""
import json
from pathlib import Path
import subprocess
import threading
import time
import uuid


class ScanManager:
    def __init__(self, app, storage, base):
        self.app, self.storage, self.base = Path(app), Path(storage), base
        self.storage.mkdir(parents=True, exist_ok=True)
        self.lock = threading.RLock()
        self.process = None
        sessions = sorted(self.storage.glob('scan-*'))
        self.session = sessions[-1] if sessions else None

    def status(self):
        with self.lock:
            if self.session is None:
                return {'phase': 'idle', 'captured_frames': 0, 'units': 'pixels', 'metric_calibrated': False}
            try:
                result = json.loads((self.session/'status.json').read_text())
            except (OSError, ValueError):
                result = {'phase': 'starting', 'captured_frames': 0}
            result['session'] = self.session.name
            if self.process is not None and self.process.poll() is not None and result['phase'] in {'starting', 'collecting'}:
                result['phase'] = 'error'
                result['note'] = 'Collector exited before completing; inspect collector.log.'
            if self.process is None and result['phase'] in {'starting', 'collecting'}:
                result['phase'] = 'interrupted'
                result['note'] = 'A previous collector was interrupted by service restart.'
            return result

    def start(self):
        with self.lock:
            if self.process is not None and self.process.poll() is None:
                raise ValueError('A collection session is already running.')
            python = self.app/'.venv/bin/python'
            if not python.exists():
                raise RuntimeError('Scanner runtime is not installed.')
            self.session = self.storage/('scan-'+time.strftime('%Y%m%d-%H%M%S', time.gmtime())+'-'+uuid.uuid4().hex[:6])
            self.session.mkdir(mode=0o700)
            (self.session/'status.json').write_text(json.dumps({'phase': 'starting', 'captured_frames': 0}))
            with (self.session/'collector.log').open('w') as log:
                self.process = subprocess.Popen([str(python), str(self.app/'scan_worker.py'), '--base', self.base, '--output', str(self.session), '--seconds', '60'], stdout=log, stderr=log)
            return self.status()

    def stop(self):
        with self.lock:
            if self.process is not None and self.process.poll() is None:
                self.process.terminate()
            return self.status()

    def image(self, name):
        if name not in {'overhead.jpg', 'wrist.jpg', 'mosaic.jpg'}:
            raise FileNotFoundError(name)
        with self.lock:
            if self.session is None:
                raise FileNotFoundError(name)
            return (self.session/name).read_bytes()
