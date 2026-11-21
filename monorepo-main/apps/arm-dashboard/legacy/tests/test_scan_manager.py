import importlib.util
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch, Mock

APP = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('scan_manager', APP/'scan_manager.py')
manager_module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(manager_module)


class ScanManagerTests(unittest.TestCase):
    def test_rejects_concurrent_sessions_and_stop_only_terminates_collector(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            python = root/'app/.venv/bin/python'
            python.parent.mkdir(parents=True)
            python.touch()
            manager = manager_module.ScanManager(root/'app', root/'data', 'http://127.0.0.1:8765')
            process = Mock()
            process.poll.return_value = None
            with patch.object(manager_module.subprocess, 'Popen', return_value=process) as launch:
                status = manager.start()
                self.assertEqual(status['phase'], 'starting')
                self.assertIn('scan_worker.py', launch.call_args.args[0][1])
                with self.assertRaises(ValueError):
                    manager.start()
                manager.stop()
                process.terminate.assert_called_once()

    def test_only_current_whitelisted_images_are_exposed(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            session = root/'data/scan-20260101-test'
            session.mkdir(parents=True)
            (session/'mosaic.jpg').write_bytes(b'jpeg')
            (session/'frames.jsonl').write_text('private capture metadata')
            manager = manager_module.ScanManager(root/'app', root/'data', 'http://127.0.0.1:8765')
            self.assertEqual(manager.image('mosaic.jpg'), b'jpeg')
            for filename in ['../secret', 'frames.jsonl', '/etc/passwd']:
                with self.assertRaises(FileNotFoundError):
                    manager.image(filename)

    def test_service_restart_does_not_report_abandoned_job_as_running(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            session = root/'data/scan-20260101-test'
            session.mkdir(parents=True)
            (session/'status.json').write_text(json.dumps({'phase': 'collecting'}))
            manager = manager_module.ScanManager(root/'app', root/'data', 'http://127.0.0.1:8765')
            self.assertEqual(manager.status()['phase'], 'interrupted')
