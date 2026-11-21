"""The collection scenario end to end, against a fake controller, and the collector process itself."""

import json
import signal
import subprocess
import sys
import threading
import time
from pathlib import Path

import pytest

cv2 = pytest.importorskip('cv2', reason='collector tests need the scan extra')
import httpx  # noqa: E402
import numpy as np  # noqa: E402  (installed with OpenCV)
from pydantic import AnyHttpUrl  # noqa: E402

import legacy  # noqa: E402
from arm_dashboard.core.typing import ScanPhase, ScanStatus, ScanWorkerArgs  # noqa: E402
from arm_dashboard.scan.worker.collector import ScanCollectionOrchestrator  # noqa: E402
from arm_dashboard.scan.worker.fetcher import FrameFetcher  # noqa: E402
from arm_dashboard.scan.worker.frames import FrameDecoder  # noqa: E402
from arm_dashboard.scan.worker.mosaic import MosaicRenderer  # noqa: E402
from arm_dashboard.scan.worker.registration import PlaneRegistrationProcessor  # noqa: E402
from arm_dashboard.scan.worker.writer import ArtifactWriter  # noqa: E402
from fakes import CannedResponse, FakeController  # noqa: E402

# Two capture cycles: at t≈0 and t≈0.6, the third would start after the 1 s budget.
SECONDS = 1.0
INTERVAL_S = 0.6
SECRET = 'NEVER_SAVE_THIS'
TIMING_FIELDS = {'elapsed_s', 'request_ms', 'started_utc'}
WAIT_S = 10.0


def board(seed: int) -> np.ndarray:
    rng = np.random.default_rng(seed)
    image = np.full((480, 640, 3), 210, np.uint8)
    for _ in range(300):
        x, y = rng.integers([10, 10], [630, 470])
        cv2.circle(
            image,
            (int(x), int(y)),
            int(rng.integers(2, 10)),
            tuple(int(c) for c in rng.integers(0, 180, 3)),
            -1,
        )
    return image


def jpeg(image: np.ndarray) -> bytes:
    ok, encoded = cv2.imencode('.jpg', image)
    assert ok
    return encoded.tobytes()


def controller_with_cameras() -> FakeController:
    overhead = board(7)
    wrist = cv2.warpPerspective(
        overhead, np.array([[0.9, 0.02, 15], [-0.01, 0.92, 12], [2e-5, 1e-5, 1]]), (640, 480)
    )
    fake = FakeController()
    state = {
        'connected': True,
        'loop_ok': True,
        'sequence': 1,
        'age_s': 0.01,
        'measured': {'j': 0},
        'token': SECRET,
    }
    fake.respond_json('GET', '/api/state', state)
    fake.respond(
        'GET', '/cameras/overhead.jpg', CannedResponse(content_type='image/jpeg', body=jpeg(overhead))
    )
    fake.respond('GET', '/cameras/wrist.jpg', CannedResponse(content_type='image/jpeg', body=jpeg(wrist)))
    return fake


def run_new(base_url: str, output: Path, stop: threading.Event | None = None) -> dict[str, object]:
    with httpx.Client(base_url=base_url, timeout=3) as http:
        orchestrator = ScanCollectionOrchestrator(
            FrameFetcher(http),
            FrameDecoder(),
            PlaneRegistrationProcessor(),
            MosaicRenderer(),
            ArtifactWriter(output),
            interval_s=INTERVAL_S,
        )
        args = ScanWorkerArgs(base_url=AnyHttpUrl(base_url), output_dir=output, seconds=SECONDS)
        orchestrator.run(args, stop or threading.Event())
    return json.loads((output / 'status.json').read_text())


def rows(output: Path) -> list[dict[str, object]]:
    return [json.loads(line) for line in (output / 'frames.jsonl').read_text().splitlines()]


def without_timing(value: object) -> object:
    if isinstance(value, dict):
        return {key: without_timing(item) for key, item in value.items() if key not in TIMING_FIELDS}
    return value


def test_collection_is_identical_to_legacy_except_timing(tmp_path: Path) -> None:
    fake = controller_with_cameras()
    with fake.serve() as (host, port):
        base = f'http://{host}:{port}'
        legacy_status = legacy.load('scan_worker').collect(
            base, tmp_path / 'legacy', seconds=SECONDS, interval=INTERVAL_S
        )
        status = run_new(base, tmp_path / 'new')

    assert status['capture_cycles'] == legacy_status['capture_cycles'] == 2
    assert without_timing(status) == without_timing(legacy_status)
    assert [without_timing(row) for row in rows(tmp_path / 'new')] == [
        without_timing(row) for row in rows(tmp_path / 'legacy')
    ]
    files = sorted(path.name for path in (tmp_path / 'new').iterdir())
    assert files == sorted(path.name for path in (tmp_path / 'legacy').iterdir())
    for name in ('overhead.jpg', 'wrist.jpg', 'mosaic.jpg', '00000-overhead.jpg', '00001-wrist.jpg'):
        assert (tmp_path / 'new' / name).read_bytes() == (tmp_path / 'legacy' / name).read_bytes(), name


def test_collection_is_read_only_and_never_logs_tokens(tmp_path: Path) -> None:
    fake = controller_with_cameras()
    with fake.serve() as (host, port):
        status = run_new(f'http://{host}:{port}', tmp_path)

    log = (tmp_path / 'frames.jsonl').read_text()
    assert SECRET not in log
    assert {row['camera'] for row in rows(tmp_path)} == {'overhead', 'wrist'}
    assert {request.method for request in fake.requests} == {'GET'}
    assert (status['phase'], status['robot_motion_commands'], status['metric_calibrated']) == (
        'complete',
        0,
        False,
    )
    assert (tmp_path / 'mosaic.jpg').exists()
    assert not list(tmp_path.glob('*.next'))


def test_status_file_is_readable_by_the_dashboard(tmp_path: Path) -> None:
    fake = controller_with_cameras()
    with fake.serve() as (host, port):
        written = run_new(f'http://{host}:{port}', tmp_path)

    status = ScanStatus.model_validate(written)

    assert status.phase is ScanPhase.COMPLETE
    assert status.model_dump(mode='json', exclude_unset=True) == written


def test_failed_captures_are_logged_and_counted(tmp_path: Path) -> None:
    fake = controller_with_cameras()
    fake.respond('GET', '/cameras/wrist.jpg', CannedResponse(status=503, body=b'{}'))
    with fake.serve() as (host, port):
        status = run_new(f'http://{host}:{port}', tmp_path)

    wrist_rows = [row for row in rows(tmp_path) if row['camera'] == 'wrist']
    assert status['errors'] == len(wrist_rows) == 2
    assert all('503' in str(row['error']) for row in wrist_rows)


def test_collection_stopped_before_any_capture_is_an_error(tmp_path: Path) -> None:
    stop = threading.Event()
    stop.set()
    fake = controller_with_cameras()
    with fake.serve() as (host, port):
        status = run_new(f'http://{host}:{port}', tmp_path, stop)

    assert status['phase'] == 'error'
    assert fake.requests == []


def test_collector_process_stops_gracefully_on_sigterm(tmp_path: Path) -> None:
    fake = controller_with_cameras()
    with fake.serve() as (host, port):
        process = subprocess.Popen(
            [
                sys.executable,
                '-m',
                'arm_dashboard.scan.worker',
                '--base',
                f'http://{host}:{port}',
                '--output',
                str(tmp_path),
                '--seconds',
                '30',
            ],
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
        )
        try:
            deadline = time.monotonic() + WAIT_S
            while time.monotonic() < deadline:
                status_file = tmp_path / 'status.json'
                if status_file.exists() and json.loads(status_file.read_text()).get('captured_frames', 0) > 0:
                    break
                time.sleep(0.05)
            process.send_signal(signal.SIGTERM)
            output, _ = process.communicate(timeout=WAIT_S)
        finally:
            process.kill()

    assert process.returncode == 0, output.decode()
    assert json.loads((tmp_path / 'status.json').read_text())['phase'] == 'stopped'


@pytest.mark.parametrize(
    'arguments', [['--output', '/tmp/x'], ['--base', 'http://h', '--output', '/tmp/x', '--seconds', '500']]
)
def test_collector_process_refuses_invalid_arguments(arguments: list[str]) -> None:
    process = subprocess.run(
        [sys.executable, '-m', 'arm_dashboard.scan.worker', *arguments], capture_output=True
    )

    assert process.returncode != 0
