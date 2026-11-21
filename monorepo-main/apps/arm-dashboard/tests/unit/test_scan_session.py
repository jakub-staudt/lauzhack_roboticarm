import asyncio
import json
import sys
from dataclasses import dataclass, field
from pathlib import Path

import pytest
from pydantic import AnyHttpUrl

from arm_dashboard.core.typing import ScanArtifact, ScanPhase, ScanWorkerArgs
from arm_dashboard.scan import (
    ScanAlreadyRunningError,
    ScanArtifactNotFoundError,
    ScanNotes,
    ScanRuntimeMissingError,
    ScanSessionService,
    ScanStorage,
    ScanWorkerRunner,
)

pytestmark = pytest.mark.anyio

CONTROLLER_URL = 'http://127.0.0.1:8765'
DURATION_S = 60.0


@dataclass
class FakeRunner:
    """Duck-typed ScanWorkerRunner: the session rules only read its three flags."""

    runtime: bool = True
    has_started: bool = False
    is_running: bool = False
    has_exited: bool = False
    started: list[ScanWorkerArgs] = field(default_factory=list)
    shutdowns: int = 0

    def ensure_runtime(self) -> None:
        if not self.runtime:
            raise ScanRuntimeMissingError('missing')

    async def start(self, args: ScanWorkerArgs, log_path: Path) -> None:
        await asyncio.sleep(0)  # let a concurrent caller run, as a real spawn would
        self.started.append(args)
        self.has_started, self.is_running = True, True

    def terminate(self) -> None:
        if self.is_running:
            self.exit()

    def exit(self) -> None:
        self.is_running, self.has_exited = False, True

    async def shutdown(self) -> None:
        self.shutdowns += 1


def seed(
    root: Path, name: str, status: dict[str, object] | None, files: dict[str, bytes] | None = None
) -> Path:
    session = root / name
    session.mkdir(parents=True)
    if status is not None:
        (session / 'status.json').write_text(json.dumps(status))
    for filename, content in (files or {}).items():
        (session / filename).write_bytes(content)
    return session


async def service(root: Path, runner: object) -> ScanSessionService:
    sessions = ScanSessionService(
        ScanStorage(root),
        runner,  # type: ignore[arg-type]
        controller_url=AnyHttpUrl(CONTROLLER_URL),
        duration_s=DURATION_S,
    )
    await sessions.restore()
    return sessions


def dumped(status: object) -> dict[str, object]:
    return status.model_dump(mode='json', exclude_unset=True)  # type: ignore[attr-defined]


async def test_idle_without_sessions(tmp_path: Path) -> None:
    sessions = await service(tmp_path / 'scans', FakeRunner())

    assert dumped(await sessions.status()) == {
        'phase': 'idle',
        'captured_frames': 0,
        'units': 'pixels',
        'metric_calibrated': False,
    }
    assert (tmp_path / 'scans').is_dir()


async def test_newest_session_on_disk_is_adopted_with_collector_fields(tmp_path: Path) -> None:
    seed(tmp_path, 'scan-20260101-000000-aaaaaa', {'phase': 'complete', 'captured_frames': 1})
    seed(tmp_path, 'scan-20260102-000000-bbbbbb', {'phase': 'complete', 'captured_frames': 5, 'errors': 0})

    status = await (await service(tmp_path, FakeRunner())).status()

    assert dumped(status) == {
        'phase': 'complete',
        'captured_frames': 5,
        'errors': 0,
        'session': 'scan-20260102-000000-bbbbbb',
    }


@pytest.mark.parametrize('phase', ['starting', 'collecting'])
async def test_collection_lost_to_a_restart_is_interrupted(tmp_path: Path, phase: str) -> None:
    seed(tmp_path, 'scan-20260101-000000-aaaaaa', {'phase': phase, 'captured_frames': 3})

    status = await (await service(tmp_path, FakeRunner())).status()

    assert (status.phase, status.note, status.captured_frames) == (
        ScanPhase.INTERRUPTED,
        ScanNotes.INTERRUPTED,
        3,
    )


async def test_unreadable_status_after_a_restart_is_interrupted(tmp_path: Path) -> None:
    seed(tmp_path, 'scan-20260101-000000-aaaaaa', None)

    status = await (await service(tmp_path, FakeRunner())).status()

    assert (status.phase, status.session) == (ScanPhase.INTERRUPTED, 'scan-20260101-000000-aaaaaa')


async def test_start_creates_a_session_and_launches_the_collector(tmp_path: Path) -> None:
    runner = FakeRunner()
    sessions = await service(tmp_path, runner)

    status = await sessions.start()

    [args] = runner.started
    assert (str(args.base_url), args.seconds) == (CONTROLLER_URL + '/', DURATION_S)
    assert args.output_dir.parent == tmp_path
    assert dumped(status) == {'phase': 'starting', 'captured_frames': 0, 'session': args.output_dir.name}
    assert json.loads((args.output_dir / 'status.json').read_text())['phase'] == 'starting'


async def test_only_one_collection_at_a_time(tmp_path: Path) -> None:
    runner = FakeRunner()
    sessions = await service(tmp_path, runner)

    results = await asyncio.gather(sessions.start(), sessions.start(), return_exceptions=True)

    assert sum(isinstance(result, ScanAlreadyRunningError) for result in results) == 1
    assert len(runner.started) == 1
    assert str(next(r for r in results if isinstance(r, Exception))) == ScanNotes.ALREADY_RUNNING


async def test_missing_runtime_creates_no_session(tmp_path: Path) -> None:
    sessions = await service(tmp_path, FakeRunner(runtime=False))

    with pytest.raises(ScanRuntimeMissingError):
        await sessions.start()

    assert list(tmp_path.iterdir()) == []
    assert (await sessions.status()).phase is ScanPhase.IDLE


async def test_collector_that_dies_mid_collection_is_an_error(tmp_path: Path) -> None:
    runner = FakeRunner()
    sessions = await service(tmp_path, runner)
    session = Path(str((await sessions.start()).session))

    runner.exit()  # crashed without writing a final status

    status = await sessions.status()
    assert (status.phase, status.note) == (ScanPhase.ERROR, ScanNotes.COLLECTOR_EXITED)
    assert session.name == status.session


async def test_finished_collection_keeps_its_own_phase(tmp_path: Path) -> None:
    runner = FakeRunner()
    sessions = await service(tmp_path, runner)
    session = tmp_path / str((await sessions.start()).session)
    (session / 'status.json').write_text(json.dumps({'phase': 'complete', 'captured_frames': 60}))
    runner.exit()

    assert (await sessions.status()).phase is ScanPhase.COMPLETE


async def test_stop_terminates_only_a_running_collector(tmp_path: Path) -> None:
    runner = FakeRunner()
    sessions = await service(tmp_path, runner)

    assert (await sessions.stop()).phase is ScanPhase.IDLE
    await sessions.start()
    await sessions.stop()

    assert runner.has_exited


async def test_artifacts_come_from_the_current_session(tmp_path: Path) -> None:
    seed(tmp_path, 'scan-20260101-000000-aaaaaa', {'phase': 'complete'}, {'mosaic.jpg': b'old'})
    seed(tmp_path, 'scan-20260102-000000-bbbbbb', {'phase': 'complete'}, {'mosaic.jpg': b'current'})

    sessions = await service(tmp_path, FakeRunner())

    assert await sessions.artifact(ScanArtifact.MOSAIC) == b'current'


async def test_artifact_without_a_session_is_not_found(tmp_path: Path) -> None:
    with pytest.raises(ScanArtifactNotFoundError):
        await (await service(tmp_path, FakeRunner())).artifact(ScanArtifact.MOSAIC)


async def test_shutdown_stops_the_collector(tmp_path: Path) -> None:
    runner = FakeRunner()

    await (await service(tmp_path, runner)).shutdown()

    assert runner.shutdowns == 1


async def test_real_collector_process_reports_its_final_status(tmp_path: Path) -> None:
    # A stand-in collector that writes a final status and exits, run as a real subprocess.
    script = (
        'import json, pathlib, sys; out = pathlib.Path(sys.argv[sys.argv.index("--output") + 1]); '
        '(out / "status.json").write_text(json.dumps({"phase": "complete", "captured_frames": 2}))'
    )
    runner = ScanWorkerRunner(command=(sys.executable, '-c', script), runtime_installed=lambda: True)
    sessions = await service(tmp_path, runner)

    await sessions.start()
    async with asyncio.timeout(5):
        while not runner.has_exited:
            await asyncio.sleep(0.01)

    assert dumped(await sessions.status())['phase'] == 'complete'
