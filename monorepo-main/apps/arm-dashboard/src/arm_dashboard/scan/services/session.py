import asyncio
from pathlib import Path
from typing import Final, final

from pydantic import AnyHttpUrl

from arm_dashboard.core.typing.scan import ScanArtifact, ScanPhase, ScanStatus, ScanUnit, ScanWorkerArgs
from arm_dashboard.scan.exceptions import ScanAlreadyRunningError, ScanArtifactNotFoundError
from arm_dashboard.scan.runner import ScanWorkerRunner
from arm_dashboard.scan.storage import ScanStorage


@final
class ScanNotes:
    """Operator-facing notes; scan.html shows them as they are."""

    ALREADY_RUNNING: Final = 'A collection session is already running.'
    COLLECTOR_EXITED: Final = 'Collector exited before completing; inspect collector.log.'
    INTERRUPTED: Final = 'A previous collector was interrupted by service restart.'


class ScanSessionService:
    """One collection session at a time, and the truth about the current one.

    The status file says what the collector last wrote; this service corrects it with what the
    dashboard knows: a collector that died, or one lost to a dashboard restart, cannot still be
    collecting. State lives in this process, so the dashboard must run as a single process.
    """

    def __init__(
        self, storage: ScanStorage, runner: ScanWorkerRunner, *, controller_url: AnyHttpUrl, duration_s: float
    ) -> None:
        self._storage = storage
        self._runner = runner
        self._controller_url = controller_url
        self._duration_s = duration_s
        self._lock = asyncio.Lock()
        self._session: Path | None = None

    async def restore(self) -> None:
        """At startup, adopt the newest session on disk as the current one."""
        async with self._lock:
            await self._storage.ensure_root()
            self._session = await self._storage.latest_session()

    async def status(self) -> ScanStatus:
        async with self._lock:
            return await self._status()

    async def start(self) -> ScanStatus:
        async with self._lock:
            if self._runner.is_running:
                raise ScanAlreadyRunningError(ScanNotes.ALREADY_RUNNING)
            self._runner.ensure_runtime()
            session = await self._storage.create_session()
            self._session = session
            await self._storage.write_status(session, ScanStatus(phase=ScanPhase.STARTING, captured_frames=0))
            args = ScanWorkerArgs(base_url=self._controller_url, output_dir=session, seconds=self._duration_s)
            await self._runner.start(args, self._storage.log_path(session))
            return await self._status()

    async def stop(self) -> ScanStatus:
        async with self._lock:
            self._runner.terminate()
            return await self._status()

    async def artifact(self, artifact: ScanArtifact) -> bytes:
        async with self._lock:
            if self._session is None:
                raise ScanArtifactNotFoundError('No collection session yet')
            return await self._storage.read_artifact(self._session, artifact)

    async def shutdown(self) -> None:
        await self._runner.shutdown()

    async def _status(self) -> ScanStatus:
        if self._session is None:
            return ScanStatus(
                phase=ScanPhase.IDLE, captured_frames=0, units=ScanUnit.PIXELS, metric_calibrated=False
            )
        written = await self._storage.read_status(self._session)
        status = written or ScanStatus(phase=ScanPhase.STARTING, captured_frames=0)
        update: dict[str, object] = {'session': self._session.name}
        if status.phase.is_active and self._runner.has_exited:
            update |= {'phase': ScanPhase.ERROR, 'note': ScanNotes.COLLECTOR_EXITED}
        elif status.phase.is_active and not self._runner.has_started:
            update |= {'phase': ScanPhase.INTERRUPTED, 'note': ScanNotes.INTERRUPTED}
        return status.model_copy(update=update)
