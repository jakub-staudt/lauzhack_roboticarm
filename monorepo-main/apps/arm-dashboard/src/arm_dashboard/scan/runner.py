"""The collector subprocess. A processor: starts, watches and stops one process; no session rules.

The collector runs with the same interpreter and environment as the dashboard (``scan`` extra),
as a separate process so OpenCV's memory and CPU never touch the web server, and so it can be
stopped without its cooperation.
"""

import asyncio
import importlib.util
import logging
import sys
from collections.abc import Callable, Sequence
from pathlib import Path
from typing import Final, final

from arm_dashboard.core.typing.scan import ScanWorkerArgs, ScanWorkerFlags
from arm_dashboard.scan.exceptions import ScanRuntimeMissingError

logger = logging.getLogger(__name__)


@final
class ScanWorkerEntry:
    COMMAND: Final = (sys.executable, '-m', 'arm_dashboard.scan.worker')
    # Imported only by the collector; its absence means the `scan` extra is not installed.
    RUNTIME_MODULE: Final = 'cv2'
    INSTALL_HINT: Final = "Scanner runtime is not installed: pip install -e 'apps/arm-dashboard[scan]'"
    SHUTDOWN_TIMEOUT_S: Final = 5.0


def scanner_runtime_installed() -> bool:
    return importlib.util.find_spec(ScanWorkerEntry.RUNTIME_MODULE) is not None


def worker_argv(args: ScanWorkerArgs) -> list[str]:
    return [
        ScanWorkerFlags.BASE_URL,
        str(args.base_url).rstrip('/'),
        ScanWorkerFlags.OUTPUT_DIR,
        str(args.output_dir),
        ScanWorkerFlags.SECONDS,
        str(args.seconds),
    ]


class ScanWorkerRunner:
    def __init__(
        self,
        *,
        command: Sequence[str] = ScanWorkerEntry.COMMAND,
        runtime_installed: Callable[[], bool] = scanner_runtime_installed,
    ) -> None:
        self._command = tuple(command)
        self._runtime_installed = runtime_installed
        self._process: asyncio.subprocess.Process | None = None

    @property
    def has_started(self) -> bool:
        """A collector was started by this process (false after a dashboard restart)."""
        return self._process is not None

    @property
    def is_running(self) -> bool:
        return self._process is not None and self._process.returncode is None

    @property
    def has_exited(self) -> bool:
        return self._process is not None and self._process.returncode is not None

    def ensure_runtime(self) -> None:
        if not self._runtime_installed():
            raise ScanRuntimeMissingError(ScanWorkerEntry.INSTALL_HINT)

    async def start(self, args: ScanWorkerArgs, log_path: Path) -> None:
        """Start the collector with stdout and stderr going to ``log_path``."""
        log = await asyncio.to_thread(log_path.open, 'wb')
        try:
            self._process = await asyncio.create_subprocess_exec(
                *self._command, *worker_argv(args), stdout=log, stderr=log
            )
        finally:
            log.close()  # the child holds its own descriptor
        logger.info('Collector started (pid %s) for %s', self._process.pid, args.output_dir.name)

    def terminate(self) -> None:
        """Ask the collector to stop; it finishes the current cycle and writes a final status."""
        if self._process is not None and self.is_running:
            self._process.terminate()

    async def shutdown(self, timeout_s: float = ScanWorkerEntry.SHUTDOWN_TIMEOUT_S) -> None:
        """Stop the collector and reap it, killing it if it does not exit in time. For app shutdown."""
        if self._process is None or not self.is_running:
            return
        self._process.terminate()
        try:
            await asyncio.wait_for(self._process.wait(), timeout_s)
        except TimeoutError:
            logger.warning(
                'Collector pid %s ignored SIGTERM for %.0f s; killing', self._process.pid, timeout_s
            )
            self._process.kill()
            await self._process.wait()
