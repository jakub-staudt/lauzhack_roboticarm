"""Session directories on disk. A processor: reads and writes files, decides nothing.

All file access runs in a worker thread so the event loop never blocks on the disk.
"""

import asyncio
import logging
import time
import uuid
from pathlib import Path

from pydantic import ValidationError

from arm_dashboard.core.typing.scan import ScanArtifact, ScanFiles, ScanStatus
from arm_dashboard.scan.exceptions import ScanArtifactNotFoundError

logger = logging.getLogger(__name__)


class ScanStorage:
    def __init__(self, root: Path) -> None:
        self._root = root

    async def ensure_root(self) -> None:
        await asyncio.to_thread(self._root.mkdir, parents=True, exist_ok=True)

    async def latest_session(self) -> Path | None:
        return await asyncio.to_thread(self._latest_session)

    async def create_session(self) -> Path:
        return await asyncio.to_thread(self._create_session)

    async def read_status(self, session: Path) -> ScanStatus | None:
        """The session's status, or ``None`` when the file is missing, unreadable or not a status."""
        return await asyncio.to_thread(self._read_status, session)

    async def write_status(self, session: Path, status: ScanStatus) -> None:
        await asyncio.to_thread(self._write_status, session, status)

    async def read_artifact(self, session: Path, artifact: ScanArtifact) -> bytes:
        try:
            return await asyncio.to_thread((session / artifact.value).read_bytes)
        except FileNotFoundError as exc:
            raise ScanArtifactNotFoundError(f'{session.name} has no {artifact.value} yet') from exc

    @staticmethod
    def log_path(session: Path) -> Path:
        return session / ScanFiles.LOG

    def _latest_session(self) -> Path | None:
        sessions = sorted(self._root.glob(ScanFiles.SESSION_PREFIX + '*'))
        return sessions[-1] if sessions else None

    def _create_session(self) -> Path:
        stamp = time.strftime(ScanFiles.SESSION_TIMESTAMP, time.gmtime())
        suffix = uuid.uuid4().hex[: ScanFiles.SESSION_ID_HEX_CHARS]
        session = self._root / f'{ScanFiles.SESSION_PREFIX}{stamp}-{suffix}'
        session.mkdir(mode=ScanFiles.SESSION_DIR_MODE)
        return session

    @staticmethod
    def _read_status(session: Path) -> ScanStatus | None:
        try:
            return ScanStatus.model_validate_json((session / ScanFiles.STATUS).read_bytes())
        except FileNotFoundError:
            return None
        except (OSError, ValidationError) as exc:
            logger.warning('Unreadable %s in %s: %s', ScanFiles.STATUS, session.name, exc)
            return None

    @staticmethod
    def _write_status(session: Path, status: ScanStatus) -> None:
        # Replace atomically: the collector and the browser poll this file concurrently.
        target = session / ScanFiles.STATUS
        staged = target.with_suffix('.next')
        staged.write_text(status.model_dump_json(exclude_unset=True))
        staged.replace(target)
