"""Board-scan vocabulary shared by the HTTP process and the collector subprocess.

The two sides run in different processes and meet only through files (``status.json``) and
command-line flags, so both shapes are defined here.
"""

from enum import StrEnum
from pathlib import Path
from typing import Final, final

from pydantic import AnyHttpUrl, BaseModel, ConfigDict, Field


class ScanPhase(StrEnum):
    IDLE = 'idle'
    STARTING = 'starting'
    COLLECTING = 'collecting'
    COMPLETE = 'complete'
    STOPPED = 'stopped'
    ERROR = 'error'
    INTERRUPTED = 'interrupted'

    @property
    def is_active(self) -> bool:
        """A collector process is expected to be alive in this phase."""
        return self in (ScanPhase.STARTING, ScanPhase.COLLECTING)


class ScanArtifact(StrEnum):
    """The only session files exposed over HTTP. Raw frames and metadata stay on the server."""

    OVERHEAD = 'overhead.jpg'
    WRIST = 'wrist.jpg'
    MOSAIC = 'mosaic.jpg'


class ScanUnit(StrEnum):
    """Collection results are in camera pixels until a physical scale is measured."""

    PIXELS = 'pixels'


@final
class ScanLimits:
    """Bounds on a collection session, shared by settings and the collector's arguments."""

    # Same range the legacy collector clamped `--seconds` into.
    DURATION_MIN_S: Final = 1
    DURATION_MAX_S: Final = 120


@final
class ScanFiles:
    """Layout of one session directory, written by the collector and read by the HTTP process.

    Sessions are named ``scan-<UTC YYYYmmdd-HHMMSS>-<6 hex>``, so name order is start order.
    """

    STATUS: Final = 'status.json'
    LOG: Final = 'collector.log'
    SESSION_PREFIX: Final = 'scan-'
    SESSION_TIMESTAMP: Final = '%Y%m%d-%H%M%S'
    SESSION_ID_HEX_CHARS: Final = 6
    # Raw camera frames live here; only the owner may read them.
    SESSION_DIR_MODE: Final = 0o700


@final
class ScanWorkerFlags:
    """Command-line flags of the collector subprocess: what the runner passes, what the worker parses."""

    BASE_URL: Final = '--base'
    OUTPUT_DIR: Final = '--output'
    SECONDS: Final = '--seconds'


class ScanStatus(BaseModel):
    """``status.json`` of a session, and the body of ``GET /api/scan/status``.

    The collector writes many more fields (counters, coverage, timings); they are kept as extras
    and relayed untouched. Serialize with ``exclude_unset=True`` so only fields that were present
    or explicitly set appear, exactly as the legacy dashboard returned them.
    """

    model_config = ConfigDict(extra='allow', frozen=True)

    phase: ScanPhase
    captured_frames: int | None = None
    session: str | None = None
    note: str | None = None
    units: ScanUnit | None = None
    metric_calibrated: bool | None = None


class ScanWorkerArgs(BaseModel):
    """Arguments of one collector run, validated on both sides of the process boundary."""

    model_config = ConfigDict(frozen=True)

    base_url: AnyHttpUrl
    output_dir: Path
    seconds: float = Field(ge=ScanLimits.DURATION_MIN_S, le=ScanLimits.DURATION_MAX_S)
