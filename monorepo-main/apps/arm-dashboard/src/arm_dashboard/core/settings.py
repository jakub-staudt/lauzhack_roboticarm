"""The dashboard's settings tree: the only module that reads the environment.

Every key is ``RAMION_DASHBOARD_<SECTION>__<FIELD>``, e.g. ``RAMION_DASHBOARD_SERVER__PORT=8780``.
Sections are nested models, one per concern; add a concern by adding a model and one field on
``Settings``, never flat keys on the root.

Upstream defaults are the current production addresses on ramion-server-01, so an empty env
reproduces the legacy dashboard except for the bind address, which defaults to loopback.
"""

from enum import StrEnum
from functools import lru_cache
from importlib import resources
from pathlib import Path
from typing import Annotated, Final, Self, final

from pydantic import AnyHttpUrl, BaseModel, ConfigDict, Field, field_validator, model_validator
from pydantic_settings import BaseSettings, NoDecode, SettingsConfigDict

from arm_dashboard.core.typing.scan import ScanLimits


@final
class SettingsSource:
    """How settings are read from the environment; documented to operators in .env.example."""

    ENV_PREFIX: Final = 'RAMION_DASHBOARD_'
    ENV_NESTED_DELIMITER: Final = '__'
    ENV_FILE: Final = '.env'
    # List values are comma-separated; pydantic-settings' JSON decoding is disabled for them.
    LIST_SEPARATOR: Final = ','


def _packaged_pages() -> Path:
    """``arm_dashboard/static`` wherever the package is installed (editable or wheel)."""
    return Path(str(resources.files('arm_dashboard') / 'static'))


@final
class ByteSize:
    KIB: Final = 1024
    MIB: Final = 1024 * 1024


class LogLevel(StrEnum):
    DEBUG = 'DEBUG'
    INFO = 'INFO'
    WARNING = 'WARNING'
    ERROR = 'ERROR'


class _Section(BaseModel):
    model_config = ConfigDict(frozen=True)


class ServerSettings(_Section):
    """Where the HTTP server listens. Production binds the Tailscale IP through the env file."""

    host: str = '127.0.0.1'
    port: int = Field(default=8780, ge=1, le=65535)
    log_level: LogLevel = LogLevel.INFO
    docs_enabled: bool = Field(
        default=False,
        description='Expose /docs and /openapi.json. Off by default: the dashboard has no auth.',
    )
    static_dir: Path = Field(
        default_factory=_packaged_pages,
        description='Directory with operator panel and map/scan assets (default: packaged copy).',
    )


class ControllerSettings(_Section):
    """soarm-panel, which owns the arm and the cameras."""

    base_url: AnyHttpUrl = AnyHttpUrl('http://100.85.56.14:8765')
    timeout_s: float = Field(default=8.0, gt=0)
    command_body_limit_bytes: int = Field(default=8 * ByteSize.KIB, gt=0)


class ViewerSettings(_Section):
    """The read-only Viser pose viewer, relayed under /viewer/."""

    base_url: AnyHttpUrl = AnyHttpUrl('http://100.85.56.14:8766')
    io_timeout_s: float = Field(default=8.0, gt=0)
    idle_timeout_s: float = Field(default=90.0, gt=0)
    page_limit_bytes: int = Field(default=4 * ByteSize.MIB, gt=0)


class CameraSettings(_Section):
    """MJPEG delivery. A stream fetches one fresh snapshot per part, paced to the chosen fps."""

    available_fps: Annotated[tuple[int, ...], NoDecode] = Field(
        default=(2, 10, 15),
        description='Comma-separated in the environment, e.g. "2,10,15". JSON lists are not accepted.',
    )
    default_fps: int = Field(
        default=2, description='Used through the SSH relay; keeps control traffic responsive.'
    )
    direct_default_fps: int = Field(
        default=10, description='Advertised for direct Tailscale/controller access.'
    )
    frame_timeout_s: float = Field(default=3.0, gt=0)
    frame_limit_bytes: int = Field(default=2_000_000, gt=0)

    @field_validator('available_fps', mode='before')
    @classmethod
    def _split_list(cls, value: object) -> object:
        if isinstance(value, str):
            return tuple(part.strip() for part in value.split(SettingsSource.LIST_SEPARATOR) if part.strip())
        return value

    @field_validator('available_fps')
    @classmethod
    def _positive_and_unique(cls, value: tuple[int, ...]) -> tuple[int, ...]:
        if not value or any(fps <= 0 for fps in value):
            raise ValueError('available_fps must be a non-empty list of positive integers')
        if len(set(value)) != len(value):
            raise ValueError('available_fps must not repeat a value')
        return value

    @model_validator(mode='after')
    def _defaults_are_available(self) -> Self:
        for name in ('default_fps', 'direct_default_fps'):
            if getattr(self, name) not in self.available_fps:
                raise ValueError(f'{name} must be one of available_fps {self.available_fps}')
        return self


class ScanSettings(_Section):
    """Automatic board collection: where sessions are stored and how long one runs."""

    # validate_default: the default itself contains `~` and must go through _expand_home.
    storage_dir: Path = Field(default=Path('~/ramion-scans'), validate_default=True)
    duration_s: float = Field(default=60.0, ge=ScanLimits.DURATION_MIN_S, le=ScanLimits.DURATION_MAX_S)

    @field_validator('storage_dir')
    @classmethod
    def _expand_home(cls, value: Path) -> Path:
        return value.expanduser()


class Settings(BaseSettings):
    model_config = SettingsConfigDict(
        env_prefix=SettingsSource.ENV_PREFIX,
        env_nested_delimiter=SettingsSource.ENV_NESTED_DELIMITER,
        env_file=SettingsSource.ENV_FILE,
        extra='ignore',
        frozen=True,
    )

    server: ServerSettings = Field(default_factory=ServerSettings)
    controller: ControllerSettings = Field(default_factory=ControllerSettings)
    viewer: ViewerSettings = Field(default_factory=ViewerSettings)
    camera: CameraSettings = Field(default_factory=CameraSettings)
    scan: ScanSettings = Field(default_factory=ScanSettings)


@lru_cache(maxsize=1)
def get_settings() -> Settings:
    """Read the environment once per process. Tests pass ``Settings(...)`` explicitly instead."""
    return Settings()
