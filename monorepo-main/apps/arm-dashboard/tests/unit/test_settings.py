import os
from pathlib import Path

import pytest
from pydantic import ValidationError

from arm_dashboard.core.settings import (
    CameraSettings,
    ControllerSettings,
    LogLevel,
    ScanSettings,
    ServerSettings,
    Settings,
    ViewerSettings,
    get_settings,
)


@pytest.fixture(autouse=True)
def isolated_environment(monkeypatch: pytest.MonkeyPatch, tmp_path: Path) -> None:
    # No developer .env and no inherited RAMION_DASHBOARD_* may leak into these tests.
    monkeypatch.chdir(tmp_path)
    for name in [name for name in os.environ if name.startswith('RAMION_DASHBOARD_')]:
        monkeypatch.delenv(name)
    get_settings.cache_clear()


def test_defaults_bind_loopback_with_docs_off() -> None:
    assert Settings().server == ServerSettings(
        host='127.0.0.1', port=8780, log_level=LogLevel.INFO, docs_enabled=False
    )


def test_upstream_defaults_match_the_legacy_production_addresses() -> None:
    settings = Settings()

    assert str(settings.controller.base_url) == 'http://100.85.56.14:8765/'
    assert str(settings.viewer.base_url) == 'http://100.85.56.14:8766/'


def test_limits_and_timeouts_match_legacy_literals() -> None:
    settings = Settings()

    assert (settings.controller.timeout_s, settings.controller.command_body_limit_bytes) == (8.0, 8192)
    assert (settings.viewer.io_timeout_s, settings.viewer.idle_timeout_s) == (8.0, 90.0)
    assert settings.viewer.page_limit_bytes == 4 * 1024 * 1024
    assert settings.camera == CameraSettings(
        available_fps=(2, 10, 15),
        default_fps=2,
        direct_default_fps=10,
        frame_timeout_s=3.0,
        frame_limit_bytes=2_000_000,
    )
    assert settings.scan.duration_s == 60.0


def test_scan_storage_expands_home() -> None:
    assert Settings().scan.storage_dir == Path.home() / 'ramion-scans'


def test_nested_environment_overrides(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setenv('RAMION_DASHBOARD_SERVER__HOST', '100.85.56.14')
    monkeypatch.setenv('RAMION_DASHBOARD_SERVER__PORT', '9000')
    monkeypatch.setenv('RAMION_DASHBOARD_SERVER__LOG_LEVEL', 'DEBUG')
    monkeypatch.setenv('RAMION_DASHBOARD_CONTROLLER__BASE_URL', 'http://controller:8765')
    monkeypatch.setenv('RAMION_DASHBOARD_VIEWER__BASE_URL', 'http://controller:8766')
    monkeypatch.setenv('RAMION_DASHBOARD_SCAN__STORAGE_DIR', '/var/lib/ramion/scans')

    settings = Settings()

    assert (settings.server.host, settings.server.port) == ('100.85.56.14', 9000)
    assert settings.server.log_level is LogLevel.DEBUG
    assert settings.controller.base_url.host == 'controller'
    assert settings.viewer.base_url.port == 8766
    assert settings.scan.storage_dir == Path('/var/lib/ramion/scans')


def test_available_fps_is_comma_separated_in_environment(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setenv('RAMION_DASHBOARD_CAMERA__AVAILABLE_FPS', '2, 5,10')
    monkeypatch.setenv('RAMION_DASHBOARD_CAMERA__DIRECT_DEFAULT_FPS', '5')

    camera = Settings().camera

    assert camera.available_fps == (2, 5, 10)
    assert camera.direct_default_fps == 5


def test_env_file_is_read_from_working_directory(tmp_path: Path) -> None:
    (tmp_path / '.env').write_text('RAMION_DASHBOARD_SERVER__PORT=8800\n')

    assert Settings().server.port == 8800


@pytest.mark.parametrize(
    ('name', 'value'),
    [
        ('SERVER__PORT', '0'),
        ('SERVER__PORT', '65536'),
        ('SERVER__PORT', 'eighty'),
        ('SERVER__LOG_LEVEL', 'VERBOSE'),
        ('CONTROLLER__BASE_URL', 'controller:8765'),
        ('CONTROLLER__TIMEOUT_S', '0'),
        ('CAMERA__AVAILABLE_FPS', '[2,10,15]'),
        ('CAMERA__AVAILABLE_FPS', '2,2,10'),
        ('CAMERA__AVAILABLE_FPS', '0,2,10'),
        ('CAMERA__AVAILABLE_FPS', ''),
        ('CAMERA__DEFAULT_FPS', '3'),
        ('SCAN__DURATION_S', '121'),
    ],
)
def test_invalid_environment_is_rejected(monkeypatch: pytest.MonkeyPatch, name: str, value: str) -> None:
    monkeypatch.setenv(f'RAMION_DASHBOARD_{name}', value)

    with pytest.raises(ValidationError):
        Settings()


@pytest.mark.parametrize(
    'section',
    [ServerSettings(), ControllerSettings(), ViewerSettings(), CameraSettings(), ScanSettings()],
    ids=type,
)
def test_sections_are_immutable(section: object) -> None:
    field = next(iter(type(section).model_fields))  # type: ignore[attr-defined]

    with pytest.raises(ValidationError):
        setattr(section, field, getattr(section, field))


def test_get_settings_reads_environment_once(monkeypatch: pytest.MonkeyPatch) -> None:
    first = get_settings()
    monkeypatch.setenv('RAMION_DASHBOARD_SERVER__PORT', '9000')

    assert get_settings() is first


def test_env_example_documents_exactly_the_defaults(tmp_path: Path) -> None:
    # .env.example is the operator-facing list of settings; it must parse and must not drift.
    example = Path(__file__).resolve().parents[2] / '.env.example'
    (tmp_path / '.env').write_text(example.read_text())

    assert Settings() == Settings(_env_file=None)


def test_per_request_http_client_logs_are_silenced() -> None:
    import logging

    from arm_dashboard.core.logging import setup_logging

    root = logging.getLogger()
    handlers, level = root.handlers[:], root.level
    try:
        setup_logging(LogLevel.INFO)
        assert not logging.getLogger('httpx').isEnabledFor(logging.INFO)
        assert logging.getLogger('httpx').isEnabledFor(logging.WARNING)
        assert logging.getLogger('arm_dashboard.controller.client').isEnabledFor(logging.INFO)
    finally:
        root.handlers[:] = handlers
        root.setLevel(level)


def test_production_env_template_is_valid() -> None:
    # deploy/ramion-dashboard.env is copied to the server as is; a typo there would fail the deploy.
    template = Path(__file__).resolve().parents[2] / 'deploy' / 'ramion-dashboard.env'

    settings = Settings(_env_file=template)

    assert (settings.server.host, settings.server.port) == ('100.85.56.14', 8780)
    assert str(settings.controller.base_url) == 'http://100.85.56.14:8765/'
    assert str(settings.viewer.base_url) == 'http://100.85.56.14:8766/'
    assert settings.scan.storage_dir == Path('/home/kwikiel/ramion-scans')
