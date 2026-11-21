import json
import logging
import re
import stat
from pathlib import Path

import pytest

from arm_dashboard.core.typing import ScanArtifact, ScanPhase, ScanStatus
from arm_dashboard.scan import ScanArtifactNotFoundError, ScanStorage

pytestmark = pytest.mark.anyio

SESSION_NAME = re.compile(r'scan-\d{8}-\d{6}-[0-9a-f]{6}')


def seed(root: Path, name: str, status: object | None = None) -> Path:
    session = root / name
    session.mkdir(parents=True)
    if status is not None:
        (session / 'status.json').write_text(status if isinstance(status, str) else json.dumps(status))
    return session


async def test_ensure_root_creates_missing_parents(tmp_path: Path) -> None:
    root = tmp_path / 'a' / 'b' / 'scans'

    await ScanStorage(root).ensure_root()
    await ScanStorage(root).ensure_root()

    assert root.is_dir()


async def test_latest_session_is_the_newest_by_name(tmp_path: Path) -> None:
    storage = ScanStorage(tmp_path)
    assert await storage.latest_session() is None

    seed(tmp_path, 'scan-20260101-000000-aaaaaa')
    newest = seed(tmp_path, 'scan-20260102-000000-bbbbbb')
    seed(tmp_path, 'unrelated-99999999')

    assert await storage.latest_session() == newest


async def test_new_session_is_private_and_named_by_utc_start(tmp_path: Path) -> None:
    session = await ScanStorage(tmp_path).create_session()

    assert SESSION_NAME.fullmatch(session.name)
    assert stat.S_IMODE(session.stat().st_mode) == 0o700


async def test_status_round_trips_exactly_the_written_fields(tmp_path: Path) -> None:
    storage = ScanStorage(tmp_path)
    session = seed(tmp_path, 'scan-20260101-000000-aaaaaa')

    await storage.write_status(session, ScanStatus(phase=ScanPhase.STARTING, captured_frames=0))

    assert json.loads((session / 'status.json').read_text()) == {'phase': 'starting', 'captured_frames': 0}
    assert not (session / 'status.next').exists()
    assert await storage.read_status(session) == ScanStatus(phase=ScanPhase.STARTING, captured_frames=0)


async def test_collector_fields_survive_reading(tmp_path: Path) -> None:
    written = {'phase': 'complete', 'captured_frames': 4, 'coverage': {'observed_output_pixels': 9}}
    session = seed(tmp_path, 'scan-20260101-000000-aaaaaa', written)

    status = await ScanStorage(tmp_path).read_status(session)

    assert status is not None
    assert status.model_dump(mode='json', exclude_unset=True) == written


@pytest.mark.parametrize(
    'content',
    [None, 'not json', '{"phase": "paused"}', '[]'],
    ids=['missing', 'garbage', 'unknown-phase', 'list'],
)
async def test_unusable_status_reads_as_none(tmp_path: Path, content: str | None) -> None:
    session = seed(tmp_path, 'scan-20260101-000000-aaaaaa', content)

    assert await ScanStorage(tmp_path).read_status(session) is None


async def test_corrupt_status_is_logged(tmp_path: Path, caplog: pytest.LogCaptureFixture) -> None:
    session = seed(tmp_path, 'scan-20260101-000000-aaaaaa', 'not json')

    with caplog.at_level(logging.WARNING):
        await ScanStorage(tmp_path).read_status(session)

    assert 'Unreadable status.json in scan-20260101-000000-aaaaaa' in caplog.text


@pytest.mark.parametrize('artifact', list(ScanArtifact))
async def test_artifacts_are_read_by_name(tmp_path: Path, artifact: ScanArtifact) -> None:
    session = seed(tmp_path, 'scan-20260101-000000-aaaaaa')
    (session / artifact.value).write_bytes(b'jpeg')

    assert await ScanStorage(tmp_path).read_artifact(session, artifact) == b'jpeg'


async def test_missing_artifact_is_not_found(tmp_path: Path) -> None:
    session = seed(tmp_path, 'scan-20260101-000000-aaaaaa')

    with pytest.raises(ScanArtifactNotFoundError):
        await ScanStorage(tmp_path).read_artifact(session, ScanArtifact.MOSAIC)
