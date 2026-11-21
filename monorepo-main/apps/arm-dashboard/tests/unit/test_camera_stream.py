import logging
from collections.abc import AsyncIterator
from dataclasses import dataclass, field

import httpx
import pytest

from arm_dashboard.cameras import (
    CameraFrameUnavailableError,
    CameraSnapshotProcessor,
    CameraStreamService,
    mjpeg_part,
)
from arm_dashboard.controller import ControllerClient, ControllerPayloadTooLargeError
from arm_dashboard.core.typing import CameraName, UpstreamResponse

pytestmark = pytest.mark.anyio

FRAME = b'jpeg'
# Simulated costs, as in the legacy pacing test: fetch 30 ms, browser write 20 ms → 50 ms per tick.
FETCH_COST_S = 0.03
WRITE_COST_S = 0.02


@dataclass
class FakeClock:
    now: float = 0.0
    sleeps: list[float] = field(default_factory=list)

    def monotonic(self) -> float:
        return self.now

    async def sleep(self, seconds: float) -> None:
        self.sleeps.append(seconds)
        self.now += seconds


@dataclass
class FakeSource:
    """Stands in for ControllerClient through UpstreamReaderProtocol."""

    clock: FakeClock
    responses: list[UpstreamResponse | Exception] = field(default_factory=list)
    calls: list[tuple[str, int | None, float | None]] = field(default_factory=list)

    async def get(
        self, path: str, *, limit_bytes: int | None = None, timeout_s: float | None = None
    ) -> UpstreamResponse:
        self.calls.append((path, limit_bytes, timeout_s))
        self.clock.now += FETCH_COST_S
        outcome = self.responses.pop(0) if self.responses else ok()
        if isinstance(outcome, Exception):
            raise outcome
        return outcome


def ok(body: bytes = FRAME) -> UpstreamResponse:
    return UpstreamResponse(status_code=200, content_type='image/jpeg', body=body)


def stream_service(source: FakeSource) -> CameraStreamService:
    snapshots = CameraSnapshotProcessor(source, limit_bytes=2_000_000, timeout_s=3.0)
    return CameraStreamService(snapshots, clock=source.clock.monotonic, sleep=source.clock.sleep)


async def take(parts: AsyncIterator[bytes], count: int, clock: FakeClock) -> list[bytes]:
    received = []
    async for part in parts:
        clock.now += WRITE_COST_S
        received.append(part)
        if len(received) == count:
            break
    return received


async def test_snapshot_uses_controller_path_limit_and_timeout() -> None:
    source = FakeSource(FakeClock())

    frame = await CameraSnapshotProcessor(source, limit_bytes=123, timeout_s=3.0).fetch(CameraName.WRIST)

    assert frame == FRAME
    assert source.calls == [('/cameras/wrist.jpg', 123, 3.0)]


async def test_snapshot_without_fresh_frame_is_unavailable() -> None:
    source = FakeSource(
        FakeClock(), [UpstreamResponse(status_code=503, content_type='application/json', body=b'{}')]
    )

    with pytest.raises(CameraFrameUnavailableError, match='503'):
        await CameraSnapshotProcessor(source, limit_bytes=10, timeout_s=3.0).fetch(CameraName.OVERHEAD)


@pytest.mark.parametrize(('fps', 'expected_sleep'), [(2, 0.45), (10, 0.05), (15, 1 / 15 - 0.05)])
async def test_cadence_counts_fetch_and_write_time(fps: int, expected_sleep: float) -> None:
    source = FakeSource(FakeClock())

    parts = await take(stream_service(source).frames(CameraName.OVERHEAD, fps), 3, source.clock)

    assert parts == [mjpeg_part(FRAME)] * 3
    assert source.clock.sleeps == pytest.approx([expected_sleep] * 2)


async def test_slow_consumer_never_gets_a_negative_wait() -> None:
    source = FakeSource(FakeClock())
    parts = stream_service(source).frames(CameraName.OVERHEAD, 15)

    await anext(parts)
    source.clock.now += 5.0  # the browser took five seconds to accept the part
    await anext(parts)

    assert source.clock.sleeps == [0.0]


async def test_next_snapshot_is_fetched_only_when_the_consumer_asks() -> None:
    source = FakeSource(FakeClock())
    parts = stream_service(source).frames(CameraName.OVERHEAD, 2)

    await anext(parts)

    assert len(source.calls) == 1
    await anext(parts)
    assert len(source.calls) == 2


async def test_every_part_is_a_fresh_snapshot() -> None:
    source = FakeSource(FakeClock(), [ok(b'one'), ok(b'two')])

    parts = await take(stream_service(source).frames(CameraName.WRIST, 10), 2, source.clock)

    assert parts == [mjpeg_part(b'one'), mjpeg_part(b'two')]


@pytest.mark.parametrize(
    'failure',
    [
        UpstreamResponse(status_code=503, content_type='application/json', body=b'{}'),
        ControllerPayloadTooLargeError('frame over limit'),
    ],
    ids=['no-fresh-frame', 'oversized-frame'],
)
async def test_first_frame_failure_raises_so_the_caller_can_answer_502(
    failure: UpstreamResponse | Exception,
) -> None:
    source = FakeSource(FakeClock(), [failure])

    with pytest.raises((CameraFrameUnavailableError, ControllerPayloadTooLargeError)):
        await anext(stream_service(source).frames(CameraName.OVERHEAD, 2))


async def test_failure_after_start_ends_stream_and_is_logged(caplog: pytest.LogCaptureFixture) -> None:
    unavailable = UpstreamResponse(status_code=503, content_type='application/json', body=b'{}')
    source = FakeSource(FakeClock(), [ok(), unavailable, ok()])

    with caplog.at_level(logging.INFO):
        parts = [part async for part in stream_service(source).frames(CameraName.OBSERVER, 10)]

    assert parts == [mjpeg_part(FRAME)]
    assert len(source.calls) == 2
    assert 'Camera observer stream ended' in caplog.text


async def test_closing_the_stream_stops_fetching() -> None:
    source = FakeSource(FakeClock())
    parts = stream_service(source).frames(CameraName.OVERHEAD, 2)

    await anext(parts)
    await parts.aclose()

    assert len(source.calls) == 1


async def test_snapshot_reads_through_the_real_controller_client() -> None:
    def handler(request: httpx.Request) -> httpx.Response:
        return httpx.Response(200, headers={'Content-Type': 'image/jpeg'}, content=b'x' * 11)

    http = httpx.AsyncClient(transport=httpx.MockTransport(handler), base_url='http://controller.test')
    snapshots = CameraSnapshotProcessor(ControllerClient(http), limit_bytes=10, timeout_s=3.0)

    with pytest.raises(ControllerPayloadTooLargeError):
        await snapshots.fetch(CameraName.OVERHEAD)
