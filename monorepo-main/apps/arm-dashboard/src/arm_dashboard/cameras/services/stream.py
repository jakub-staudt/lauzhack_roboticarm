import asyncio
import logging
import time
from collections.abc import AsyncGenerator, Awaitable, Callable

from arm_dashboard.cameras.snapshot import CameraSnapshotProcessor
from arm_dashboard.cameras.utils import mjpeg_part
from arm_dashboard.core.exceptions import UpstreamError
from arm_dashboard.core.typing.camera import CameraName

logger = logging.getLogger(__name__)

Clock = Callable[[], float]
Sleep = Callable[[float], Awaitable[None]]


class CameraStreamService:
    """A paced MJPEG stream built from fresh snapshots, never from a relayed upstream stream.

    Pull-based: the next snapshot is fetched only after the server accepted the previous part,
    so the application never queues frames, and the default 2 fps keeps two streams from
    starving control traffic through the SSH relay. Socket buffers can still hold several parts
    for a stalled reader; that was equally true of the legacy proxy (measured with
    tests/manual/backpressure.py). The wait between parts is measured from the start of each
    fetch, so the cadence holds whatever the fetch and write took.
    """

    def __init__(
        self,
        snapshots: CameraSnapshotProcessor,
        *,
        clock: Clock = time.monotonic,
        sleep: Sleep = asyncio.sleep,
    ) -> None:
        self._snapshots = snapshots
        self._clock = clock
        self._sleep = sleep

    async def frames(self, camera: CameraName, fps: int) -> AsyncGenerator[bytes]:
        """MJPEG parts until the consumer stops or the camera fails.

        A failure on the first frame raises, so the caller can still answer 502; once parts have
        been sent, a failure is logged and ends the stream.
        """
        period = 1.0 / fps
        started = False
        while True:
            tick = self._clock()
            try:
                frame = await self._snapshots.fetch(camera)
            except UpstreamError as exc:
                if not started:
                    raise
                logger.info('Camera %s stream ended: %s', camera, exc)
                return
            started = True
            yield mjpeg_part(frame)
            await self._sleep(max(0.0, period - (self._clock() - tick)))
