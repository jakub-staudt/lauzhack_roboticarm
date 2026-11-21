from http import HTTPStatus

from arm_dashboard.cameras.exceptions import CameraFrameUnavailableError
from arm_dashboard.core.typing.camera import CameraName
from arm_dashboard.core.typing.controller import ControllerRoutes, UpstreamReaderProtocol


class CameraSnapshotProcessor:
    """Fetches one fresh JPEG from the controller's snapshot cache, bounded in size and time."""

    def __init__(self, source: UpstreamReaderProtocol, *, limit_bytes: int, timeout_s: float) -> None:
        self._source = source
        self._limit_bytes = limit_bytes
        self._timeout_s = timeout_s

    async def fetch(self, camera: CameraName) -> bytes:
        response = await self._source.get(
            ControllerRoutes.snapshot(camera), limit_bytes=self._limit_bytes, timeout_s=self._timeout_s
        )
        if response.status_code != HTTPStatus.OK:
            raise CameraFrameUnavailableError(f'{camera} snapshot answered {response.status_code}')
        return response.body
