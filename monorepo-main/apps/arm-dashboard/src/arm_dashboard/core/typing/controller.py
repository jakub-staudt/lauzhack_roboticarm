"""The controller (soarm-panel) API as the dashboard sees it: routes, commands, raw responses."""

from enum import StrEnum
from typing import Any, Final, Protocol, final

from pydantic import BaseModel, ConfigDict

from arm_dashboard.core.typing.camera import CameraName

ControllerState = dict[str, Any]
"""``GET /api/state`` body. Deliberately not modelled: soarm owns this contract and extends it
(capabilities, model_control_version, ...). The dashboard types only what it adds itself."""


class ControllerCommand(StrEnum):
    """POST routes the dashboard forwards. Anything else never reaches the controller.

    The value is the controller path. Bodies are opaque JSON validated by the controller.
    """

    CONTROL_CLAIM = '/api/control/claim'
    CONTROL_HEARTBEAT = '/api/control/heartbeat'
    CONTROL_RELEASE = '/api/control/release'
    STOP = '/api/stop'
    EMERGENCY_STOP = '/api/emergency-stop'
    EMERGENCY_STOP_RESET = '/api/emergency-stop/reset'
    JOG = '/api/jog'
    TARGET = '/api/target'
    HOME = '/api/home'
    SPEED = '/api/speed'
    TORQUE = '/api/torque'
    CAMERAS = '/api/cameras'
    VIEW = '/api/view'
    CARTESIAN_START = '/api/cartesian/start'
    CARTESIAN_RENEW = '/api/cartesian/renew'
    CARTESIAN_STOP = '/api/cartesian/stop'


@final
class ControllerRoutes:
    """Controller GET paths used by the proxy, the camera stream and the scan worker."""

    STATE: Final = '/api/state'
    SNAPSHOT_TEMPLATE: Final = '/cameras/{camera}.jpg'

    @staticmethod
    def snapshot(camera: CameraName) -> str:
        return ControllerRoutes.SNAPSHOT_TEMPLATE.format(camera=camera.value)


class UpstreamResponse(BaseModel):
    """A fully read upstream answer, relayed as-is: status, content type and body bytes."""

    model_config = ConfigDict(frozen=True)

    status_code: int
    content_type: str
    body: bytes


class UpstreamReaderProtocol(Protocol):
    """Reads one controller path whole, with a byte cap and a per-request timeout.

    ``ControllerClient`` satisfies it structurally; consumers outside the controller package
    (the camera stream) depend on this instead of importing that package.
    """

    async def get(
        self, path: str, *, limit_bytes: int | None = None, timeout_s: float | None = None
    ) -> UpstreamResponse: ...
