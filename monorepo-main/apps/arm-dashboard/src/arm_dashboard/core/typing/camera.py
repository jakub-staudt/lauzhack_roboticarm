"""Camera vocabulary shared by the camera stream, the controller proxy and the scan worker."""

from enum import StrEnum
from typing import Final, final

from pydantic import BaseModel, ConfigDict, computed_field


class CameraName(StrEnum):
    """Cameras the controller exposes. The observer is outside the VLA input contract."""

    OVERHEAD = 'overhead'
    WRIST = 'wrist'
    OBSERVER = 'observer'


@final
class CameraStreamQuery:
    """The MJPEG stream's only query parameter, as browsers send it and ``camera_delivery`` advertises it."""

    FPS: Final = 'fps'


@final
class CameraDeliveryNotes:
    """Operator-facing texts in ``/api/state`` → ``camera_delivery``; panel.js shows them verbatim."""

    BUFFERING: Final = 'Each fresh snapshot is fetched after the previous part is written.'
    LATENCY: Final = 'Output cadence is not a sensor-to-screen latency measurement.'


class CameraDelivery(BaseModel):
    """How the dashboard delivers camera frames; injected into the controller state for panel.js.

    Built by the camera package, embedded into state by the controller package: the shape lives
    here so neither imports the other.
    """

    model_config = ConfigDict(frozen=True)

    default_fps: int
    direct_default_fps: int
    direct_hosts: tuple[str, ...]
    available_fps: tuple[int, ...]
    buffering: str = CameraDeliveryNotes.BUFFERING
    latency_note: str = CameraDeliveryNotes.LATENCY

    @computed_field
    @property
    def max_fps(self) -> int:
        return max(self.available_fps)

    @computed_field
    @property
    def selection_query(self) -> str:
        options = '|'.join(str(fps) for fps in self.available_fps)
        return f'?{CameraStreamQuery.FPS}=<{options}>'
