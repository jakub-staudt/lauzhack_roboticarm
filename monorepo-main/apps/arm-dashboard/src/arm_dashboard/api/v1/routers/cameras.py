"""Snapshots (relayed) and paced MJPEG streams. One route per camera: an unknown camera is 404."""

from collections.abc import Awaitable, Callable

from fastapi import APIRouter, Request, Response

from arm_dashboard.api.constants import CameraRoutes
from arm_dashboard.api.v1 import handlers, responses
from arm_dashboard.api.v1.dependencies import ServicesDep
from arm_dashboard.core.typing import CameraName, ControllerRoutes

router = APIRouter(tags=['cameras'])


def _snapshot(camera: CameraName) -> Callable[[ServicesDep], Awaitable[Response]]:
    async def endpoint(services: ServicesDep) -> Response:
        return responses.relayed_stream(await services.controller.open(ControllerRoutes.snapshot(camera)))

    return endpoint


def _stream(camera: CameraName) -> Callable[[Request, ServicesDep], Awaitable[Response]]:
    async def endpoint(request: Request, services: ServicesDep) -> Response:
        # The raw query string: fps parsing is stricter than FastAPI's query parameters (legacy parity).
        return responses.mjpeg(
            await handlers.open_camera_stream(
                services.camera_stream, services.settings.camera, camera, request.url.query
            )
        )

    return endpoint


for _camera in CameraName:
    router.add_api_route(
        CameraRoutes.SNAPSHOT_TEMPLATE.format(camera=_camera.value),
        _snapshot(_camera),
        methods=['GET'],
        include_in_schema=False,
    )
    router.add_api_route(
        CameraRoutes.STREAM_TEMPLATE.format(camera=_camera.value),
        _stream(_camera),
        methods=['GET'],
        include_in_schema=False,
    )
