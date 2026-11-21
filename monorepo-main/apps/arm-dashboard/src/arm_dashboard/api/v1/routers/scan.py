"""Automatic board collection: status, start/stop, and the current session's three images."""

from collections.abc import Awaitable, Callable

from fastapi import APIRouter, Depends, Response

from arm_dashboard.api.constants import ScanRoutes
from arm_dashboard.api.v1 import responses
from arm_dashboard.api.v1.dependencies import ServicesDep, read_command_body
from arm_dashboard.core.typing import ScanArtifact

router = APIRouter(tags=['scan'])


@router.get(ScanRoutes.STATUS, include_in_schema=False)
async def status(services: ServicesDep) -> Response:
    return responses.scan_status(await services.scan.status())


# The body is ignored, but read under the same limit as every POST (legacy parity: 413).
@router.post(ScanRoutes.START, dependencies=[Depends(read_command_body)], include_in_schema=False)
async def start(services: ServicesDep) -> Response:
    return responses.scan_status(await services.scan.start())


@router.post(ScanRoutes.STOP, dependencies=[Depends(read_command_body)], include_in_schema=False)
async def stop(services: ServicesDep) -> Response:
    return responses.scan_status(await services.scan.stop())


def _image(artifact: ScanArtifact) -> Callable[[ServicesDep], Awaitable[Response]]:
    async def endpoint(services: ServicesDep) -> Response:
        return responses.jpeg(await services.scan.artifact(artifact))

    return endpoint


# One route per whitelisted image: any other name never reaches the disk.
for _artifact in ScanArtifact:
    router.add_api_route(
        ScanRoutes.IMAGE_TEMPLATE.format(artifact=_artifact.value),
        _image(_artifact),
        methods=['GET'],
        include_in_schema=False,
    )
