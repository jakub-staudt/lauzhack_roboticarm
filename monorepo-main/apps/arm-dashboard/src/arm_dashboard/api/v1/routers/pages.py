"""Serve the dashboard-owned operator panel, map, and scan pages."""

from collections.abc import Awaitable, Callable

from fastapi import APIRouter, Response

from arm_dashboard.api.constants import LocalPages
from arm_dashboard.api.v1 import responses
from arm_dashboard.api.v1.dependencies import ServicesDep

router = APIRouter(tags=['pages'])


def _local_page(filename: str, media_type: str) -> Callable[[ServicesDep], Awaitable[Response]]:
    async def endpoint(services: ServicesDep) -> Response:
        return responses.local_file(services.settings.server.static_dir / filename, media_type)

    return endpoint


for _path, (_filename, _media_type) in LocalPages.BY_PATH.items():
    router.add_api_route(_path, _local_page(_filename, _media_type), methods=['GET'], include_in_schema=False)
