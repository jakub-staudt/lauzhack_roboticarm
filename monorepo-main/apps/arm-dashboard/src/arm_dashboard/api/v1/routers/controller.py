"""Controller state and the allow-listed commands. One explicit route per command: anything else is 404."""

from collections.abc import Awaitable, Callable

from fastapi import APIRouter, Response

from arm_dashboard.api.v1 import responses
from arm_dashboard.api.v1.dependencies import CommandBodyDep, ServicesDep
from arm_dashboard.core.typing import ControllerCommand, ControllerRoutes

router = APIRouter(tags=['controller'])


@router.get(ControllerRoutes.STATE, include_in_schema=False)
async def state(services: ServicesDep) -> Response:
    return responses.controller_state(await services.controller_state.fetch())


def _forward(command: ControllerCommand) -> Callable[[ServicesDep, bytes], Awaitable[Response]]:
    async def endpoint(services: ServicesDep, body: CommandBodyDep) -> Response:
        return responses.relayed(await services.controller.post(command, body))

    return endpoint


for _command in ControllerCommand:
    router.add_api_route(_command.value, _forward(_command), methods=['POST'], include_in_schema=False)
