import json
import logging
from http import HTTPStatus
from typing import Final, final

from arm_dashboard.controller.client import ControllerClient
from arm_dashboard.controller.exceptions import InvalidControllerStateError
from arm_dashboard.core.typing.camera import CameraDelivery
from arm_dashboard.core.typing.controller import (
    ControllerCommand,
    ControllerRoutes,
    ControllerState,
    UpstreamResponse,
)
from arm_dashboard.core.typing.viewer import ViewerRoutes

logger = logging.getLogger(__name__)


@final
class StateKeys:
    """Controller state fields used or enriched by the dashboard."""

    CAMERA_DELIVERY: Final = 'camera_delivery'
    CAPABILITIES: Final = 'capabilities'
    OPERATOR_COMMANDS: Final = 'operator_commands'
    VIEWER: Final = 'viewer'
    VIEWER_PROXY_URL: Final = 'proxy_url'


class ControllerStateService:
    """The controller state as the browser should see it through the dashboard.

    Adds camera delivery, the viewer URL, and commands available through this dashboard.
    A non-200 answer preserves the controller status and error body.
    """

    def __init__(
        self,
        client: ControllerClient,
        delivery: CameraDelivery,
        viewer_proxy_url: str = ViewerRoutes.PROXY_URL,
    ) -> None:
        self._client = client
        self._delivery = delivery
        self._viewer_proxy_url = viewer_proxy_url

    async def fetch(self) -> ControllerState | UpstreamResponse:
        response = await self._client.get(ControllerRoutes.STATE)
        if response.status_code != HTTPStatus.OK:
            return response
        state = self._decode(response.body)
        state[StateKeys.CAMERA_DELIVERY] = self._delivery.model_dump(mode='json')
        state[StateKeys.OPERATOR_COMMANDS] = self._operator_commands(state)
        viewer = state.get(StateKeys.VIEWER)
        if isinstance(viewer, dict):
            viewer[StateKeys.VIEWER_PROXY_URL] = self._viewer_proxy_url
        return state

    @staticmethod
    def _operator_commands(state: ControllerState) -> list[str]:
        capabilities = state.get(StateKeys.CAPABILITIES)
        if not isinstance(capabilities, dict) or StateKeys.OPERATOR_COMMANDS not in capabilities:
            # Workbench releases before this field supported the existing dashboard routes.
            return [command.value for command in ControllerCommand]
        advertised = capabilities[StateKeys.OPERATOR_COMMANDS]
        if not isinstance(advertised, list):
            return []
        return [command.value for command in ControllerCommand if command.value in advertised]

    @staticmethod
    def _decode(body: bytes) -> ControllerState:
        try:
            state = json.loads(body)
        except ValueError as exc:  # JSONDecodeError and UnicodeDecodeError
            logger.warning('Controller state is not JSON: %s', exc)
            raise InvalidControllerStateError('Controller state is not JSON') from exc
        if not isinstance(state, dict):
            # The legacy proxy answered 502 here too; kept for parity.
            logger.warning('Controller state is a JSON %s, not an object', type(state).__name__)
            raise InvalidControllerStateError('Controller state is not a JSON object')
        return state
