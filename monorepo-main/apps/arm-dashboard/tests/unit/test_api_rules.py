from http import HTTPStatus

import pytest

from arm_dashboard.api.errors import RequestBodyTooLargeError, error_body, status_for
from arm_dashboard.api.middlewares import Decision, same_origin
from arm_dashboard.cameras import CameraFrameUnavailableError, InvalidStreamOptionsError
from arm_dashboard.controller import ControllerUnavailableError, InvalidControllerStateError
from arm_dashboard.core.exceptions import DashboardError
from arm_dashboard.scan import ScanAlreadyRunningError, ScanArtifactNotFoundError, ScanRuntimeMissingError
from arm_dashboard.viewer import InvalidViewerHandshakeError, ViewerPayloadTooLargeError


@pytest.mark.parametrize(
    ('origin', 'host', 'decision'),
    [
        (None, '127.0.0.1:8780', Decision.ALLOW),
        ('http://127.0.0.1:8780', '127.0.0.1:8780', Decision.ALLOW),
        ('http://localhost:8780', '127.0.0.1:8780', Decision.DENY),
        ('https://127.0.0.1:8780', '127.0.0.1:8780', Decision.DENY),
        ('http://untrusted.invalid', '127.0.0.1:8780', Decision.DENY),
        ('http://', None, Decision.ALLOW),
        ('http://x', None, Decision.DENY),
    ],
)
def test_same_origin(origin: str | None, host: str | None, decision: Decision) -> None:
    assert same_origin(origin, host) is decision


@pytest.mark.parametrize(
    ('error', 'status'),
    [
        (InvalidStreamOptionsError('x'), HTTPStatus.BAD_REQUEST),
        (InvalidViewerHandshakeError('x'), HTTPStatus.BAD_REQUEST),
        (ScanArtifactNotFoundError('x'), HTTPStatus.NOT_FOUND),
        (ScanAlreadyRunningError('x'), HTTPStatus.CONFLICT),
        (ScanRuntimeMissingError('x'), HTTPStatus.CONFLICT),
        (ControllerUnavailableError('x'), HTTPStatus.BAD_GATEWAY),
        (InvalidControllerStateError('x'), HTTPStatus.BAD_GATEWAY),
        (CameraFrameUnavailableError('x'), HTTPStatus.BAD_GATEWAY),
        (ViewerPayloadTooLargeError('x'), HTTPStatus.BAD_GATEWAY),
        (RequestBodyTooLargeError('x'), HTTPStatus.REQUEST_ENTITY_TOO_LARGE),
        (DashboardError('x'), HTTPStatus.INTERNAL_SERVER_ERROR),
    ],
    ids=lambda value: type(value).__name__ if isinstance(value, Exception) else str(value),
)
def test_every_domain_error_maps_by_its_base_class(error: DashboardError, status: HTTPStatus) -> None:
    assert status_for(error) is status


def test_conflicts_carry_error_for_scan_html_and_others_detail() -> None:
    assert error_body(ScanAlreadyRunningError('busy')) == {'error': 'busy'}
    assert error_body(ControllerUnavailableError('down')) == {'detail': 'down'}
