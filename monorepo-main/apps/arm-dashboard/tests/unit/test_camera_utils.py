import pytest

from arm_dashboard.cameras import (
    InvalidStreamOptionsError,
    MjpegFormat,
    describe_delivery,
    mjpeg_part,
    parse_fps,
)
from arm_dashboard.core.exceptions import InvalidRequestError
from arm_dashboard.core.settings import CameraSettings

ALLOWED = (2, 10, 15)


@pytest.mark.parametrize(('query', 'fps'), [('', 2), ('fps=2', 2), ('fps=10', 10), ('fps=15', 15)])
def test_accepted_queries(query: str, fps: int) -> None:
    assert parse_fps(query, allowed=ALLOWED, default=2) == fps


@pytest.mark.parametrize(
    'query',
    ['fps=3', 'fps=02', 'fps=2.0', 'fps=+2', 'fps= 2', 'fps=', 'fps=two', 'fps=2&fps=2', 'fps=2&q=1', 'q=1'],
)
def test_rejected_queries_are_invalid_requests(query: str) -> None:
    with pytest.raises(InvalidStreamOptionsError) as raised:
        parse_fps(query, allowed=ALLOWED, default=2)

    assert isinstance(raised.value, InvalidRequestError)


def test_allowed_rates_come_from_the_caller() -> None:
    assert parse_fps('fps=5', allowed=(2, 5), default=2) == 5
    with pytest.raises(InvalidStreamOptionsError, match='fps must be one of 2, 5'):
        parse_fps('fps=15', allowed=(5, 2), default=2)


def test_part_framing_matches_legacy_bytes() -> None:
    assert mjpeg_part(b'jpeg') == b'--frame\r\nContent-Type: image/jpeg\r\nContent-Length: 4\r\n\r\njpeg\r\n'


def test_content_type_names_the_part_boundary() -> None:
    assert MjpegFormat.CONTENT_TYPE.endswith(f'boundary={MjpegFormat.BOUNDARY}')


def test_delivery_lists_controller_then_this_host() -> None:
    delivery = describe_delivery(
        CameraSettings(), controller_host='100.85.56.14', hostname='ramion-server-01'
    )

    assert delivery.direct_hosts == ('100.85.56.14', 'ramion-server-01')
    assert (delivery.default_fps, delivery.direct_default_fps, delivery.available_fps) == (2, 10, (2, 10, 15))


def test_delivery_does_not_repeat_a_host() -> None:
    delivery = describe_delivery(CameraSettings(), controller_host='box', hostname='box')

    assert delivery.direct_hosts == ('box',)
