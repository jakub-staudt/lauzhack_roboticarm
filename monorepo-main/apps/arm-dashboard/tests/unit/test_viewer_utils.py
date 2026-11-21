import pytest

from arm_dashboard.core.exceptions import InvalidRequestError
from arm_dashboard.core.typing import ViewerHandshake
from arm_dashboard.viewer import (
    CloseCodes,
    InvalidViewerHandshakeError,
    connection_tokens,
    validate_handshake,
    websocket_url,
)
from arm_dashboard.viewer.relay import sendable
from fakes.websocket import SAMPLE_KEY, VISER_PROTOCOL


def handshake(**override: object) -> ViewerHandshake:
    fields: dict[str, object] = {
        'key': SAMPLE_KEY,
        'version': '13',
        'protocol': VISER_PROTOCOL,
        'connection_tokens': connection_tokens('keep-alive, Upgrade'),
    }
    return ViewerHandshake.model_validate(fields | override)


def test_viser_handshake_is_accepted() -> None:
    validate_handshake(handshake())


@pytest.mark.parametrize(
    'override',
    [
        {'version': '8'},
        {'version': None},
        {'key': 'not-base64!'},
        {'key': 'c2hvcnQ='},
        {'protocol': 'chat'},
        {'protocol': 'viser-v1 extra'},
        {'protocol': ''},
        {'connection_tokens': connection_tokens('keep-alive')},
    ],
    ids=[
        'version',
        'no-version',
        'key-not-base64',
        'key-not-16-bytes',
        'foreign',
        'malformed',
        'empty',
        'no-upgrade',
    ],
)
def test_non_viser_handshakes_are_invalid_requests(override: dict[str, object]) -> None:
    with pytest.raises(InvalidViewerHandshakeError) as raised:
        validate_handshake(handshake(**override))

    assert isinstance(raised.value, InvalidRequestError)


def test_connection_tokens_are_trimmed_and_lowercased() -> None:
    assert connection_tokens(' keep-alive ,Upgrade,, ') == frozenset({'keep-alive', 'upgrade'})


@pytest.mark.parametrize(
    ('base', 'expected'),
    [('http://100.85.56.14:8766/', 'ws://100.85.56.14:8766/'), ('https://viewer:443/', 'wss://viewer:443/')],
)
def test_websocket_url_follows_the_http_scheme(base: str, expected: str) -> None:
    assert websocket_url(base) == expected


@pytest.mark.parametrize('base', ['ftp://viewer/', 'viewer:8766'])
def test_websocket_url_rejects_non_http(base: str) -> None:
    with pytest.raises(ValueError):
        websocket_url(base)


@pytest.mark.parametrize(
    ('code', 'expected'),
    [(1000, 1000), (1001, 1001), (4000, 4000), (None, 1000), (1005, 1000), (1006, 1000), (1015, 1000)],
)
def test_reserved_close_codes_are_never_sent(code: int | None, expected: int) -> None:
    assert sendable(code, fallback=CloseCodes.NORMAL) == expected
