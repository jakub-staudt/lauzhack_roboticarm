"""Pure helpers for the Viser relay: handshake rules and the upstream WebSocket URL."""

import base64
import binascii
import re
from typing import Final, final

from arm_dashboard.core.typing.viewer import ViewerHandshake
from arm_dashboard.viewer.exceptions import InvalidViewerHandshakeError


@final
class ViserHandshakeRules:
    """What a browser upgrade must look like before the relay contacts Viser (legacy parity)."""

    VERSION: Final = '13'
    KEY_BYTES: Final = 16
    PROTOCOL: Final = re.compile(r'viser-v[0-9A-Za-z.+-]+')
    CONNECTION_TOKEN: Final = 'upgrade'


@final
class WebSocketSchemes:
    BY_HTTP_SCHEME: Final = {'http': 'ws', 'https': 'wss'}


def validate_handshake(handshake: ViewerHandshake) -> None:
    """Raise ``InvalidViewerHandshakeError`` unless this is a version-13 upgrade offering a Viser protocol.

    uvicorn already refuses a bad version or key itself; the checks are repeated so this function
    states the whole rule, and so the HTTP route can apply it to an upgrade uvicorn did not
    recognise (``Upgrade: websocket`` without ``Connection: upgrade``).
    """
    if handshake.version != ViserHandshakeRules.VERSION:
        raise InvalidViewerHandshakeError('Unsupported WebSocket version')
    if not _is_valid_key(handshake.key):
        raise InvalidViewerHandshakeError('Invalid WebSocket key')
    if not ViserHandshakeRules.PROTOCOL.fullmatch(handshake.protocol):
        raise InvalidViewerHandshakeError('Not a Viser WebSocket protocol')
    if ViserHandshakeRules.CONNECTION_TOKEN not in handshake.connection_tokens:
        raise InvalidViewerHandshakeError('Connection header does not request an upgrade')


def connection_tokens(header: str) -> frozenset[str]:
    """``Connection: keep-alive, Upgrade`` → ``{'keep-alive', 'upgrade'}``."""
    return frozenset(token.strip().lower() for token in header.split(',') if token.strip())


def websocket_url(base_url: str) -> str:
    """``http://host:8766/`` → ``ws://host:8766/``. Viser serves its socket at the page path."""
    scheme, separator, rest = base_url.partition('://')
    if not separator or scheme not in WebSocketSchemes.BY_HTTP_SCHEME:
        raise ValueError(f'Expected an http(s) URL, got {base_url!r}')
    return f'{WebSocketSchemes.BY_HTTP_SCHEME[scheme]}://{rest}'


def _is_valid_key(key: str) -> bool:
    try:
        return len(base64.b64decode(key, validate=True)) == ViserHandshakeRules.KEY_BYTES
    except (ValueError, binascii.Error):
        return False
