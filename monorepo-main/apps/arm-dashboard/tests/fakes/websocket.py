"""Minimal RFC 6455 framing for tests that speak raw WebSocket on both sides of the proxy."""

import base64
import hashlib
from typing import BinaryIO

GUID = '258EAFA5-E914-47DA-95CA-C5AB0DC85B11'
SAMPLE_KEY = 'dGhlIHNhbXBsZSBub25jZQ=='
VISER_PROTOCOL = 'viser-v1.1.1'
CLIENT_MASK = b'key!'

OPCODE_TEXT = 1
OPCODE_BINARY = 2
OPCODE_CLOSE = 8
SHORT_PAYLOAD_MAX = 125
EXTENDED_16_MARKER = 126


def accept_key(key: str) -> str:
    return base64.b64encode(hashlib.sha1((key + GUID).encode()).digest()).decode()


def encode_frame(opcode: int, payload: bytes, *, masked: bool = False) -> bytes:
    """A single final frame; payloads up to 64 KiB are enough for these tests."""
    mask_bit = 0x80 if masked else 0
    if len(payload) <= SHORT_PAYLOAD_MAX:
        header = bytes([0x80 | opcode, mask_bit | len(payload)])
    else:
        header = bytes([0x80 | opcode, mask_bit | EXTENDED_16_MARKER]) + len(payload).to_bytes(2, 'big')
    if not masked:
        return header + payload
    return header + CLIENT_MASK + _unmask(payload, CLIENT_MASK)


def read_frame(reader: BinaryIO) -> tuple[int, bytes]:
    """Read one frame, unmasking if needed. Raises EOFError on a closed stream."""
    head = reader.read(2)
    if len(head) < 2:
        raise EOFError('WebSocket stream closed')
    length = head[1] & 0x7F
    if length == EXTENDED_16_MARKER:
        length = int.from_bytes(reader.read(2), 'big')
    mask = reader.read(4) if head[1] & 0x80 else b''
    payload = reader.read(length)
    return head[0] & 0x0F, _unmask(payload, mask) if mask else payload


def _unmask(payload: bytes, mask: bytes) -> bytes:
    return bytes(byte ^ mask[index % 4] for index, byte in enumerate(payload))


def upgrade_request(host: str, port: int, path: str, headers: dict[str, str]) -> bytes:
    lines = [
        f'GET {path} HTTP/1.1',
        f'Host: {host}:{port}',
        *(f'{name}: {value}' for name, value in headers.items()),
    ]
    return ('\r\n'.join(lines) + '\r\n\r\n').encode()


def viser_upgrade_headers(origin: str | None) -> dict[str, str]:
    headers = {
        'Connection': 'keep-alive, Upgrade',
        'Upgrade': 'websocket',
        'Sec-WebSocket-Version': '13',
        'Sec-WebSocket-Key': SAMPLE_KEY,
        'Sec-WebSocket-Protocol': VISER_PROTOCOL,
    }
    if origin is not None:
        headers['Origin'] = origin
    return headers
