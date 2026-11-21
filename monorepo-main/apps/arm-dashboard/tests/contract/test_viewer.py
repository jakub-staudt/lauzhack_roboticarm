"""Read-only Viser viewer relayed through ``/viewer/``: HTML over HTTP, scene over WebSocket."""

import http.client
import socket
from collections.abc import Iterator
from contextlib import contextmanager
from typing import BinaryIO

import pytest

from fakes import INITIAL_FRAME, VIEWER_HTML, unused_address
from fakes.websocket import (
    OPCODE_BINARY,
    OPCODE_CLOSE,
    VISER_PROTOCOL,
    encode_frame,
    read_frame,
    upgrade_request,
    viser_upgrade_headers,
)
from harness import IO_TIMEOUT_S, Dashboard

VIEWER_PATHS = ['/viewer', '/viewer/']
NORMAL_CLOSURE = (1000).to_bytes(2, 'big')


@contextmanager
def raw_upgrade(
    dashboard: Dashboard, headers: dict[str, str], path: str = '/viewer'
) -> Iterator[tuple[int, http.client.HTTPMessage, BinaryIO, socket.socket]]:
    """Send a hand-written upgrade request; yield (status, headers, reader, socket) for the rest."""
    with (
        socket.create_connection(dashboard.address, timeout=IO_TIMEOUT_S) as client,
        client.makefile('rb') as reader,
    ):
        client.sendall(upgrade_request(dashboard.host, dashboard.port, path, headers))
        status = int(reader.readline().split()[1])
        response_headers = http.client.parse_headers(reader)
        yield status, response_headers, reader, client


@pytest.mark.parametrize('path', VIEWER_PATHS)
def test_viewer_html_is_relayed(dashboard: Dashboard, path: str) -> None:
    reply = dashboard.request('GET', path)

    assert reply.status == 200
    assert reply.body == VIEWER_HTML
    assert reply.headers['Content-Type'] == 'text/html; charset=utf-8'
    assert reply.headers['Cache-Control'] == 'no-store'
    assert dashboard.viewer.request_lines() == ['GET / HTTP/1.1']


def test_viewer_query_is_not_forwarded(dashboard: Dashboard) -> None:
    assert dashboard.request('GET', '/viewer/?darkMode').status == 200
    assert dashboard.viewer.request_lines() == ['GET / HTTP/1.1']


def test_viewer_html_error_is_502(dashboard: Dashboard) -> None:
    dashboard.viewer.html_status = 500

    assert dashboard.request('GET', '/viewer/').status == 502


def test_viewer_html_is_502_when_viewer_is_down(start_dashboard) -> None:
    dashboard = start_dashboard(viewer_address=unused_address())

    assert dashboard.request('GET', '/viewer/').status == 502


@pytest.mark.parametrize('path', VIEWER_PATHS)
def test_websocket_relays_initial_frame_echo_and_close(dashboard: Dashboard, path: str) -> None:
    with raw_upgrade(dashboard, viser_upgrade_headers(dashboard.origin), path) as (
        status,
        headers,
        reader,
        client,
    ):
        assert status == 101
        assert headers['Sec-WebSocket-Protocol'] == VISER_PROTOCOL
        assert encode_frame(*read_frame(reader)) == INITIAL_FRAME

        client.sendall(encode_frame(OPCODE_BINARY, b'\x00\xff', masked=True))
        assert read_frame(reader) == (OPCODE_BINARY, b'\x00\xff')

        large = bytes(range(256)) * 4
        client.sendall(encode_frame(OPCODE_BINARY, large, masked=True))
        assert read_frame(reader) == (OPCODE_BINARY, large)

        client.sendall(encode_frame(OPCODE_CLOSE, NORMAL_CLOSURE, masked=True))
        assert read_frame(reader) == (OPCODE_CLOSE, NORMAL_CLOSURE)
        with pytest.raises(EOFError):
            read_frame(reader)

    [(request_line, upstream_headers)] = dashboard.viewer.requests
    assert request_line == 'GET / HTTP/1.1'
    assert upstream_headers['Sec-WebSocket-Protocol'] == VISER_PROTOCOL


def test_websocket_without_origin_is_allowed(dashboard: Dashboard) -> None:
    with raw_upgrade(dashboard, viser_upgrade_headers(origin=None)) as (status, _headers, _reader, _client):
        assert status == 101


def test_cross_origin_websocket_never_reaches_viewer(dashboard: Dashboard) -> None:
    with raw_upgrade(dashboard, viser_upgrade_headers('http://outside.example')) as (status, *_):
        assert status == 403
    assert dashboard.viewer.requests == []


@pytest.mark.parametrize(
    'override',
    [
        {'Sec-WebSocket-Version': '8'},
        {'Sec-WebSocket-Key': 'not-base64!'},
        {'Sec-WebSocket-Key': 'c2hvcnQ='},
        {'Sec-WebSocket-Protocol': 'chat'},
        {'Sec-WebSocket-Protocol': 'viser-v1 extra'},
        {'Connection': 'keep-alive'},
    ],
    ids=[
        'version',
        'key-not-base64',
        'key-not-16-bytes',
        'protocol-foreign',
        'protocol-malformed',
        'no-upgrade-token',
    ],
)
def test_invalid_handshake_is_400_and_never_reaches_viewer(
    dashboard: Dashboard, override: dict[str, str]
) -> None:
    headers = viser_upgrade_headers(dashboard.origin) | override

    with raw_upgrade(dashboard, headers) as (status, *_):
        assert status == 400
    assert dashboard.viewer.requests == []


def test_websocket_is_502_when_viewer_is_down(start_dashboard) -> None:
    dashboard = start_dashboard(viewer_address=unused_address())

    with raw_upgrade(dashboard, viser_upgrade_headers(dashboard.origin)) as (status, *_):
        assert status == 502
