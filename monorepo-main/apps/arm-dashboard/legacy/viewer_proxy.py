"""Serve the read-only Viser viewer through the dashboard's existing port."""

import base64
import binascii
import http.client
import re
import select
import socket


VIEWER_UPSTREAM = ('100.85.56.14', 8766)
HANDSHAKE_LIMIT = 16 * 1024
IO_TIMEOUT = 8
IDLE_TIMEOUT = 90
VISER_PROTOCOL = re.compile(r'viser-v[0-9A-Za-z.+-]+')


def proxy_viewer(handler):
    if handler.headers.get('Upgrade', '').lower() == 'websocket':
        _proxy_websocket(handler)
    else:
        _proxy_html(handler)


def _proxy_html(handler):
    connection = http.client.HTTPConnection(*VIEWER_UPSTREAM, timeout=IO_TIMEOUT)
    try:
        connection.request('GET', '/')
        response = connection.getresponse()
        if response.status != 200:
            handler.send_error(502, 'Viewer unavailable')
            return
        body = response.read(4 * 1024 * 1024 + 1)
        if len(body) > 4 * 1024 * 1024:
            handler.send_error(502, 'Viewer response too large')
            return
    except (OSError, http.client.HTTPException):
        handler.send_error(502, 'Viewer unavailable')
        return
    finally:
        connection.close()
    handler.send_response(200)
    handler.send_header('Content-Type', 'text/html; charset=utf-8')
    handler.send_header('Content-Length', str(len(body)))
    handler.send_header('Cache-Control', 'no-store')
    handler.end_headers()
    try:
        handler.wfile.write(body)
    except OSError:
        handler.close_connection = True


def _proxy_websocket(handler):
    origin = handler.headers.get('Origin')
    if origin and origin != 'http://' + handler.headers.get('Host', ''):
        handler.send_error(403)
        return

    key = handler.headers.get('Sec-WebSocket-Key', '')
    version = handler.headers.get('Sec-WebSocket-Version')
    protocol = handler.headers.get('Sec-WebSocket-Protocol', '')
    connection_tokens = handler.headers.get('Connection', '').lower().split(',')
    try:
        valid_key = len(base64.b64decode(key, validate=True)) == 16
    except (ValueError, binascii.Error):
        valid_key = False
    if (version != '13' or not valid_key or not VISER_PROTOCOL.fullmatch(protocol)
            or 'upgrade' not in {token.strip() for token in connection_tokens}):
        handler.send_error(400, 'Invalid Viser WebSocket handshake')
        return

    handler.close_connection = True
    upgraded = False
    try:
        with socket.create_connection(VIEWER_UPSTREAM, timeout=IO_TIMEOUT) as upstream:
            request = (
                f'GET / HTTP/1.1\r\nHost: {VIEWER_UPSTREAM[0]}:{VIEWER_UPSTREAM[1]}\r\n'
                'Connection: Upgrade\r\nUpgrade: websocket\r\n'
                f'Sec-WebSocket-Key: {key}\r\nSec-WebSocket-Version: 13\r\n'
                f'Sec-WebSocket-Protocol: {protocol}\r\n\r\n'
            )
            upstream.sendall(request.encode('ascii'))
            response = _read_upgrade(upstream)
            handler.connection.settimeout(IO_TIMEOUT)
            upgraded = True
            handler.connection.sendall(response)
            _relay_websocket(handler.connection, upstream)
    except (OSError, ValueError):
        if not upgraded:
            handler.send_error(502, 'Viewer WebSocket unavailable')


def _read_upgrade(upstream):
    response = bytearray()
    while b'\r\n\r\n' not in response:
        chunk = upstream.recv(min(4096, HANDSHAKE_LIMIT - len(response)))
        if not chunk:
            raise ValueError('Incomplete viewer WebSocket handshake')
        response.extend(chunk)
    if not response.startswith(b'HTTP/1.1 101 '):
        raise ValueError('Viewer refused WebSocket upgrade')
    # A first frame can arrive in the same read as the upgrade response.
    return response


def _relay_websocket(client, upstream):
    peers = {client: upstream, upstream: client}
    while True:
        readable, _, _ = select.select(list(peers), [], [], IDLE_TIMEOUT)
        if not readable:
            return
        for source in readable:
            data = source.recv(65536)
            if not data:
                return
            peers[source].sendall(data)
