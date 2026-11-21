import base64
import hashlib
import http.client
import json
import socket
import socketserver
import sys
import threading
import unittest
from contextlib import contextmanager
from http.server import ThreadingHTTPServer
from pathlib import Path
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import server
import viewer_proxy


KEY = 'dGhlIHNhbXBsZSBub25jZQ=='
PROTOCOL = 'viser-v1.1.1'
GUID = '258EAFA5-E914-47DA-95CA-C5AB0DC85B11'
HTML = b'<html><title>Viser</title></html>'


@contextmanager
def serve(httpd):
    worker = threading.Thread(target=httpd.serve_forever, daemon=True)
    worker.start()
    try:
        yield httpd.server_address
    finally:
        httpd.shutdown()
        httpd.server_close()
        worker.join(2)


def send_response(sock, mime, body):
    sock.sendall(f'HTTP/1.1 200 OK\r\nContent-Type: {mime}\r\n'
                 f'Content-Length: {len(body)}\r\nConnection: close\r\n\r\n'.encode() + body)


def frame(opcode, payload, *, masked=False):
    if masked:
        mask = b'key!'
        return (bytes([0x80 | opcode, 0x80 | len(payload)]) + mask
                + bytes(byte ^ mask[index % 4] for index, byte in enumerate(payload)))
    return bytes([0x80 | opcode, len(payload)]) + payload


class FakeViewer(socketserver.StreamRequestHandler):
    def handle(self):
        self.connection.settimeout(3)
        request_line = self.rfile.readline().decode().strip()
        headers = http.client.parse_headers(self.rfile)
        self.server.requests.append((request_line, headers))
        if request_line == 'GET /api/state HTTP/1.1':
            send_response(self.connection, 'application/json', b'{"viewer":{"port":8766}}')
            return
        if headers.get('Upgrade') != 'websocket':
            send_response(self.connection, 'text/html', HTML)
            return
        accept = base64.b64encode(hashlib.sha1((headers['Sec-WebSocket-Key'] + GUID).encode()).digest())
        # Send an initial frame in the same write as the 101 response.
        self.connection.sendall(
            b'HTTP/1.1 101 Switching Protocols\r\nUpgrade: websocket\r\nConnection: Upgrade\r\n'
            b'Sec-WebSocket-Accept: ' + accept + b'\r\nSec-WebSocket-Protocol: '
            + headers['Sec-WebSocket-Protocol'].encode() + b'\r\n\r\n' + frame(2, b'scene')
        )
        for _ in range(2):
            kind, length = self.rfile.read(2)
            mask = self.rfile.read(4)
            payload = self.rfile.read(length & 127)
            decoded = bytes(byte ^ mask[index % 4] for index, byte in enumerate(payload))
            self.connection.sendall(frame(kind & 15, decoded))


class ViewerProxyTest(unittest.TestCase):
    @contextmanager
    def servers(self):
        upstream = socketserver.ThreadingTCPServer(('127.0.0.1', 0), FakeViewer)
        upstream.daemon_threads = True
        upstream.requests = []
        with serve(upstream) as address, patch.object(viewer_proxy, 'VIEWER_UPSTREAM', address), \
                patch.object(server, 'UPSTREAM', address):
            with serve(ThreadingHTTPServer(('127.0.0.1', 0), server.Handler)) as dashboard:
                yield dashboard, upstream.requests

    def test_html_and_viewer_metadata_use_the_dashboard_origin(self):
        with self.servers() as (dashboard, requests):
            connection = http.client.HTTPConnection(*dashboard, timeout=3)
            try:
                connection.request('GET', '/viewer/?darkMode')
                response = connection.getresponse()
                self.assertEqual(response.status, 200)
                self.assertEqual(response.read(), HTML)
                connection.request('GET', '/api/state')
                response = connection.getresponse()
                self.assertEqual(response.status, 200)
                self.assertEqual(json.loads(response.read())['viewer']['proxy_url'], '/viewer/')
            finally:
                connection.close()
            self.assertEqual([request for request, _ in requests],
                             ['GET / HTTP/1.1', 'GET /api/state HTTP/1.1'])

    def test_websocket_relays_initial_frame_binary_echo_and_close(self):
        with self.servers() as (dashboard, requests):
            with socket.create_connection(dashboard, timeout=3) as client, client.makefile('rb') as reader:
                client.sendall(
                    f'GET /viewer HTTP/1.1\r\nHost: {dashboard[0]}:{dashboard[1]}\r\n'
                    f'Origin: http://{dashboard[0]}:{dashboard[1]}\r\nConnection: keep-alive, Upgrade\r\n'
                    f'Upgrade: websocket\r\nSec-WebSocket-Version: 13\r\nSec-WebSocket-Key: {KEY}\r\n'
                    f'Sec-WebSocket-Protocol: {PROTOCOL}\r\n\r\n'.encode()
                )
                self.assertTrue(reader.readline().startswith(b'HTTP/1.1 101 '))
                headers = http.client.parse_headers(reader)
                self.assertEqual(headers['Sec-WebSocket-Protocol'], PROTOCOL)
                self.assertEqual(reader.read(7), frame(2, b'scene'))
                client.sendall(frame(2, b'\x00\xff', masked=True))
                self.assertEqual(reader.read(4), frame(2, b'\x00\xff'))
                client.sendall(frame(8, b'\x03\xe8', masked=True))
                self.assertEqual(reader.read(4), frame(8, b'\x03\xe8'))
                self.assertEqual(reader.read(1), b'')
            self.assertEqual(requests[0][0], 'GET / HTTP/1.1')
            self.assertEqual(requests[0][1]['Sec-WebSocket-Protocol'], PROTOCOL)

    def test_rejects_cross_origin_upgrade_without_contacting_viewer(self):
        with self.servers() as (dashboard, requests):
            connection = http.client.HTTPConnection(*dashboard, timeout=3)
            try:
                connection.request('GET', '/viewer', headers={
                    'Upgrade': 'websocket', 'Origin': 'http://outside.example',
                })
                response = connection.getresponse()
                self.assertEqual(response.status, 403)
                response.read()
            finally:
                connection.close()
            self.assertEqual(requests, [])
