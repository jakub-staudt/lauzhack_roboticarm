"""Construction of everything the handlers use, and the dependencies that hand it out.

Built once per process in the lifespan (``build_services``) and published on ``app.state``;
dependencies only read it. No business decisions here.
"""

import socket
from collections.abc import AsyncGenerator, Callable
from contextlib import asynccontextmanager
from dataclasses import dataclass
from typing import Annotated

import httpx
from fastapi import Depends
from starlette.requests import HTTPConnection, Request

from arm_dashboard.api.constants import AppStateKeys, Headers
from arm_dashboard.api.errors import RequestBodyTooLargeError
from arm_dashboard.cameras import CameraSnapshotProcessor, CameraStreamService, describe_delivery
from arm_dashboard.controller import ControllerClient, ControllerStateService
from arm_dashboard.core.settings import Settings
from arm_dashboard.scan import ScanSessionService, ScanStorage, ScanWorkerRunner
from arm_dashboard.viewer import ViewerPageProcessor, ViewerSocketRelay, websocket_url


@dataclass(frozen=True)
class UpstreamClients:
    controller: httpx.AsyncClient
    viewer: httpx.AsyncClient


@dataclass(frozen=True)
class Services:
    """Everything a request may need, assembled once. Holds live objects, so not a schema."""

    settings: Settings
    controller: ControllerClient
    controller_state: ControllerStateService
    camera_stream: CameraStreamService
    viewer_page: ViewerPageProcessor
    viewer_relay: ViewerSocketRelay
    scan: ScanSessionService


ServicesFactory = Callable[[Settings, UpstreamClients], Services]


@asynccontextmanager
async def open_upstream_clients(settings: Settings) -> AsyncGenerator[UpstreamClients]:
    # trust_env=False: never route controller or viewer traffic through an HTTP(S)_PROXY from the
    # environment; the legacy proxy (http.client) never did.
    async with (
        httpx.AsyncClient(
            base_url=str(settings.controller.base_url), timeout=settings.controller.timeout_s, trust_env=False
        ) as controller,
        httpx.AsyncClient(
            base_url=str(settings.viewer.base_url), timeout=settings.viewer.io_timeout_s, trust_env=False
        ) as viewer,
    ):
        yield UpstreamClients(controller=controller, viewer=viewer)


def build_services(
    settings: Settings, clients: UpstreamClients, *, scan_runner: ScanWorkerRunner | None = None
) -> Services:
    controller = ControllerClient(clients.controller)
    delivery = describe_delivery(
        settings.camera,
        controller_host=settings.controller.base_url.host or '',
        hostname=socket.gethostname(),
    )
    snapshots = CameraSnapshotProcessor(
        controller, limit_bytes=settings.camera.frame_limit_bytes, timeout_s=settings.camera.frame_timeout_s
    )
    return Services(
        settings=settings,
        controller=controller,
        controller_state=ControllerStateService(controller, delivery),
        camera_stream=CameraStreamService(snapshots),
        viewer_page=ViewerPageProcessor(clients.viewer, limit_bytes=settings.viewer.page_limit_bytes),
        viewer_relay=ViewerSocketRelay(
            websocket_url(str(settings.viewer.base_url)),
            open_timeout_s=settings.viewer.io_timeout_s,
            idle_timeout_s=settings.viewer.idle_timeout_s,
        ),
        scan=ScanSessionService(
            ScanStorage(settings.scan.storage_dir),
            scan_runner or ScanWorkerRunner(),
            controller_url=settings.controller.base_url,
            duration_s=settings.scan.duration_s,
        ),
    )


def get_services(connection: HTTPConnection) -> Services:
    services = getattr(connection.app.state, AppStateKeys.SERVICES)
    assert isinstance(services, Services)
    return services


ServicesDep = Annotated[Services, Depends(get_services)]


async def read_command_body(request: Request, services: ServicesDep) -> bytes:
    """The POST body, at most ``controller.command_body_limit_bytes``; larger is refused with 413.

    Checks the declared length first (no read at all, as the legacy proxy), then caps the actual
    read, so a lying ``Content-Length`` cannot push more through.
    """
    limit = services.settings.controller.command_body_limit_bytes
    declared = request.headers.get(Headers.CONTENT_LENGTH)
    if declared is not None and declared.isdigit() and int(declared) > limit:
        raise RequestBodyTooLargeError(f'Request body is larger than {limit} bytes')
    body = bytearray()
    async for chunk in request.stream():
        body += chunk
        if len(body) > limit:
            raise RequestBodyTooLargeError(f'Request body is larger than {limit} bytes')
    return bytes(body)


CommandBodyDep = Annotated[bytes, Depends(read_command_body)]
