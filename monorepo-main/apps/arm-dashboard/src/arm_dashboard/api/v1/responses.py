"""Domain results → HTTP responses. Transport mapping only; one function per result shape."""

from collections.abc import AsyncIterator
from pathlib import Path

from fastapi.responses import FileResponse, JSONResponse, Response, StreamingResponse
from starlette.background import BackgroundTask

from arm_dashboard.api.constants import MediaTypes, ResponseHeaders
from arm_dashboard.cameras import MjpegFormat
from arm_dashboard.controller import UpstreamStream
from arm_dashboard.core.typing import ControllerState, ScanStatus, UpstreamResponse


def relayed(response: UpstreamResponse) -> Response:
    """The controller's answer as it came: status, content type and body."""
    return Response(
        response.body,
        status_code=response.status_code,
        media_type=response.content_type,
        headers=ResponseHeaders.NO_STORE_CLOSE,
    )


def controller_state(result: ControllerState | UpstreamResponse) -> Response:
    if isinstance(result, UpstreamResponse):
        return relayed(result)
    return JSONResponse(result, headers=ResponseHeaders.NO_STORE)


def relayed_stream(stream: UpstreamStream) -> StreamingResponse:
    """Relay body chunks as they arrive. The stream also closes when the browser disconnects."""
    return StreamingResponse(
        stream.chunks(),
        status_code=stream.status_code,
        media_type=stream.content_type,
        headers=ResponseHeaders.NO_STORE_CLOSE,
        background=BackgroundTask(stream.aclose),
    )


def mjpeg(parts: AsyncIterator[bytes]) -> StreamingResponse:
    return StreamingResponse(
        parts, media_type=MjpegFormat.CONTENT_TYPE, headers=ResponseHeaders.NO_STORE_CLOSE
    )


def scan_status(status: ScanStatus) -> JSONResponse:
    # exclude_unset: only the fields the collector wrote or the service set, as the legacy dashboard.
    return JSONResponse(status.model_dump(mode='json', exclude_unset=True), headers=ResponseHeaders.NO_STORE)


def jpeg(body: bytes) -> Response:
    return Response(body, media_type=MediaTypes.JPEG, headers=ResponseHeaders.NO_STORE)


def html(body: bytes) -> Response:
    return Response(body, media_type=MediaTypes.HTML, headers=ResponseHeaders.NO_STORE)


def local_file(path: Path, media_type: str) -> FileResponse:
    return FileResponse(path, media_type=media_type, headers=ResponseHeaders.NO_STORE)
