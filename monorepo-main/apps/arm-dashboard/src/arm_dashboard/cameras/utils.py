"""Pure helpers for MJPEG delivery: query parsing, part framing, the advertised delivery."""

from collections.abc import Collection
from typing import Final, final
from urllib.parse import parse_qs

from arm_dashboard.cameras.exceptions import InvalidStreamOptionsError
from arm_dashboard.core.settings import CameraSettings
from arm_dashboard.core.typing.camera import CameraDelivery, CameraStreamQuery


@final
class MjpegFormat:
    """``multipart/x-mixed-replace`` framing, identical to the legacy dashboard and to soarm-panel."""

    BOUNDARY: Final = 'frame'
    CONTENT_TYPE: Final = 'multipart/x-mixed-replace; boundary=frame'
    PART_CONTENT_TYPE: Final = 'image/jpeg'


def parse_fps(query: str, *, allowed: Collection[int], default: int) -> int:
    """The requested rate from a raw query string, or ``default`` when there is none.

    Strict on purpose, like the legacy proxy: only the ``fps`` key, once, spelled exactly as one of
    ``allowed`` (``02``, ``2.0`` and ``+2`` are refused). Anything else is refused before any
    upstream request is made.
    """
    params = parse_qs(query, keep_blank_values=True)
    values = params.get(CameraStreamQuery.FPS, [])
    if set(params) - {CameraStreamQuery.FPS} or len(values) > 1:
        raise InvalidStreamOptionsError('Invalid camera stream options')
    if not values:
        return default
    choices = {str(fps): fps for fps in allowed}
    if values[0] not in choices:
        options = ', '.join(str(fps) for fps in sorted(allowed))
        raise InvalidStreamOptionsError(f'{CameraStreamQuery.FPS} must be one of {options}')
    return choices[values[0]]


def mjpeg_part(frame: bytes) -> bytes:
    """One multipart part: boundary, headers, JPEG, trailing CRLF."""
    head = (
        f'--{MjpegFormat.BOUNDARY}\r\n'
        f'Content-Type: {MjpegFormat.PART_CONTENT_TYPE}\r\n'
        f'Content-Length: {len(frame)}\r\n\r\n'
    )
    return head.encode() + frame + b'\r\n'


def describe_delivery(settings: CameraSettings, *, controller_host: str, hostname: str) -> CameraDelivery:
    """What ``/api/state`` advertises. Direct hosts: the controller first, then this machine, no repeats."""
    return CameraDelivery(
        default_fps=settings.default_fps,
        direct_default_fps=settings.direct_default_fps,
        direct_hosts=tuple(dict.fromkeys((controller_host, hostname))),
        available_fps=settings.available_fps,
    )
