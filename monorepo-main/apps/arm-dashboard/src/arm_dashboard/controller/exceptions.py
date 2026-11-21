from arm_dashboard.core.exceptions import (
    UpstreamError,
    UpstreamPayloadTooLargeError,
    UpstreamUnavailableError,
)


class ControllerUnavailableError(UpstreamUnavailableError):
    """soarm-panel could not be reached, timed out, or dropped the connection."""


class ControllerPayloadTooLargeError(UpstreamPayloadTooLargeError):
    """soarm-panel answered with more bytes than the caller allows (e.g. a runaway camera frame)."""


class InvalidControllerStateError(UpstreamError):
    """``/api/state`` answered 200 with something other than a JSON object."""
