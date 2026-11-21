from arm_dashboard.core.exceptions import InvalidRequestError, UpstreamUnavailableError


class InvalidStreamOptionsError(InvalidRequestError):
    """The MJPEG query is not exactly ``fps=<one of the available rates>`` or empty."""


class CameraFrameUnavailableError(UpstreamUnavailableError):
    """The controller has no fresh frame for this camera (it answered something other than 200)."""
