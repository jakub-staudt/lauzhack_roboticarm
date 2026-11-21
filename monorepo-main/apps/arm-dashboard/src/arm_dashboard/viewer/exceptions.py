from arm_dashboard.core.exceptions import (
    InvalidRequestError,
    UpstreamPayloadTooLargeError,
    UpstreamUnavailableError,
)


class InvalidViewerHandshakeError(InvalidRequestError):
    """The browser's WebSocket upgrade is not a Viser handshake; Viser is never contacted."""


class ViewerUnavailableError(UpstreamUnavailableError):
    """Viser could not be reached, refused the upgrade, or answered the page with a non-200."""


class ViewerPayloadTooLargeError(UpstreamPayloadTooLargeError):
    """The Viser page is larger than the configured limit."""
