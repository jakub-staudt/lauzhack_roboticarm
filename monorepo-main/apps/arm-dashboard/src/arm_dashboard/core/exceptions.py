"""The base exception hierarchy. Capability packages subclass these with their own names.

The API layer maps the *base* classes to status codes in one place, so a new subclass needs
no change there. Nothing here names a business concept (scan, camera, viewer).
"""


class DashboardError(Exception):
    """Root of every error the dashboard raises on purpose."""


class InvalidRequestError(DashboardError):
    """The caller sent something the dashboard refuses before doing any work (→ 400)."""


class NotFoundError(DashboardError):
    """The requested resource does not exist (→ 404)."""


class ConflictError(DashboardError):
    """The request is valid but clashes with current state (→ 409)."""


class UpstreamError(DashboardError):
    """A service behind the dashboard failed or answered unusably (→ 502)."""


class UpstreamUnavailableError(UpstreamError):
    """The upstream could not be reached, or timed out."""


class UpstreamPayloadTooLargeError(UpstreamError):
    """The upstream answered with more bytes than the configured limit."""
