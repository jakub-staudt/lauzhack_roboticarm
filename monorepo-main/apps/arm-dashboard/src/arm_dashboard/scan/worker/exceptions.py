from arm_dashboard.core.exceptions import DashboardError


class CollectorError(DashboardError):
    """A recoverable failure inside one collection cycle; recorded in the frame log, never fatal."""


class FrameFetchError(CollectorError):
    """The controller answered a capture request with a non-200, or with too many bytes."""


class InvalidFrameError(CollectorError):
    """The bytes are not a decodable image."""


class MosaicBudgetExceededError(CollectorError):
    """The registered views would produce a mosaic beyond the size budget."""
