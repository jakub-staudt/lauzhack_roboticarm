from arm_dashboard.core.exceptions import ConflictError, NotFoundError


class ScanAlreadyRunningError(ConflictError):
    """A collector is still running; only one session at a time."""


class ScanRuntimeMissingError(ConflictError):
    """The collector's OpenCV dependencies are not installed in this environment."""


class ScanArtifactNotFoundError(NotFoundError):
    """No current session, or the session has not produced this image yet."""
