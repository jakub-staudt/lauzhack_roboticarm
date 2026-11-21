"""Every tuning number of the collector, named. Values are the legacy collector's, unchanged."""

from typing import Final, final

from arm_dashboard.core.typing.camera import CameraName


@final
class RegistrationLimits:
    """When a candidate frame counts as a plausible planar view of the anchor."""

    SIFT_FEATURES: Final = 4000
    # Both frames need this many keypoints, and the match must keep this many pairs.
    MIN_FEATURES: Final = 12
    MIN_MATCHES: Final = 12
    # Lowe's ratio test: best match must be clearly better than the second best.
    RATIO: Final = 0.7
    KNN_NEIGHBOURS: Final = 2
    RANSAC_REPROJECTION_PX: Final = 3.0
    MIN_INLIERS: Final = 12
    MIN_INLIER_FRACTION: Final = 0.5
    # Matches must span at least this share of each image, not one textured corner.
    MIN_HULL_FRACTION: Final = 0.03
    # Corner projection denominators closer to zero than this mean a degenerate homography.
    MIN_DENOMINATOR: Final = 1e-8
    # Projected footprint, as a share of the anchor's area, must lie strictly inside this range.
    FOOTPRINT_AREA_MIN: Final = 0.01
    FOOTPRINT_AREA_MAX: Final = 16
    MAX_PROJECTED_PX: Final = 4000
    REPROJECTION_PERCENTILE: Final = 95


@final
class MosaicLimits:
    """Memory and size budget of the candidate mosaic."""

    MAX_SIDE_PX: Final = 4096
    MAX_PIXELS: Final = 8_000_000
    # Registered views kept as layers; later accepted views only count as registered.
    MAX_LAYERS: Final = 16
    # A view becomes a layer only if it adds this share of the anchor's area as new coverage.
    MIN_NEW_COVERAGE_FRACTION: Final = 0.03
    COVERAGE_UNITS: Final = 'anchor_camera_pixels_not_mm'


@final
class CollectionSettings:
    """Cadence and transport of one collection session."""

    INTERVAL_S: Final = 1.0
    # One controller state request and one snapshot per camera, concurrently.
    FETCH_WORKERS: Final = 3
    FETCH_TIMEOUT_S: Final = 3.0
    FRAME_LIMIT_BYTES: Final = 2_000_000
    CAMERAS: Final = (CameraName.OVERHEAD, CameraName.WRIST)
    ANCHOR_CAMERA: Final = CameraName.OVERHEAD
    # Only these controller fields are logged; tokens and ownership never reach disk.
    STATE_FIELDS: Final = ('connected', 'loop_ok', 'age_s', 'sequence', 'measured', 'faults')
    ROW_ELAPSED_DIGITS: Final = 3
    STATUS_ELAPSED_DIGITS: Final = 2
    REQUEST_MS_DIGITS: Final = 3


@final
class CollectionFiles:
    """Collector-only files; the HTTP process never exposes them."""

    FRAME_LOG: Final = 'frames.jsonl'
    RAW_FRAME_TEMPLATE: Final = '{index:05d}-{camera}.jpg'
    STAGING_SUFFIX: Final = '.next'
    STARTED_UTC_FORMAT: Final = '%Y-%m-%dT%H:%M:%SZ'
    NOTE: Final = 'Automatic camera collection; unknown areas and scale are not inferred.'
