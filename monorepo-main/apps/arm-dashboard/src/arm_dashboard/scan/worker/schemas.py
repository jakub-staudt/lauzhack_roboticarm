"""What the collector writes to disk: ``status.json``, ``frames.jsonl`` rows, registration results.

Rows and registration results are dumped with ``exclude_unset=True``: a key appears only when the
collector reached the step that sets it, exactly as the legacy dicts grew step by step.
"""

from enum import StrEnum
from typing import Any

import numpy as np
from pydantic import BaseModel

from arm_dashboard.core.typing.camera import CameraName
from arm_dashboard.core.typing.scan import ScanPhase, ScanUnit
from arm_dashboard.scan.worker.constants import CollectionFiles, MosaicLimits


class RegistrationReason(StrEnum):
    ANCHOR_VIEW = 'anchor_view'
    IDENTICAL_CACHED_FRAME = 'identical_cached_frame'
    TOO_FEW_FEATURES = 'too_few_features'
    TOO_FEW_MATCHES = 'too_few_matches'
    NO_HOMOGRAPHY = 'no_homography'
    WEAK_CONSENSUS = 'weak_consensus'
    MATCHES_TOO_LOCALIZED = 'matches_too_localized'
    INVALID_PROJECTION = 'invalid_projection'
    IMPLAUSIBLE_FOOTPRINT = 'implausible_footprint'
    ACCEPTED = 'candidate_planar_alignment'


class RegistrationResult(BaseModel):
    accepted: bool
    reason: RegistrationReason
    reference_features: int | None = None
    candidate_features: int | None = None
    ratio_matches: int | None = None
    inliers: int | None = None
    inlier_fraction: float | None = None
    feature_hull_fraction: list[float] | None = None
    reprojection_p95_px: float | None = None
    candidate_to_anchor: list[list[float]] | None = None

    @property
    def homography(self) -> np.ndarray | None:
        """Candidate-to-anchor transform of an accepted registration."""
        if self.candidate_to_anchor is None:
            return None
        return np.array(self.candidate_to_anchor)


class MosaicCoverage(BaseModel):
    anchor_pixel_bounds: list[list[int]]
    observed_output_pixels: int
    unobserved_output_pixels: int
    coverage_units: str = MosaicLimits.COVERAGE_UNITS


class FrameRecord(BaseModel):
    """One camera capture in ``frames.jsonl``."""

    index: int
    camera: CameraName
    elapsed_s: float
    controller: dict[str, Any]
    synchronized_exposure: bool = False
    file: str | None = None
    sha256: str | None = None
    request_ms: float | None = None
    width: int | None = None
    height: int | None = None
    sharpness: float | None = None
    registration: RegistrationResult | None = None
    error: str | None = None


class CollectorStatus(BaseModel):
    """The collector's ``status.json``. The HTTP process reads it as ``ScanStatus`` with extras."""

    phase: ScanPhase
    started_utc: str
    elapsed_s: float = 0
    duration_s: float
    captured_frames: int = 0
    registered_frames: int = 0
    rejected_frames: int = 0
    duplicate_frames: int = 0
    errors: int = 0
    units: ScanUnit = ScanUnit.PIXELS
    metric_calibrated: bool = False
    robot_motion_commands: int = 0
    registration_validated: bool = False
    note: str = CollectionFiles.NOTE
    coverage: MosaicCoverage | None = None
    candidate_mosaic_layers: int | None = None
    mosaic_error: str | None = None
    capture_cycles: int | None = None
