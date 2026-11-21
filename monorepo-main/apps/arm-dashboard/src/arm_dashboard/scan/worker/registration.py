import cv2
import numpy as np

from arm_dashboard.scan.worker.constants import RegistrationLimits as Limits
from arm_dashboard.scan.worker.schemas import RegistrationReason, RegistrationResult


class PlaneRegistrationProcessor:
    """Fits a candidate frame onto the anchor as a plane; rejects weak matches rather than fabricate coverage.

    Moved unchanged from the legacy collector: same detector, thresholds and order of checks.
    """

    def __init__(self) -> None:
        self._sift = cv2.SIFT.create(nfeatures=Limits.SIFT_FEATURES)
        self._matcher = cv2.BFMatcher()

    def register(self, reference: np.ndarray, candidate: np.ndarray) -> RegistrationResult:
        a, da = self._sift.detectAndCompute(cv2.cvtColor(reference, cv2.COLOR_BGR2GRAY), None)
        b, db = self._sift.detectAndCompute(cv2.cvtColor(candidate, cv2.COLOR_BGR2GRAY), None)
        found: dict[str, object] = {'reference_features': len(a), 'candidate_features': len(b)}
        if da is None or db is None or len(a) < Limits.MIN_FEATURES or len(b) < Limits.MIN_FEATURES:
            return _rejected(RegistrationReason.TOO_FEW_FEATURES, found)

        pairs = self._matcher.knnMatch(db, da, k=Limits.KNN_NEIGHBOURS)
        matches = [
            pair[0]
            for pair in pairs
            if len(pair) == Limits.KNN_NEIGHBOURS and pair[0].distance < Limits.RATIO * pair[1].distance
        ]
        found['ratio_matches'] = len(matches)
        if len(matches) < Limits.MIN_MATCHES:
            return _rejected(RegistrationReason.TOO_FEW_MATCHES, found)

        src = np.array([b[m.queryIdx].pt for m in matches], dtype=np.float32)
        dst = np.array([a[m.trainIdx].pt for m in matches], dtype=np.float32)
        h, mask = cv2.findHomography(src, dst, cv2.RANSAC, Limits.RANSAC_REPROJECTION_PX)
        if h is None or mask is None:
            return _rejected(RegistrationReason.NO_HOMOGRAPHY, found)

        keep = mask.ravel().astype(bool)
        found |= {'inliers': int(keep.sum()), 'inlier_fraction': float(keep.mean())}
        if keep.sum() < Limits.MIN_INLIERS or keep.mean() < Limits.MIN_INLIER_FRACTION:
            return _rejected(RegistrationReason.WEAK_CONSENSUS, found)

        coverage = [
            float(cv2.contourArea(cv2.convexHull(points))) / (image.shape[0] * image.shape[1])
            for points, image in [(src[keep], candidate), (dst[keep], reference)]
        ]
        found['feature_hull_fraction'] = coverage
        if min(coverage) < Limits.MIN_HULL_FRACTION:
            return _rejected(RegistrationReason.MATCHES_TOO_LOCALIZED, found)

        if not _projects_corners_soundly(h, candidate):
            return _rejected(RegistrationReason.INVALID_PROJECTION, found)
        if not _footprint_is_plausible(h, candidate, reference):
            return _rejected(RegistrationReason.IMPLAUSIBLE_FOOTPRINT, found)

        projected = cv2.perspectiveTransform(src[:, None], h)[:, 0]
        error = np.linalg.norm(projected[keep] - dst[keep], axis=1)
        return RegistrationResult.model_validate(
            found
            | {
                'accepted': True,
                'reason': RegistrationReason.ACCEPTED,
                'reprojection_p95_px': float(np.percentile(error, Limits.REPROJECTION_PERCENTILE)),
                'candidate_to_anchor': h.tolist(),
            }
        )


def _rejected(reason: RegistrationReason, found: dict[str, object]) -> RegistrationResult:
    return RegistrationResult.model_validate(found | {'accepted': False, 'reason': reason})


def _corners(image: np.ndarray) -> np.ndarray:
    height, width = image.shape[:2]
    return np.array([[0, 0], [width - 1, 0], [width - 1, height - 1], [0, height - 1]], dtype=np.float32)


def _projects_corners_soundly(h: np.ndarray, candidate: np.ndarray) -> bool:
    """Finite, and no image corner projects to (or across) the horizon line."""
    denominators = _corners(candidate) @ h[2, :2] + h[2, 2]
    return bool(
        np.isfinite(h).all()
        and not np.any(abs(denominators) < Limits.MIN_DENOMINATOR)
        and np.all(np.sign(denominators) == np.sign(denominators[0]))
    )


def _footprint_is_plausible(h: np.ndarray, candidate: np.ndarray, reference: np.ndarray) -> bool:
    """The projected candidate is a convex quad of sensible size and position on the anchor."""
    warped = cv2.perspectiveTransform(_corners(candidate)[:, None], h)[:, 0]
    area = abs(cv2.contourArea(warped.astype(np.float32))) / (reference.shape[0] * reference.shape[1])
    return bool(
        cv2.isContourConvex(warped.astype(np.float32))
        and Limits.FOOTPRINT_AREA_MIN < area < Limits.FOOTPRINT_AREA_MAX
        and not np.max(abs(warped)) > Limits.MAX_PROJECTED_PX
    )
