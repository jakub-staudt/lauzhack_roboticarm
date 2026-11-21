import cv2
import numpy as np

from arm_dashboard.scan.worker.constants import MosaicLimits as Limits
from arm_dashboard.scan.worker.exceptions import MosaicBudgetExceededError
from arm_dashboard.scan.worker.schemas import MosaicCoverage

Layer = tuple[np.ndarray, np.ndarray]
"""A registered view and its candidate-to-anchor homography."""


class MosaicRenderer:
    """Lays registered views onto the anchor's pixel grid. Unobserved pixels stay black.

    Moved unchanged from the legacy collector.
    """

    def render(self, anchor: np.ndarray, layers: list[Layer]) -> tuple[np.ndarray, MosaicCoverage]:
        corners = [_outline(anchor)]
        for image, h in layers:
            corners.append(cv2.perspectiveTransform(_outline(image)[:, None], h)[:, 0])
        points = np.concatenate(corners)
        low = np.floor(points.min(axis=0)).astype(int)
        high = np.ceil(points.max(axis=0)).astype(int)
        width, height = high - low
        if width > Limits.MAX_SIDE_PX or height > Limits.MAX_SIDE_PX or width * height > Limits.MAX_PIXELS:
            raise MosaicBudgetExceededError('Mosaic bounds exceed collection budget')
        translate = np.array([[1.0, 0.0, -low[0]], [0.0, 1.0, -low[1]], [0.0, 0.0, 1.0]])
        mosaic = np.zeros((height, width, 3), np.uint8)
        covered = np.zeros((height, width), np.uint8)
        # Lay registered details down first, then the anchor; overlapping pixels are not new coverage.
        for image, h in [*layers, (anchor, np.eye(3))]:
            transform = translate @ h
            warped = cv2.warpPerspective(image, transform, (width, height))
            mask = cv2.warpPerspective(
                np.full(image.shape[:2], 255, np.uint8), transform, (width, height), flags=cv2.INTER_NEAREST
            )
            mosaic[mask > 0] = warped[mask > 0]
            covered = np.maximum(covered, mask)
        observed = int(np.count_nonzero(covered))
        return mosaic, MosaicCoverage(
            anchor_pixel_bounds=[low.tolist(), high.tolist()],
            observed_output_pixels=observed,
            unobserved_output_pixels=int(covered.size - observed),
        )

    def should_add_layer(
        self, layers: list[Layer], anchor: np.ndarray, image: np.ndarray, h: np.ndarray
    ) -> bool:
        """Keep a registered view as a layer only while within budget and if it shows new area."""
        if len(layers) >= Limits.MAX_LAYERS:
            return False
        height, width = image.shape[:2]
        footprint = cv2.perspectiveTransform(
            np.array([[[0, 0]], [[width, 0]], [[width, height]], [[0, height]]], dtype=np.float32), h
        )[:, 0]
        overlap, _ = cv2.intersectConvexConvex(footprint.astype(np.float32), _outline(anchor))
        new_area = abs(cv2.contourArea(footprint.astype(np.float32))) - overlap
        return bool(new_area > Limits.MIN_NEW_COVERAGE_FRACTION * anchor.shape[0] * anchor.shape[1])


def _outline(image: np.ndarray) -> np.ndarray:
    height, width = image.shape[:2]
    return np.array([[0, 0], [width, 0], [width, height], [0, height]], dtype=np.float32)
