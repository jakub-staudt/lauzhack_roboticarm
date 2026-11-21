"""Registration and mosaic: the moved algorithm must give the legacy collector's results exactly."""

import pytest

cv2 = pytest.importorskip('cv2', reason='collector tests need the scan extra')
import numpy as np  # noqa: E402  (installed with OpenCV)

import legacy  # noqa: E402
from arm_dashboard.scan.worker.exceptions import MosaicBudgetExceededError  # noqa: E402
from arm_dashboard.scan.worker.mosaic import MosaicRenderer  # noqa: E402
from arm_dashboard.scan.worker.registration import PlaneRegistrationProcessor  # noqa: E402
from arm_dashboard.scan.worker.schemas import RegistrationReason  # noqa: E402

WIDTH, HEIGHT = 640, 480


def texture(seed: int) -> np.ndarray:
    """The legacy test's synthetic board: random discs on grey."""
    rng = np.random.default_rng(seed)
    image = np.full((HEIGHT, WIDTH, 3), 210, np.uint8)
    for _ in range(300):
        x, y = rng.integers([10, 10], [WIDTH - 10, HEIGHT - 10])
        colour = tuple(int(c) for c in rng.integers(0, 180, 3))
        cv2.circle(image, (int(x), int(y)), int(rng.integers(2, 10)), colour, -1)
    return image


def warped(image: np.ndarray, h: list[list[float]]) -> np.ndarray:
    return cv2.warpPerspective(image, np.array(h, dtype=np.float64), (WIDTH, HEIGHT))


def localized(image: np.ndarray) -> np.ndarray:
    """Only a small textured corner survives: matches exist but cover too little of the frame."""
    blank = np.full_like(image, 210)
    blank[:60, :80] = image[:60, :80]
    return blank


FORWARD = [[0.9, 0.02, 15], [-0.01, 0.92, 12], [0.00002, 0.00001, 1]]
PAIRS = {
    'accepted-perspective': lambda: (texture(7), warped(texture(7), FORWARD)),
    'accepted-shift': lambda: (texture(5), warped(texture(5), [[1, 0, 30], [0, 1, -20], [0, 0, 1]])),
    'blank-reference': lambda: (np.zeros((HEIGHT, WIDTH, 3), np.uint8), texture(3)),
    'unrelated-boards': lambda: (texture(1), texture(2)),
    'localized-matches': lambda: (texture(9), localized(texture(9))),
}


@pytest.fixture(scope='module')
def legacy_worker():
    return legacy.load('scan_worker')


@pytest.mark.parametrize('pair', list(PAIRS))
def test_registration_is_identical_to_legacy(legacy_worker, pair: str) -> None:
    reference, candidate = PAIRS[pair]()

    legacy_h, legacy_result = legacy_worker.register_plane(reference, candidate)
    result = PlaneRegistrationProcessor().register(reference, candidate)

    assert result.model_dump(mode='json', exclude_unset=True) == legacy_result
    if legacy_h is None:
        assert result.homography is None
    else:
        assert result.homography is not None
        assert np.array_equal(result.homography, legacy_h)


def test_pairs_cover_accepting_and_rejecting_branches(legacy_worker) -> None:
    reasons = {PlaneRegistrationProcessor().register(*make()).reason for make in PAIRS.values()}

    assert RegistrationReason.ACCEPTED in reasons
    assert RegistrationReason.TOO_FEW_FEATURES in reasons
    assert len(reasons) >= 3


def test_known_transform_is_recovered() -> None:
    reference = texture(7)

    result = PlaneRegistrationProcessor().register(reference, warped(reference, FORWARD))

    assert result.accepted, result
    points = np.array([[[100, 100]], [[300, 200]], [[500, 300]]], dtype=np.float32)
    round_trip = cv2.perspectiveTransform(
        cv2.perspectiveTransform(points, np.array(FORWARD)), result.homography
    )
    assert float(np.max(np.linalg.norm(round_trip - points, axis=2))) < 1.0


def mosaic_cases() -> dict[str, tuple[np.ndarray, list[tuple[np.ndarray, np.ndarray]]]]:
    anchor = texture(4)
    return {
        'anchor-only': (anchor, []),
        'two-layers': (
            anchor,
            [
                (texture(6), np.array([[1.0, 0, 200], [0, 1.0, 50], [0, 0, 1]])),
                (texture(8), np.array([[0.9, 0.05, -120], [0.02, 0.95, 300], [0.0001, 0, 1]])),
            ],
        ),
    }


@pytest.mark.parametrize('case', list(mosaic_cases()))
def test_mosaic_is_identical_to_legacy(legacy_worker, case: str) -> None:
    anchor, layers = mosaic_cases()[case]

    legacy_image, legacy_coverage = legacy_worker.render_mosaic(anchor, layers)
    image, coverage = MosaicRenderer().render(anchor, layers)

    assert np.array_equal(image, legacy_image)
    assert coverage.model_dump() == legacy_coverage


def test_mosaic_over_budget_is_refused_like_legacy(legacy_worker) -> None:
    anchor = texture(4)
    far = [(texture(6), np.array([[1.0, 0, 5000], [0, 1.0, 0], [0, 0, 1]]))]

    with pytest.raises(ValueError, match='budget'):
        legacy_worker.render_mosaic(anchor, far)
    with pytest.raises(MosaicBudgetExceededError, match='budget'):
        MosaicRenderer().render(anchor, far)


def test_mosaic_keeps_unobserved_holes_black() -> None:
    anchor = np.full((80, 80, 3), 80, np.uint8)
    other = np.full((80, 80, 3), 140, np.uint8)

    image, coverage = MosaicRenderer().render(
        anchor, [(other, np.array([[1.0, 0, 160], [0, 1.0, 0], [0, 0, 1]]))]
    )

    assert image.shape == (80, 240, 3)
    assert (coverage.observed_output_pixels, coverage.unobserved_output_pixels) == (12800, 6400)
    assert np.all(image[:, 80:160] == 0)


def test_layer_needs_new_area_and_budget() -> None:
    renderer = MosaicRenderer()
    anchor = texture(4)
    identity = np.eye(3)
    beside = np.array([[1.0, 0, WIDTH], [0, 1.0, 0], [0, 0, 1]])

    assert not renderer.should_add_layer([], anchor, anchor, identity)
    assert renderer.should_add_layer([], anchor, anchor, beside)
    assert not renderer.should_add_layer([(anchor, beside)] * 16, anchor, anchor, beside)
