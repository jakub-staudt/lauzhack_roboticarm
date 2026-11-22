import math

import pytest

from simulator import alignment, frame

POINTS = [(0.0, 0.0, 0.1997, 0.0384), (1.0, 0.0, 0.1967, -0.1394), (1.0, 1.0, 0.3923, -0.1427), (0.0, 1.0, 0.3953, 0.0351)]


@pytest.fixture(autouse=True)
def restore_frame():
    saved = dict(frame.LEFT), frame.RIGHT_ENABLED
    yield
    frame.LEFT.update(saved[0])
    frame.RIGHT_ENABLED = saved[1]


def test_the_fit_is_an_orthogonal_rectangle_through_the_recorded_corners():
    fit = alignment.fit(POINTS)
    assert fit["origin"] == pytest.approx((0.1997, 0.0384), abs=0.003)               # (0, 0) lands on the real corner_00
    assert 0.17 < fit["su_m"] < 0.19 and 0.19 < fit["sv_m"] < 0.21 and fit["residual_mm"] < 12
    assert abs(math.degrees(fit["theta"])) < 3


def test_a_configured_block_keeps_its_corners_orthogonal_and_round_trips():
    frame.configure_left((-0.15, 0.04), math.radians(-1.0), 0.178, 0.196)
    (x0, y0), (x1, y1), (x2, y2), (x3, y3) = frame.left_corners_world()
    assert (x0, y0) == pytest.approx((-0.15, 0.04))
    e_u, e_v = (x1 - x0, y1 - y0), (x3 - x0, y3 - y0)
    assert e_u[0] * e_v[0] + e_u[1] * e_v[1] == pytest.approx(0, abs=1e-12)           # the sides stay at right angles
    assert math.hypot(*e_u) == pytest.approx(0.178) and math.hypot(*e_v) == pytest.approx(0.196)
    assert (x2, y2) == pytest.approx((x1 + e_v[0], y1 + e_v[1]))                       # a true rectangle, no shear
    for cx, cy in ((0, 0), (20, 0), (7.5, 13.2), (20, 20)):
        name, bx, by = frame.world_to_block(*frame.block_to_world("left", cx, cy), margin_cm=1e-6)
        assert (name, bx, by) == ("left", pytest.approx(cx), pytest.approx(cy))
    assert frame.world_to_block(0.5, 0.5) is None
