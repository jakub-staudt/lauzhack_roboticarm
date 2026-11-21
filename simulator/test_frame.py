"""Block coordinates and camera projection of the gaze station."""

import pytest

from simulator import frame


def test_block_frame_corners_from_the_robots_point_of_view():
    # Robot faces +x; left is +y. (0, 0) is a block's near-left corner, (20, 20) its far-right corner.
    assert frame.block_to_world("left", 0, 0) == pytest.approx((-0.24, 0.2))
    assert frame.block_to_world("left", 20, 20) == pytest.approx((-0.04, 0.0))
    assert frame.block_to_world("right", 0, 0) == pytest.approx((-0.24, 0.0))
    assert frame.block_to_world("right", 20, 20) == pytest.approx((-0.04, -0.2))


@pytest.mark.parametrize("block", ["left", "right"])
def test_world_block_round_trip(block):
    x, y = frame.block_to_world(block, 7.5, 12.0)
    assert frame.world_to_block(x, y) == (block, pytest.approx(7.5), pytest.approx(12.0))
    assert frame.world_to_block(0.3, 0.3) is None


def test_camera_projection_round_trip_and_orientation():
    for x, y in [(-0.24, 0.2), (-0.04, -0.2), (-0.14, 0.0)]:
        assert frame.pixel_to_world(*frame.world_to_pixel(x, y)) == pytest.approx((x, y))
    # the camera looks straight down: world +x is image right, world +y is image up
    (x0, y0), (x1, y1) = frame.world_to_pixel(-0.2, 0.0), frame.world_to_pixel(-0.1, 0.1)
    assert x1 > x0 and y1 < y0
    # the camera is centred over the two blocks
    assert frame.world_to_pixel(-0.14, 0.0) == pytest.approx(((frame.CAM_SIZE[0] - 1) / 2, (frame.CAM_SIZE[1] - 1) / 2))
