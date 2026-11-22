"""Pixel <-> table position, with and without a height above the table."""

import numpy as np
import pytest

from lauzkach_eye_robot_control.projection import pixel_from_table, table_from_pixel

# a perspective view of the 20 x 20 cm square, like the real camera's (rectangle ~ 200 x 220 px)
SRC = np.array([[0, 0], [20, 0], [20, 20], [0, 20]], np.float32)
DST = np.array([[147, 120], [332, 119], [335, 340], [127, 347]], np.float32)
import cv2
HINV = cv2.getPerspectiveTransform(SRC, DST)
M = np.linalg.inv(HINV).tolist()                      # pixel -> table cm


def test_points_on_the_table_round_trip_and_match_the_detector_matrix():
    for x, y in ((0, 0), (20, 20), (7.5, 12.0)):
        px = pixel_from_table(M, x, y)
        assert table_from_pixel(M, *px) == pytest.approx((x, y), abs=1e-6)
    assert pixel_from_table(M, 0, 0) == pytest.approx((147, 120), abs=1e-3)


def test_a_raised_point_is_found_where_it_is_not_where_it_appears():
    lift = np.array([-4.0, -9.0, -0.0012])             # a made-up pinhole lift: points move up the picture as they rise
    for x, y, h in ((5.0, 8.0, 3.0), (15.0, 3.0, 1.0), (10.0, 10.0, 4.0)):
        px = pixel_from_table(M, x, y, h, lift)
        assert table_from_pixel(M, *px, h, lift) == pytest.approx((x, y), abs=1e-6)       # exact inverse with the right height
        flat = table_from_pixel(M, *px, 0.0, None)                                          # ignoring the height is wrong by...
        assert np.hypot(flat[0] - x, flat[1] - y) > 0.05 * h                               # ...a real amount that grows with h


def test_no_lift_means_no_height_correction():
    px = pixel_from_table(M, 6.0, 9.0, 3.0, None)
    assert px == pytest.approx(pixel_from_table(M, 6.0, 9.0))
    assert table_from_pixel(M, *px, 3.0, None) == pytest.approx((6.0, 9.0), abs=1e-6)
