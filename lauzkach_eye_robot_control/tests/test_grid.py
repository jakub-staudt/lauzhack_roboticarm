"""The right-angle grid model: one perspective fit of a rectilinear grid recovers the square and table centimetres."""

import cv2
import numpy as np

from lauzkach_eye_robot_control.grid import SIDE_CM, fit_grid, world_line

# A perspective view of the grid like the real camera's: a trapezoid, converging verticals, 640 x 480 picture.
WORLD = np.array([[0, 0], [60, 0], [60, 20], [0, 20]], np.float32)
IMAGE = np.array([[120, 118], [560, 122], [595, 345], [100, 335]], np.float32)
TRUE = cv2.getPerspectiveTransform(WORLD, IMAGE)


def to_px(points):
    q = TRUE @ np.vstack([np.asarray(points, float).T, np.ones(len(points))])
    return (q[:2] / q[2]).T


def line_pixels(name, right_cm, n=200):
    kind, value = world_line(name, right_cm)
    t = np.linspace(0, 60 if kind == "y" else 20, n)
    pts = np.stack([t, np.full(n, value)], 1) if kind == "y" else np.stack([np.full(n, value), t], 1)
    return to_px(pts) + np.random.default_rng(0).normal(0, 0.15, (n, 2))      # a little pixel noise


def test_square_corners_and_table_cm_are_recovered_from_the_perspective_view():
    pixels = {name: line_pixels(name, 22.0) for name in ("top", "bottom", "left", "mid")}
    init = to_px([[3, 2], [22, 1], [21, 19], [2, 21]])                          # a rough starting guess
    fit = fit_grid(pixels, init, (480, 640))
    assert fit is not None and fit.rms_mm < 0.6
    assert np.abs(fit.corners_px() - to_px([[0, 0], [20, 0], [20, 20], [0, 20]])).max() < 0.5      # corners, in pixels
    probe = np.array([[5.0, 7.0], [12.5, 18.0]])                                # table positions in cm -> pixels -> back
    assert np.abs(fit.to_cm(to_px(probe)) - probe).max() < 0.05
    assert np.abs(fit.to_px(probe) - to_px(probe)).max() < 0.6


def test_a_wrong_layout_gives_a_clearly_worse_fit():
    pixels = {name: line_pixels(name, 22.0) for name in ("top", "bottom", "left", "mid", "right")}
    fit = fit_grid(pixels, to_px([[0, 0], [20, 0], [20, 20], [0, 20]]), (480, 640), right_cm=22.0)
    assert fit.rms_mm < 0.6                                                       # all five fit when the width is right
    wrong = fit_grid(pixels, to_px([[0, 0], [20, 0], [20, 20], [0, 20]]), (480, 640), right_cm=20.0)
    assert wrong.rms_mm > 3 * fit.rms_mm      # a 2 cm mistake in the layout is visible, though it is spread over all the lines


def test_needs_two_lines_each_way():
    pixels = {name: line_pixels(name, 20.0) for name in ("top", "mid", "right")}
    assert fit_grid(pixels, to_px([[0, 0], [20, 0], [20, 20], [0, 20]]), (480, 640)) is None
    assert SIDE_CM == 20.0
