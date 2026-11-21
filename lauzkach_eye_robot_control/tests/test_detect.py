"""The rectangle detector, on a synthetic table with known corners and on real frames from the overhead camera."""

from pathlib import Path

import cv2
import numpy as np
import pytest

from lauzkach_eye_robot_control.detect import detect, rectify

FIXTURES = Path(__file__).parent / "fixtures"
CORNERS = [(120, 120), (335, 120), (335, 340), (120, 340)]  # TL, TR, BR, BL of the left rectangle in the synthetic picture


def synthetic_table(hide_left: bool) -> np.ndarray:
    """White 640x480 picture: four AprilTags 36h11, two horizontal and three vertical thin lines. `hide_left` puts a
    white-and-grey block (like the arm) over the left line and the left tags."""
    img = np.full((480, 640, 3), 235, np.uint8)
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
    for tag_id, (cx, cy) in zip((40, 41, 42, 43), ((50, 85), (590, 385), (50, 385), (590, 85)), strict=True):
        marker = cv2.cvtColor(cv2.aruco.generateImageMarker(dictionary, tag_id, 64), cv2.COLOR_GRAY2BGR)
        img[cy - 32:cy + 32, cx - 32:cx + 32] = marker
    for y in (120, 340):
        cv2.line(img, (100, y), (560, y), (70, 60, 60), 1)
    for x in (120, 335, 550):
        cv2.line(img, (x, 100), (x, 360), (70, 60, 60), 1)
    if hide_left:
        cv2.rectangle(img, (0, 0), (200, 480), (200, 200, 205), -1)      # the arm covers everything left of x = 200
        cv2.rectangle(img, (40, 200), (180, 300), (40, 40, 40), -1)      # with a dark motor on it
    return cv2.GaussianBlur(img, (3, 3), 0)


def test_all_lines_visible_finds_the_exact_rectangle():
    det = detect(synthetic_table(hide_left=False))
    assert {t["id"] for t in det["tags"]} == {40, 41, 42, 43}
    assert det["ok"] and "detected" in det["method"]
    assert np.abs(np.array(det["polygon"]) - np.array(CORNERS)).max() < 3


def test_hidden_left_edge_is_estimated_and_reported():
    det = detect(synthetic_table(hide_left=True), right_cm=20.0)   # the synthetic table has two equal squares
    assert {t["id"] for t in det["tags"]} == {41, 43}          # only the right tags remain
    assert det["ok"] and "estimated" in det["method"]
    assert any("estimated" in issue for issue in det["issues"])  # the page must warn that the left edge is a guess
    assert np.abs(np.array(det["polygon"]) - np.array(CORNERS)).max() < 6


def test_without_tags_there_is_no_rectangle_and_a_reason():
    det = detect(np.full((480, 640, 3), 235, np.uint8))
    assert det["polygon"] is None and not det["ok"]
    assert det["issues"] and "tag" in det["issues"][0]


def test_dark_cable_across_the_table_is_not_taken_for_a_line():
    img = synthetic_table(hide_left=False)
    pts = np.array([[230, 150], [250, 220], [215, 290], [245, 340]], np.int32).reshape(-1, 1, 2)
    cv2.polylines(img, [pts], False, (20, 20, 20), 8)             # a thick cable, nearly vertical, between the left and mid lines
    det = detect(img)
    assert det["ok"] and np.abs(np.array(det["polygon"]) - np.array(CORNERS)).max() < 3


@pytest.mark.parametrize("name", ["overhead_1.jpg", "overhead_2.jpg", "overhead_3.jpg"])
def test_real_camera_frames(name):
    """Real frames: the arm hides the left tags and the left line, so the left edge must be flagged as estimated."""
    det = detect(cv2.imread(str(FIXTURES / name)))
    assert {t["id"] for t in det["tags"]} == {41, 43}
    assert det["ok"] and "estimated" in det["method"]
    tl, tr, br, bl = np.array(det["polygon"])
    assert np.abs(tr - (332, 119)).max() < 5 and np.abs(br - (335, 340)).max() < 6   # the corners that come from visible lines
    assert 0.85 < det["aspect"] < 1.1
    assert rectify(cv2.imread(str(FIXTURES / name)), det["polygon"]).shape == (360, 360, 3)


def test_real_frame_with_the_arm_moved_away_uses_the_printed_left_line():
    """With the arm out of the way the dotted printed left line is visible; it is where the eye sees it (zoomed in by hand:
    about x 160 at y 70, 137 at y 235, 128 at y 335), and it is 1-2 cm from the rough estimate."""
    img = cv2.imread(str(FIXTURES / "overhead_arm_away.jpg"))
    det = detect(img)
    assert {t["id"] for t in det["tags"]} >= {41, 42, 43}
    assert det["ok"] and "left line detected" in det["method"]
    tl, tr, br, bl = np.array(det["polygon"])
    assert np.abs(tl - (146, 120)).max() < 8 and np.abs(bl - (128, 348)).max() < 10
    assert 20.5 < det["grid"]["right_width_cm"] < 22.5                   # the right square is not 20 cm wide
    assert det["grid"]["rms_mm"] < 2.0                                    # the four lines fit one right-angled grid to ~1 mm
    assert np.abs(np.array(det["left_estimate"][0]) - tl).max() < 14      # and the grid's prediction (right square 21.1 cm) lands near it
