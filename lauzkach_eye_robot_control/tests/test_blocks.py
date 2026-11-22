import cv2
import numpy as np
import pytest
from pathlib import Path

from lauzkach_eye_robot_control.blocks import detect_blocks
from lauzkach_eye_robot_control.detect import detect
from lauzkach_eye_robot_control.grid import WORLD_CORNERS

FIX = Path(__file__).parent / "fixtures"


def matrix_of(name):
    img = cv2.imread(str(FIX / f"{name}.jpg"))
    poly = detect(img)["polygon"]
    return img, cv2.getPerspectiveTransform(np.array(poly, np.float32), WORLD_CORNERS).tolist()


def test_the_four_blocks_in_the_arm_away_picture_are_found_with_their_colours_and_places():
    img, m = matrix_of("overhead_arm_away")
    found = {b["colour"]: b for b in detect_blocks(img, m)}
    assert set(found) == {"red", "blue", "yellow"}                                   # the wooden block is not used
    for colour, (x, y) in {"blue": (3.8, 7.8), "yellow": (2.8, 12.7), "red": (12.4, 15.3)}.items():
        assert found[colour]["cm"] == pytest.approx([x, y], abs=1.5), colour
    assert all(0.0 <= b["yaw_deg"] < 90.0 for b in found.values())
    assert found["red"]["yaw_deg"] == pytest.approx(18.0, abs=12.0)               # the red block is turned about 20 degrees in the picture


def test_pictures_with_the_arm_over_the_table_give_only_a_few_known_colours_and_no_false_blocks():
    for name in ("overhead_1", "overhead_2", "overhead_3"):
        img, m = matrix_of(name)
        colours = [b["colour"] for b in detect_blocks(img, m)]
        assert set(colours) <= {"red", "blue", "yellow"} and len(colours) == len(set(colours)), (name, colours)


def test_thin_lines_and_spots_outside_the_rectangle_are_not_blocks():
    img = np.full((480, 640, 3), 235, np.uint8)
    cv2.rectangle(img, (300, 200), (340, 240), (0, 0, 220), -1)                       # a red square (a block)
    cv2.line(img, (50, 400), (400, 405), (0, 0, 220), 5)                             # a long red cable
    cv2.rectangle(img, (600, 20), (630, 50), (220, 60, 0), -1)                       # a blue square, but outside the rectangle
    m = [[0.03, 0, -5.0], [0, 0.03, -4.0], [0, 0, 1]]                                  # 1 px = 0.03 cm: the rectangle is x, y in 0..20 cm
    found = detect_blocks(img, m, side_cm=20.0, margin_cm=0.0)
    assert [(b["colour"]) for b in found] == ["red"]
