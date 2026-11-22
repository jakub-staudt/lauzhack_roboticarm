"""Tip observers on synthetic pictures where the true position is known."""

import cv2
import numpy as np
import pytest

from lauzkach_eye_robot_control.tip import ColorMarkerObserver, DarkBlobObserver, TagMarkerObserver, observe_first


def board() -> np.ndarray:
    return np.full((480, 640, 3), 190, np.uint8)


def test_colour_marker_is_found_to_a_fraction_of_a_pixel_and_the_hint_picks_the_right_one():
    img = board()
    cv2.circle(img, (200, 150), 6, (255, 0, 255), -1)           # the sticker on the gripper (BGR magenta)
    cv2.circle(img, (450, 300), 8, (255, 0, 255), -1)           # something else with the same colour
    observer = ColorMarkerObserver()
    near = observer.observe(img, hint=(205, 148))
    assert near is not None and np.hypot(near.px[0] - 200, near.px[1] - 150) < 0.6 and near.confidence > 0.7
    far = observer.observe(img, hint=(450, 300))
    assert np.hypot(far.px[0] - 450, far.px[1] - 300) < 0.6                     # the hint decides which one
    assert observer.observe(img, hint=(50, 400)) is None                          # nothing near that hint
    assert observer.observe(board()) is None                                      # no sticker at all


def test_colour_marker_ignores_specks_and_huge_areas():
    img = board()
    img[100:102, 100:102] = (255, 0, 255)                                         # a 2 x 2 speck of noise
    cv2.rectangle(img, (300, 200), (500, 400), (255, 0, 255), -1)                 # a poster-sized patch of the colour
    assert ColorMarkerObserver().observe(img) is None


def test_tag_marker_gives_the_tag_centre_plus_the_tip_offset():
    img = board()
    marker = cv2.cvtColor(cv2.aruco.generateImageMarker(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11), 7, 48), cv2.COLOR_GRAY2BGR)
    img[226:274, 316:364] = marker                                                # centre at (340, 250)
    seen = TagMarkerObserver(7, offset_px=(0, 20)).observe(img)
    assert seen is not None and np.hypot(seen.px[0] - 340, seen.px[1] - 270) < 1.0
    assert TagMarkerObserver(8).observe(img) is None                              # a different tag is not ours


def test_dark_blob_needs_a_hint_and_prefers_compact_blobs_over_cables():
    img = board()
    cv2.rectangle(img, (95, 85), (125, 115), (20, 20, 20), -1)                    # the wrist camera board: a dark 30 x 30 square
    cv2.polylines(img, [np.array([[150, 60], [170, 120], [140, 200], [180, 260]], np.int32).reshape(-1, 1, 2)], False, (15, 15, 15), 5)   # a cable
    observer = DarkBlobObserver()
    assert observer.observe(img) is None                                          # no hint: refuses
    seen = observer.observe(img, hint=(112, 105))
    assert seen is not None and np.hypot(seen.px[0] - 110, seen.px[1] - 100) < 1.0 and seen.method == "dark blob"
    assert observer.observe(img, hint=(160, 150)) is None                         # near the cable only: thin, so not accepted
    assert observer.observe(img, hint=(400, 300)) is None                         # nothing dark nearby


def test_the_first_observer_that_sees_something_wins():
    img = board()
    cv2.rectangle(img, (95, 85), (125, 115), (20, 20, 20), -1)
    observers = [ColorMarkerObserver(), DarkBlobObserver()]
    assert observe_first(observers, img, hint=(110, 100)).method == "dark blob"    # no sticker: falls back to the dark part
    cv2.circle(img, (110, 100), 5, (255, 0, 255), -1)
    assert observe_first(observers, img, hint=(110, 100)).method == "colour marker"


def test_a_dark_blob_alone_is_never_trusted_enough_to_drive_the_loop():
    from lauzkach_eye_robot_control.servo import ServoConfig
    img = board()
    cv2.rectangle(img, (95, 85), (125, 115), (20, 20, 20), -1)
    seen = DarkBlobObserver().observe(img, hint=(110, 100))          # a perfect, centred hit
    assert seen is not None and seen.confidence < ServoConfig().min_confidence     # on the real camera such hits were cables
