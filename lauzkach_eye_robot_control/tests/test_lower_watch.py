import cv2
import numpy as np

from lauzkach_eye_robot_control.lower_watch import LowerWatcher, bottom_edge_reached


def picture(bottom_y: int, colour=(220, 60, 20)) -> np.ndarray:
    img = np.full((480, 640, 3), 225, np.uint8)
    cv2.rectangle(img, (200, bottom_y - 200), (440, bottom_y), colour, -1)            # a blue block whose lower edge is at bottom_y
    return img


def test_the_blocks_lower_edge_reaching_the_bottom_of_the_picture_is_detected():
    far = bottom_edge_reached(picture(300), "blue")
    near = bottom_edge_reached(picture(476), "blue")
    over = bottom_edge_reached(picture(479), "blue")
    assert far["seen"] and not far["reached"] and far["bottom_px"] == 301
    assert near["reached"] and over["reached"]
    assert not bottom_edge_reached(picture(476), "red")["seen"]                       # another colour is not the selected block
    assert not bottom_edge_reached(np.full((480, 640, 3), 225, np.uint8), "blue")["seen"]


def test_the_watcher_tells_the_arm_to_stop_after_two_frames_with_the_edge_at_the_bottom():
    frames = iter([picture(300), picture(380), picture(440), picture(478), picture(479), picture(479)])
    stops, notes = [], []
    w = LowerWatcher("blue", lambda: next(frames), lambda: True, lambda: stops.append(1), notes.append, period_s=0.0).start()
    w.join()
    assert stops == [1] and "lowered enough" in notes[0] and w.status["state"].startswith("edge reached")


def test_the_watcher_ends_quietly_when_the_arm_finished_without_the_edge_being_seen():
    running = iter([True, True, False])
    stops = []
    w = LowerWatcher("blue", lambda: picture(300), lambda: next(running), lambda: stops.append(1), period_s=0.0).start()
    w.join()
    assert stops == [] and w.status["state"].startswith("finished without seeing")
