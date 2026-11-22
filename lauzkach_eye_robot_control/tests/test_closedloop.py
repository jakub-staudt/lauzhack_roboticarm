"""The adapters that connect the closed loop to the simulator and to the camera."""

import cv2
import numpy as np
import pytest

from lauzkach_eye_robot_control.closedloop import CameraTipObserver, SimMover, SimTruthObserver
from lauzkach_eye_robot_control.drive import SimError
from lauzkach_eye_robot_control.feature_model import FeatureModel
from lauzkach_eye_robot_control.robot_map import RobotMap
from lauzkach_eye_robot_control.servo import ServoConfig, run
from lauzkach_eye_robot_control.tip import ColorMarkerObserver
from tests.test_robot_map import RECORDED

SRC = np.array([[0, 0], [20, 0], [20, 20], [0, 20]], np.float32)
DST = np.array([[147, 120], [332, 119], [335, 340], [127, 347]], np.float32)
M = np.linalg.inv(cv2.getPerspectiveTransform(SRC, DST)).tolist()
RMAP = RobotMap(RECORDED)


class FakeSim:
    """A simulated arm: moves take two polls to finish."""

    def __init__(self):
        self.tip = {"x": 0.2, "y": 0.0, "z": 0.03}
        self.state_name, self.polls, self.target = "idle", 0, None
        self.moves = []

    def go_to_base(self, x, y, height_cm):
        self.moves.append((x, y, height_cm))
        self.target, self.state_name, self.polls = (x, y), "running", 0
        return {"state": "planning"}

    def state(self):
        if self.state_name == "running":
            self.polls += 1
            if self.polls >= 2:
                self.state_name = "done"
                self.tip = {"x": self.target[0], "y": self.target[1], "z": 0.03}
        return {"task": {"state": self.state_name}, "tip_base": self.tip}


def test_the_loop_corrects_a_calibration_error_on_the_simulated_arm():
    sim = FakeSim()
    error = (0.012, -0.008)                                         # the board is 14 mm from where the arm thinks
    target = (0.30, -0.05)
    result = run(target, 3.0, SimMover(sim, poll_s=0.001), SimTruthObserver(sim, error), ServoConfig(settle_s=0.0), sleep=lambda s: None)
    seen = (sim.tip["x"] - error[0], sim.tip["y"] - error[1])       # where the tip is, as the board sees it
    assert result.ok and np.hypot(seen[0] - target[0], seen[1] - target[1]) < 0.003
    assert len(sim.moves) <= 3 and sim.moves[0][:2] == pytest.approx(target)


def test_a_stopped_or_failed_move_ends_the_wait():
    class Stopping(FakeSim):
        def state(self):
            return {"task": {"state": "stopped"}, "tip_base": self.tip}
    assert SimMover(Stopping(), poll_s=0.001).wait_idle(1.0) is False


def test_an_unreachable_simulator_is_not_a_crash():
    class Down:
        def state(self):
            raise SimError("down", 503)
    assert SimMover(Down(), poll_s=0.001).wait_idle(0.2) is False
    assert SimTruthObserver(Down()).observe_xy() is None


def synthetic_scene(model: FeatureModel, tip, rotation, color_px=None):
    img = np.full((480, 640, 3), 190, np.uint8)
    px = model.predict_pixel(tip, rotation) if color_px is None else color_px
    cv2.circle(img, (round(px[0]), round(px[1])), 6, (255, 0, 255), -1)       # a sticker on the gripper
    return img, px


def test_the_camera_observer_finds_the_tip_from_a_picture_and_the_arms_pose():
    model = FeatureModel(M, "up", RMAP, lift=np.array([-3.5, -8.0, -0.0015]), bias=np.array([0.006, -0.004]))
    tip, rotation = (0.30, -0.06, 0.03), np.eye(3)
    img, _ = synthetic_scene(model, tip, rotation)
    observer = CameraTipObserver(lambda: img, lambda: {"tip": tip, "rotation": rotation}, [ColorMarkerObserver()], model)
    x, y, confidence = observer.observe_xy()
    assert np.hypot(x - tip[0], y - tip[1]) < 0.0015 and confidence > 0.6      # within 1.5 mm (a pixel is ~1 mm here)


def test_a_marker_seen_far_from_where_the_arm_believes_is_refused():
    model = FeatureModel(M, "up", RMAP)
    tip, rotation = (0.30, -0.06, 0.03), np.eye(3)
    hint = model.predict_pixel(tip, rotation)
    img, _ = synthetic_scene(model, tip, rotation, color_px=(hint[0] + 60, hint[1] - 20))   # something magenta, 6 cm away
    observer = CameraTipObserver(lambda: img, lambda: {"tip": tip, "rotation": rotation}, [ColorMarkerObserver(radius=120)], model)
    assert observer.observe_xy() is None                                       # implausible: refused, not chased


def test_no_picture_or_no_pose_means_no_observation():
    model = FeatureModel(M, "up", RMAP)
    assert CameraTipObserver(lambda: None, lambda: {"tip": (0, 0, 0), "rotation": np.eye(3)}, [], model).observe_xy() is None
    assert CameraTipObserver(lambda: np.zeros((10, 10, 3), np.uint8), lambda: None, [], model).observe_xy() is None
