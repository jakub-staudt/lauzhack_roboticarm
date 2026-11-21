"""Python client for the Gazebo station: joint commands/state and camera images over gz-transport.

Needs the Gazebo Python bindings from the ROS Jazzy install (`gz.transport13`); they are importable by the system
python3. With another interpreter, add `/opt/ros/jazzy/opt/gz_transport_vendor/lib/python` (and the gz_msgs_vendor
equivalent) to PYTHONPATH.
"""

from __future__ import annotations

import os
import sys
import tempfile
import threading
import time

import numpy as np

try:
    import google.protobuf  # noqa: F401
    import gz.transport13  # noqa: F401
except ImportError:  # e.g. the robotic_arm venv: borrow the ROS Jazzy Gazebo bindings and the system protobuf
    _shim = os.path.join(tempfile.gettempdir(), "gz_python_shim")
    os.makedirs(_shim, exist_ok=True)
    if not os.path.exists(os.path.join(_shim, "google")):
        os.symlink("/usr/lib/python3/dist-packages/google", os.path.join(_shim, "google"))
    sys.path[:0] = [_shim] + [f"/opt/ros/jazzy/opt/gz_{p}_vendor/lib/python" for p in ("transport", "msgs", "math")]

from gz.msgs10.double_pb2 import Double
from gz.msgs10.image_pb2 import Image
from gz.msgs10.model_pb2 import Model
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.transport13 import Node

MODEL = "so101"
JOINTS = ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper")
# Home pose of the station (station.json), joint order as JOINTS, in radians.
HOME = (-0.7330, -0.4887, 1.0821, 0.3142, 0.0, 0.2725)


class GazeboStation:
    def __init__(self) -> None:
        self.node = Node()
        self._lock = threading.Lock()
        self._joints: dict[str, float] = {}
        self._images: dict[str, Image] = {}
        self._poses: dict[str, tuple[float, float, float]] = {}
        self._pubs = {j: self.node.advertise(f"/{MODEL}/{j}/cmd_pos", Double) for j in JOINTS}
        self.node.subscribe(Model, f"/{MODEL}/joint_states", self._on_joints)
        self.node.subscribe(Pose_V, "/world/gaze_station/pose/info", self._on_poses)  # all poses, resting blocks included
        for cam in ("overhead", "wrist_cam"):
            self.node.subscribe(Image, f"/{cam}/image", lambda msg, cam=cam: self._on_image(cam, msg))
        self._targets: dict[str, float] = {}

    # ------------------------------------------------------------------ callbacks
    def _on_joints(self, msg: Model) -> None:
        state = {j.name: j.axis1.position for j in msg.joint if j.name in JOINTS}
        with self._lock:
            self._joints.update(state)

    def _on_poses(self, msg: Pose_V) -> None:
        with self._lock:
            for pose in msg.pose:
                if pose.name in ("red_block", "blue_block", "yellow_block"):
                    self._poses[pose.name] = (pose.position.x, pose.position.y, pose.position.z)

    def _on_image(self, cam: str, msg: Image) -> None:
        with self._lock:
            self._images[cam] = msg

    # ------------------------------------------------------------------ public API
    def wait_ready(self, timeout: float = 30.0) -> None:
        """Block until the simulator publishes joint states, an overhead frame, and the command topics are connected."""
        t0 = time.time()
        while time.time() - t0 < timeout:
            with self._lock:
                ready = len(self._joints) == len(JOINTS) and "overhead" in self._images
            if ready and all(p.has_connections() for p in self._pubs.values()):
                return
            time.sleep(0.1)
        raise TimeoutError("Gazebo is not publishing; is `gz sim` running with simulator/gazebo/station.sdf?")

    def joints(self) -> dict[str, float]:
        with self._lock:
            return dict(self._joints)

    def command(self, targets: dict[str, float]) -> None:
        """Set joint position targets in radians. The controllers hold the last command."""
        for name, value in targets.items():
            self._pubs[name].publish(Double(data=float(value)))
        self._targets.update(targets)

    def go_home(self) -> None:
        self.command(dict(zip(JOINTS, HOME)))

    def wait_until_reached(self, tol: float = 0.02, timeout: float = 10.0) -> bool:
        t0 = time.time()
        while time.time() - t0 < timeout:
            state = self.joints()
            if all(abs(state[j] - v) < tol for j, v in self._targets.items()):
                return True
            time.sleep(0.05)
        return False

    def block(self, color: str) -> tuple[float, float, float]:
        """World position [m] of the red/blue/yellow block (needs the block to have moved at least once)."""
        with self._lock:
            return self._poses[f"{color}_block"]

    def reset_blocks(self, initial: dict[str, list[float]]) -> None:
        """Put the blocks back at `initial` ({color: [x, y, z]})."""
        import subprocess
        for color, (x, y, z) in initial.items():
            req = f'name: "{color}_block" position {{x: {x} y: {y} z: {z}}} orientation {{w: 1}}'
            subprocess.run(["gz", "service", "-s", "/world/gaze_station/set_pose", "--reqtype", "gz.msgs.Pose",
                            "--reptype", "gz.msgs.Boolean", "--timeout", "2000", "--req", req],
                           capture_output=True, timeout=10)

    def image(self, cam: str = "overhead") -> np.ndarray:
        """Latest RGB frame as an (H, W, 3) uint8 array."""
        with self._lock:
            msg = self._images[cam]
        return np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.width, 3).copy()
