"""Read-only measured-pose view using the twin's model and rendering components.

This module never constructs TwinApp, a backend, or a simulation loop. Its only
input is the panel controller's cached snapshot; camera orbit is the only 3D
interaction. Missing optional dependencies leave the control panel usable.
"""
from __future__ import annotations

import math
import os
import threading
from collections.abc import Callable, Mapping
from dataclasses import replace
from pathlib import Path
from typing import Any

from .panel_environment import add_environment, environment_spec, environment_status

STALE_AFTER_S = 0.75


def measured_pose(snapshot: Mapping[str, Any], joints: tuple[str, ...]) -> tuple[dict[str, float], bool]:
    """Validate a complete cached measurement, without substituting targets/home.

    Invalid values raise; stale but complete measurements may still be displayed,
    clearly marked as the last measurement rather than a live pose.
    """
    measured = snapshot.get("measured")
    if not isinstance(measured, Mapping):
        raise ValueError("No measured pose")
    pose = {}
    for joint in joints:
        value = measured.get(joint)
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
            raise ValueError(f"No finite measurement for {joint}")
        pose[joint] = float(value)
    sample = snapshot.get("sample", snapshot)
    age = sample.get("age_s")
    fresh = (isinstance(age, (int, float)) and not isinstance(age, bool) and math.isfinite(age)
             and 0 <= age <= STALE_AFTER_S and sample.get("connected") is True
             and sample.get("loop_ok") is True)
    return pose, not fresh


class PoseViewer:
    """A separate Viser server observing cached joint measurements at 10 Hz.

    `gripper_range_q` optionally overrides the robot description with the real
    backend's calibrated (closed, open) MJCF range. start() never raises for an
    unavailable renderer; inspect status() for the error. Selection is visual only.
    """

    def __init__(self, robot_dir: Path, state: Callable[[], dict], host: str,
                 port: int = 8766, gripper_range_q: tuple[float, float] | None = None,
                 environment: str | None = None):
        self.robot_dir, self._state, self.host, self.port = Path(robot_dir), state, host, port
        self._gripper_range_q = gripper_range_q
        self._environment = environment_spec(
            os.environ.get("RAMION_WORKSPACE_LAYOUT", "") if environment is None else environment)
        self._environment_handles: list[Any] = []
        self._lock = threading.RLock()
        self._halt = threading.Event()
        self._thread: threading.Thread | None = None
        self._server = None
        self._kin = self._mirror = self._root = self._marker = self._label = None
        self._joints: tuple[str, ...] = ()
        self._selected: str | None = None
        self._available = False
        self._error: str | None = None
        self._stale = True
        self._has_pose = False
        self._feed = "No measured pose"

    def start(self) -> None:
        with self._lock:
            if self._available:
                return
            try:
                self._initialize()
                self._halt.clear()
                self._thread = threading.Thread(target=self._run, name="panel-pose-viewer", daemon=True)
                self._available = True
                self._error = None
                self._thread.start()
            except Exception as exc:
                self._error = f"3D viewer unavailable: {exc}"
                self._available = False
                if self._server is not None:
                    self._server.stop()
                    self._server = None

    def _initialize(self) -> None:
        import mujoco
        import numpy as np
        import viser
        from machinekind_twin.kinematics import RobotKinematics
        from machinekind_twin.robots import load_robot
        from machinekind_twin.ui.bridge import SceneMirror, mat_to_wxyz

        spec = load_robot(self.robot_dir)
        if self._gripper_range_q is not None:
            pair = tuple(float(v) for v in self._gripper_range_q)
            if len(pair) != 2 or not all(math.isfinite(v) for v in pair) or pair[0] == pair[1]:
                raise ValueError("gripper_range_q must contain two distinct finite values")
            spec = replace(spec, gripper_range_q=pair)
        self._kin = RobotKinematics(spec)
        self._joints = tuple(spec.joints)
        self._mujoco, self._mat_to_wxyz = mujoco, mat_to_wxyz
        self._server = viser.ViserServer(host=self.host, port=self.port,
                                         label="Measured pose | read only", verbose=False)
        # Viser may choose another port if the requested one is occupied.
        self.port = self._server.get_port()
        self._server.gui.configure_theme(control_layout="collapsible", control_width="small",
                                         dark_mode=True, show_logo=False, show_share_button=False)
        self._server.scene.set_up_direction("+z")
        self._server.scene.set_background_image(np.full((2, 2, 3), (19, 24, 27), dtype=np.uint8))
        self._server.scene.add_grid("/ground", width=1.0, height=1.0, cell_size=0.05,
                                    section_size=0.25, cell_color=(47, 58, 61),
                                    section_color=(68, 83, 85), plane_color=(23, 29, 31),
                                    plane_opacity=0.6, position=(0.0, 0.0, self._environment["table_z_m"]
                                              if self._environment else -0.004))
        self._server.initial_camera.position = (0.62, -0.54, 0.44)
        self._server.initial_camera.look_at = (0.12, 0.0, 0.12)
        if self._environment is not None:
            self._environment_handles = add_environment(self._server.scene, self._environment)
            self._server.initial_camera.position = (0.78, -0.68, 0.58)
            self._server.initial_camera.look_at = (0.12, -0.025, 0.22)
        self._root = self._server.scene.add_frame("/measured", show_axes=False, visible=False)
        self._mirror = SceneMirror(self._server, self._kin.model, root="/measured/robot")
        self._marker = self._server.scene.add_frame("/selected_joint", axes_length=0.075,
                                                    axes_radius=0.003, visible=False)
        self._label = self._server.scene.add_label("/measurement_status", "WAITING | no measured pose",
                                                   position=(0.0, 0.0, 0.48), font_screen_scale=0.65)
        # With no generated GUI, Viser opens its large server/developer settings
        # as the default panel body. Provide useful camera help instead.
        self._server.gui.add_markdown("**Read-only measured pose**\n\n"
                                      "Drag to orbit. Scroll to zoom. Select a joint in the workbench.")
        # No joint sliders, transform controls, scene click handlers, or motion callbacks.
        self._minimize_gui(self._server.gui)
        # Reissue per client after its first camera message: this is later than
        # initial scene replay and uses a fresh client-scoped layout command.
        self._server.on_client_connect(self._configure_client)

    def _configure_client(self, client: Any) -> None:
        self._minimize_gui(client.gui)

    @staticmethod
    def _minimize_gui(gui: Any) -> None:
        panel = getattr(gui, "main_panel", None)
        # Docking gives the canvas its own viewport even if a client fails to
        # apply collapse; the panel then cannot float over the measured robot.
        if panel is not None and callable(getattr(panel, "dock_right", None)):
            panel.dock_right()
        if panel is not None and callable(getattr(panel, "hide", None)):
            panel.hide()
        elif panel is not None and callable(getattr(panel, "minimize", None)):
            panel.minimize()

    def status(self) -> dict[str, Any]:
        with self._lock:
            return {"available": self._available, "port": self.port, "error": self._error,
                    "selected_joint": self._selected, "stale": self._stale, "feed": self._feed,
                    "environment": environment_status(self._environment)}

    def select_joint(self, name: str) -> None:
        with self._lock:
            # Validation also works when optional rendering dependencies are absent.
            if not self._joints:
                import yaml
                description = yaml.safe_load((self.robot_dir / "robot.yaml").read_text(encoding="utf-8"))
                self._joints = tuple(description["joints"])
            if name not in self._joints:
                raise ValueError(f"Unknown joint: {name}")
            self._selected = name
            self._highlight()

    def _highlight(self) -> None:
        if self._marker is None or not self._has_pose or self._selected is None:
            return
        model, data = self._kin.model, self._kin.data
        joint = model.joint(self._selected).id
        body = int(model.jnt_bodyid[joint])
        self._marker.position = data.xanchor[joint].copy()
        self._marker.wxyz = self._mat_to_wxyz(data.xmat[body].reshape(3, 3))
        self._marker.visible = True

    def _update(self) -> None:
        """One observer tick. Never reads hardware or writes the source snapshot."""
        snapshot = self._state()
        with self._lock:
            pose, stale = measured_pose(snapshot, self._joints)
            kin = self._kin
            kin.write_qpos(kin.data.qpos, kin.to_q(pose))
            self._mujoco.mj_forward(kin.model, kin.data)
            self._mirror.update(kin.data)
            self._has_pose = True
            self._root.visible = True
            self._stale = stale
            self._feed = "STALE | last measured pose" if stale else "LIVE | measured pose | read only"
            self._label.text = "STALE | last measured pose" if stale else ""
            self._label.visible = stale
            self._highlight()

    def _run(self) -> None:
        while not self._halt.is_set():
            try:
                self._update()
            except Exception as exc:
                with self._lock:
                    self._stale = True
                    self._feed = f"STALE | measurement unavailable: {exc}"
                    if self._label is not None:
                        self._label.text = "STALE | measurement unavailable"
                        self._label.visible = True
            self._halt.wait(0.1)

    def close(self) -> None:
        self._halt.set()
        if self._thread is not None and self._thread is not threading.current_thread():
            self._thread.join(timeout=2.0)
        with self._lock:
            self._available = False
            if self._server is not None:
                self._server.stop()
                self._server = None
