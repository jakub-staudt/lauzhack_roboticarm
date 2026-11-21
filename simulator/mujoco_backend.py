"""MuJoCo implementation of the station interface used by the web UI (same methods as `gazebo.client.GazeboStation`).

It needs only `pip install mujoco numpy`, so it runs on Linux, macOS (Apple Silicon included) and Windows. The physics and
the camera rendering live in one object. Rendering uses OpenGL, which on macOS must happen on the main thread: call
`run()` from the main thread and serve HTTP from another thread.
"""

from __future__ import annotations

import threading
import time

import mujoco
import numpy as np

from simulator.run import build_model, load_station
from soarm_lab.kinematics import RobotKinematics
from soarm_lab.station import PREFIX

JOINTS = ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper")
COLORS = ("red", "blue", "yellow")


class MujocoStation:
    def __init__(self, render: bool = True, fps: float = 15.0) -> None:
        station = load_station()
        self.model = build_model(station)
        # The stock scene's lights wash out the white board seen from straight above; dim them for the camera.
        self.model.light_diffuse[:] *= 0.5
        self.model.light_specular[:] *= 0.3
        self.model.vis.headlight.ambient[:] = 0.15
        self.model.vis.headlight.diffuse[:] = 0.2
        self.data = mujoco.MjData(self.model)
        robot = station.spec()
        kin = RobotKinematics(robot, self.model, prefix=PREFIX)
        home = kin.to_q(station.home or robot.home)
        self._act = {n: self.model.actuator(PREFIX + n).id for n in JOINTS}
        self._qadr = {n: int(self.model.joint(PREFIX + n).qposadr[0]) for n in JOINTS}
        kin.write_qpos(self.data.qpos, home)
        for name, angle in zip(robot.joints, home, strict=True):
            self.data.ctrl[self._act[name]] = angle
        mujoco.mj_forward(self.model, self.data)

        self.lock = threading.Lock()
        self._stop = threading.Event()
        self._fps = fps
        self._renderer = mujoco.Renderer(self.model, 720, 960) if render else None
        self._frame: np.ndarray | None = None
        if self._renderer:
            self._render()

    # ------------------------------------------------------------------ station interface
    def wait_ready(self, timeout: float = 0.0) -> None:  # the model is ready once constructed
        return None

    def joints(self) -> dict[str, float]:
        with self.lock:
            return {n: float(self.data.qpos[a]) for n, a in self._qadr.items()}

    def command(self, targets: dict[str, float]) -> None:
        """Joint position targets in radians; the actuators hold the last command."""
        with self.lock:
            for name, value in targets.items():
                self.data.ctrl[self._act[name]] = float(value)

    def block(self, color: str) -> tuple[float, float, float]:
        with self.lock:
            x, y, z = self.data.xpos[self.model.body(f"{color}_block").id]
        return float(x), float(y), float(z)

    def image(self, cam: str = "overhead") -> np.ndarray:
        if cam != "overhead":
            raise KeyError(cam)
        with self.lock:
            if self._frame is None:
                raise KeyError("no frame rendered yet")
            return self._frame.copy()

    def reset_blocks(self, initial: dict[str, list[float]]) -> None:
        with self.lock:
            for color, (x, y, z) in initial.items():
                joint = self.model.joint(self.model.body(f"{color}_block").jntadr[0])
                q, v = int(joint.qposadr[0]), int(joint.dofadr[0])
                self.data.qpos[q:q + 7] = [x, y, z, 1, 0, 0, 0]
                self.data.qvel[v:v + 6] = 0
            mujoco.mj_forward(self.model, self.data)

    # ------------------------------------------------------------------ running
    def advance(self, seconds: float) -> None:
        """Step the physics by `seconds` of simulated time without waiting (tests, offline runs)."""
        with self.lock:
            mujoco.mj_step(self.model, self.data, nstep=int(round(seconds / self.model.opt.timestep)))

    def _render(self) -> None:
        self._renderer.update_scene(self.data, camera="overhead")
        self._frame = self._renderer.render().copy()

    def run(self) -> None:
        """Real-time loop: physics follows the wall clock, the overhead camera renders at `fps`. Blocks until `stop()`."""
        wall0, sim0, next_render = time.perf_counter(), self.data.time, 0.0
        while not self._stop.is_set():
            target = sim0 + time.perf_counter() - wall0
            with self.lock:
                steps = 0
                while self.data.time < target and steps < 400:  # catch up, but never spiral
                    mujoco.mj_step(self.model, self.data)
                    steps += 1
                if steps == 400:  # far behind real time (slow machine): re-anchor instead of racing
                    wall0, sim0 = time.perf_counter(), self.data.time
                if self._renderer and time.perf_counter() >= next_render:
                    self._render()
                    next_render = time.perf_counter() + 1.0 / self._fps
            time.sleep(0.002)

    def stop(self) -> None:
        self._stop.set()
