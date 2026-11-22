"""Relate a feature seen on the gripper (a sticker, a dark board) to the tip, fitted from data.

For every sample we know the arm's own pose (tip position and tool orientation from its joint angles) and the pixel where
the feature appeared. A feature sits at a fixed offset `o` in the tool's frame, so its 3D position is  tip + R @ o.
Three things are unknown and fitted:

* `o`      where the feature is on the gripper (metres, tool frame); fixed to zero for a marker at the tip;
* `lift`   how a point appears shifted by its height above the table (see projection.py);
* `bias`   a small shift (metres) between the arm's coordinates and the board's: a correction to the 4-point calibration.

With the model, a pixel plus the arm's pose (orientation and height are known) gives the tip's position on the board,
independently of the arm's own calibration: exactly what the closed loop needs. Data come from `tools/record_pose_samples.py`
while the arm is moved around by hand over the board.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

from lauzkach_eye_robot_control.drive import px_to_uv, uv_to_px
from lauzkach_eye_robot_control.robot_map import RobotMap


@dataclass
class Sample:
    tip: tuple[float, float, float]           # the arm's own tip position [m, base frame]
    rotation: list[list[float]]               # tool orientation in the base frame (3 x 3)
    pixel: tuple[float, float]                # where the feature appeared


@dataclass
class FeatureModel:
    px_to_cm: list[list[float]]
    faces: str
    robot_map: RobotMap
    offset: np.ndarray = field(default_factory=lambda: np.zeros(3))      # o, metres, tool frame
    lift: np.ndarray = field(default_factory=lambda: np.zeros(3))
    bias: np.ndarray = field(default_factory=lambda: np.zeros(2))        # metres, base frame
    rms_px: float | None = None
    n_samples: int = 0

    def _feature(self, tip, rotation) -> np.ndarray:
        return np.asarray(tip, float) + np.asarray(rotation, float) @ self.offset

    def predict_pixel(self, tip, rotation) -> tuple[float, float]:
        f = self._feature(tip, rotation)
        u, v = self.robot_map.xy_to_uv(f[0] - self.bias[0], f[1] - self.bias[1])
        return uv_to_px(self.px_to_cm, u, v, self.faces, height_cm=(f[2] - self.robot_map.z_table) * 100.0, lift=self.lift)

    def tip_xy_from_pixel(self, pixel, tip_z: float, rotation) -> tuple[float, float]:
        """Where the tip is on the board [m, arm base frame] given where the feature appears, the tip's height and the tool's
        orientation (both known from the arm's joint angles)."""
        r_o = np.asarray(rotation, float) @ self.offset
        height_cm = (tip_z + r_o[2] - self.robot_map.z_table) * 100.0
        u, v = px_to_uv(self.px_to_cm, pixel[0], pixel[1], self.faces, height_cm=height_cm, lift=self.lift)
        fx, fy = self.robot_map.uv_to_xy(u, v)
        return fx + self.bias[0] - r_o[0], fy + self.bias[1] - r_o[1]

    # ------------------------------------------------------------------ fitting
    @classmethod
    def fit(cls, samples: list[Sample], px_to_cm, faces: str, robot_map: RobotMap, fit_offset: bool = True,
            iterations: int = 80) -> FeatureModel:
        if len(samples) < 8:
            raise ValueError(f"need at least 8 samples to fit, got {len(samples)}")
        model = cls(px_to_cm, faces, robot_map)

        def unpack(p: np.ndarray) -> None:
            k = 0
            if fit_offset:
                model.offset, k = p[:3] / 100.0, 3                      # parameters in cm
            model.lift = np.array([p[k], p[k + 1], p[k + 2] / 1000.0])  # lift_z scaled so all parameters are of similar size
            model.bias = p[k + 3:k + 5] / 100.0

        def residuals(p: np.ndarray) -> np.ndarray:
            unpack(p)
            return np.concatenate([np.array(model.predict_pixel(s.tip, s.rotation)) - np.array(s.pixel) for s in samples])

        p = np.zeros(8 if fit_offset else 5)
        r = residuals(p)
        damping = 1e-2
        for _ in range(iterations):
            jac = np.empty((len(r), len(p)))
            for j in range(len(p)):
                d = np.zeros(len(p))
                d[j] = 1e-4
                jac[:, j] = (residuals(p + d) - r) / 1e-4
            a, g = jac.T @ jac, jac.T @ r
            improved, moved = False, 0.0
            for _try in range(30):
                trial = p + np.linalg.solve(a + damping * np.diag(np.diag(a) + 1e-9), -g)
                rt = residuals(trial)
                if np.all(np.isfinite(rt)) and rt @ rt < r @ r:
                    improved, moved = True, float(np.linalg.norm(trial - p))
                    p, r, damping = trial, rt, damping / 3
                    break
                damping *= 4
            if not improved or moved < 1e-9:
                break
        unpack(p)
        model.rms_px = float(math.sqrt(np.mean(r ** 2)))
        model.n_samples = len(samples)
        return model

    def summary(self) -> dict:
        return {"offset_cm": [round(float(v) * 100, 2) for v in self.offset], "lift": [round(float(v), 4) for v in self.lift],
                "bias_mm": [round(float(v) * 1000, 1) for v in self.bias], "rms_px": None if self.rms_px is None else round(self.rms_px, 2),
                "samples": self.n_samples}
