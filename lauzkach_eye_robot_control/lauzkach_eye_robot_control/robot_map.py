"""From the camera rectangle to the arm's own coordinates, calibrated with the points recorded on the real arm.

The paper rectangle is described by unit coordinates (u, v): (0, 0) = the corner nearest the robot on its left, u runs to the
robot's right, v runs away from the robot, (1, 1) = the far-right corner. The robot's calibration file lists the tip position
[m, arm base frame: x forward, y left, z up] measured with the tip on some of those corners.

This does NOT assume the rectangle is exactly 20 x 20 cm (or that the simulator's drawn blocks are): it fits what the robot
measured. The paper's lines are orthogonal, so the fit is a rotation plus a separate stretch along each axis (no shear). The
shear a free affine fit would show is reported as a warning: it means the recorded points are inconsistent (for instance
recorded at different gripper tilts), and the residuals say by how much.
"""

from __future__ import annotations

import json
import math
from pathlib import Path

import numpy as np

NOMINAL_CM = 20.0
CORNERS = (("near-left", 0.0, 0.0), ("near-right", 1.0, 0.0), ("far-right", 1.0, 1.0), ("far-left", 0.0, 1.0))


class CalibrationError(RuntimeError):
    pass


def load_points(path: str | Path) -> list[dict]:
    """The calibration file written by `record_point.py`: a list of {name, u, v, xyz_m, ...}."""
    data = json.loads(Path(path).read_text())
    points = [{"name": r["name"], "u": float(r["u"]), "v": float(r["v"]), "x": float(r["xyz_m"][0]),
               "y": float(r["xyz_m"][1]), "z": float(r["xyz_m"][2])} for r in data if "xyz_m" in r and "u" in r]
    if len(points) < 3:
        raise CalibrationError(f"{path} has {len(points)} usable point(s); at least 3 (three corners) are needed")
    return points


class RobotMap:
    """(u, v) <-> (x, y) in the arm base frame, fitted to the recorded points."""

    def __init__(self, points: list[dict], source: str = "") -> None:
        if len(points) < 3:
            raise CalibrationError("at least 3 recorded points are needed")
        self.points, self.source = points, source
        uv = np.array([[p["u"], p["v"]] for p in points])
        xy = np.array([[p["x"], p["y"]] for p in points])
        self.z_table = float(np.mean([p["z"] for p in points]))
        # free affine fit: only to measure how far the recorded points are from a rectangle
        design = np.column_stack([uv, np.ones(len(uv))])
        coeff, *_ = np.linalg.lstsq(design, xy, rcond=None)
        col_u, col_v = coeff[0], coeff[1]
        cosine = float(col_u @ col_v / (np.linalg.norm(col_u) * np.linalg.norm(col_v)))
        self.shear_deg = abs(90.0 - math.degrees(math.acos(max(-1.0, min(1.0, cosine)))))
        # rotation + one stretch per axis (right angles kept): position = t + sv*v*e1 + su*u*e2, e1 forward, e2 to the right
        theta0 = math.atan2(col_v[1], col_v[0])
        p = np.array([coeff[2][0], coeff[2][1], theta0, float(np.linalg.norm(col_u)), float(np.linalg.norm(col_v))])
        p = self._levenberg_marquardt(p, uv, xy)
        self.params = p
        self.residual_mm = {pt["name"]: float(np.linalg.norm(self._forward(p, pt["u"], pt["v"]) - [pt["x"], pt["y"]]) * 1000)
                            for pt in points}

    @staticmethod
    def _forward(p: np.ndarray, u: float, v: float) -> np.ndarray:
        tx, ty, th, su, sv = p
        return np.array([tx + sv * v * math.cos(th) + su * u * math.sin(th), ty + sv * v * math.sin(th) - su * u * math.cos(th)])

    def _levenberg_marquardt(self, p: np.ndarray, uv: np.ndarray, xy: np.ndarray) -> np.ndarray:
        def residuals(q):
            return np.concatenate([self._forward(q, u, v) - t for (u, v), t in zip(uv, xy, strict=True)])
        r, damping = residuals(p), 1e-3
        for _ in range(100):
            jac = np.empty((len(r), 5))
            for j in range(5):
                d = np.zeros(5)
                d[j] = 1e-7
                jac[:, j] = (residuals(p + d) - r) / 1e-7
            a, g = jac.T @ jac, jac.T @ r
            for _try in range(30):
                trial = p + np.linalg.solve(a + damping * np.eye(5) * (np.diag(a).mean() + 1e-12), -g)
                rt = residuals(trial)
                if rt @ rt < r @ r:
                    p, r, damping = trial, rt, damping / 3
                    break
                damping *= 4
            else:
                break
        return p

    # ------------------------------------------------------------------ conversions
    def uv_to_xy(self, u: float, v: float) -> tuple[float, float]:
        x, y = self._forward(self.params, u, v)
        return float(x), float(y)

    def xy_to_uv(self, x: float, y: float) -> tuple[float, float]:
        tx, ty, th, su, sv = self.params
        dx, dy = x - tx, y - ty
        return (dx * math.sin(th) - dy * math.cos(th)) / su, (dx * math.cos(th) + dy * math.sin(th)) / sv

    @property
    def size_cm(self) -> tuple[float, float]:
        """(extent along u = to the robot's right, extent along v = away from the robot) in centimetres."""
        return float(self.params[3] * 100), float(self.params[4] * 100)

    def corners(self) -> list[dict]:
        return [{"index": i, "name": name, "u": u, "v": v, "x": self.uv_to_xy(u, v)[0], "y": self.uv_to_xy(u, v)[1]}
                for i, (name, u, v) in enumerate(CORNERS)]

    def summary(self) -> dict:
        w, d = self.size_cm
        warnings = []
        if max(abs(w - NOMINAL_CM), abs(d - NOMINAL_CM)) > 0.1 * NOMINAL_CM:
            warnings.append(f"the robot's points imply a rectangle of {w:.1f} x {d:.1f} cm, not 20 x 20. That is a stretch of "
                            f"{(w / NOMINAL_CM - 1) * 100:+.0f}% and {(d / NOMINAL_CM - 1) * 100:+.0f}%, which is more than slight: "
                            "check the recorded points (see below) or the paper")
        if self.shear_deg > 2.0:
            warnings.append(f"the recorded corners are {self.shear_deg:.1f} degrees away from a right angle; the paper's lines are "
                            "orthogonal, so the points were probably recorded at different gripper tilts")
        worst = max(self.residual_mm.values())
        if worst > 5.0:
            warnings.append(f"the points do not fit a rectangle exactly (worst {worst:.0f} mm); the fit is a compromise")
        return {"source": self.source, "n_points": len(self.points), "points": [p["name"] for p in self.points],
                "size_cm": {"u": round(w, 1), "v": round(d, 1)}, "shear_deg": round(self.shear_deg, 1),
                "rotation_deg": round(math.degrees(float(self.params[2])), 1), "residual_mm": {k: round(v, 1) for k, v in self.residual_mm.items()},
                "z_table_m": round(self.z_table, 4), "warnings": warnings,
                "corners": [{**c, "x": round(c["x"], 4), "y": round(c["y"], 4)} for c in self.corners()]}

    @classmethod
    def load(cls, path: str | Path) -> RobotMap:
        return cls(load_points(path), source=str(path))
