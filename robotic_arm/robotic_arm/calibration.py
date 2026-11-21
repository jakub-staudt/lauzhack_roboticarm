"""Table rectangle (u, v in 0..1) -> arm base frame (x, y) [m], fitted from arm poses placed on known points.

The user places the gripper tip on the rectangle's corners, edge midpoints and centre. Forward kinematics of
the measured joints gives the tip position in the arm's base frame; a homography fitted to those pairs
maps any (u, v) to (x, y). Using a homography (not just an affine map) absorbs a rectangle that is slightly
rotated or keystoned relative to the arm.

u runs along the rectangle's first edge, v along the second; (0, 0) is tag 0. Name the corners in the order the
eye-tracking side uses; see ../PROJECT.md.
"""

from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path

import numpy as np

#: Calibration points: name -> (u, v). Corners first; the extra points average out placement error.
POINTS: dict[str, tuple[float, float]] = {
    "corner_00": (0.0, 0.0),
    "corner_10": (1.0, 0.0),
    "corner_11": (1.0, 1.0),
    "corner_01": (0.0, 1.0),
    "edge_u_v0": (0.5, 0.0),
    "edge_u1_v": (1.0, 0.5),
    "edge_u_v1": (0.5, 1.0),
    "edge_u0_v": (0.0, 0.5),
    "centre": (0.5, 0.5),
}


def _normalise(pts: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Hartley normalisation: centroid at the origin, mean distance sqrt(2). Returns (points, 3x3 transform)."""
    c = pts.mean(axis=0)
    d = np.linalg.norm(pts - c, axis=1).mean()
    s = np.sqrt(2) / d if d > 0 else 1.0
    T = np.array([[s, 0, -s * c[0]], [0, s, -s * c[1]], [0, 0, 1.0]])
    return (pts - c) * s, T


def fit_homography(src: np.ndarray, dst: np.ndarray) -> np.ndarray:
    """3x3 H with dst ~ H @ src (both Nx2, N >= 4), normalised DLT."""
    src, dst = np.asarray(src, float), np.asarray(dst, float)
    if len(src) < 4 or len(src) != len(dst):
        raise ValueError("A homography needs at least 4 matching points.")
    s, Ts = _normalise(src)
    d, Td = _normalise(dst)
    rows = []
    for (x, y), (X, Y) in zip(s, d, strict=True):
        rows.append([-x, -y, -1, 0, 0, 0, X * x, X * y, X])
        rows.append([0, 0, 0, -x, -y, -1, Y * x, Y * y, Y])
    _, _, vt = np.linalg.svd(np.asarray(rows))
    H = vt[-1].reshape(3, 3)
    H = np.linalg.inv(Td) @ H @ Ts
    return H / H[2, 2]


def apply_homography(H: np.ndarray, pts: np.ndarray) -> np.ndarray:
    pts = np.atleast_2d(np.asarray(pts, float))
    homog = np.hstack([pts, np.ones((len(pts), 1))]) @ H.T
    return homog[:, :2] / homog[:, 2:3]


@dataclass
class Calibration:
    H: np.ndarray  # (u, v) -> (x, y) [m], arm base frame
    z_table: float  # mean tip height over the calibration points [m]
    residual_max_m: float
    residual_rms_m: float
    samples: list[dict]  # name, u, v, joints, xyz, residual_m

    def xy(self, u: float, v: float) -> tuple[float, float]:
        x, y = apply_homography(self.H, np.array([[u, v]]))[0]
        return float(x), float(y)

    def to_json(self) -> dict:
        return {
            "H": self.H.tolist(), "z_table": self.z_table,
            "residual_max_m": self.residual_max_m, "residual_rms_m": self.residual_rms_m,
            "samples": self.samples,
        }

    @classmethod
    def from_json(cls, data: dict) -> Calibration:
        return cls(np.asarray(data["H"], float), float(data["z_table"]), float(data["residual_max_m"]),
                   float(data["residual_rms_m"]), list(data["samples"]))

    def save(self, path: str | Path) -> None:
        Path(path).write_text(json.dumps(self.to_json(), indent=2))

    @classmethod
    def load(cls, path: str | Path) -> Calibration:
        return cls.from_json(json.loads(Path(path).read_text()))


def fit_calibration(kin, recorded: list[dict]) -> Calibration:
    """`recorded`: dicts with `name`, `u`, `v`, `joints` (app units, as in /api/state `measured`)."""
    if len(recorded) < 4:
        raise ValueError("Record at least 4 points (the four corners).")
    uv = np.array([[r["u"], r["v"]] for r in recorded], float)
    xyz = np.array([kin.tcp(r["joints"])[:3, 3] for r in recorded], float)
    H = fit_homography(uv, xyz[:, :2])
    err = np.linalg.norm(apply_homography(H, uv) - xyz[:, :2], axis=1)
    samples = [
        {"name": r["name"], "u": r["u"], "v": r["v"], "joints": r["joints"], "xyz": xyz[i].tolist(),
         "residual_m": float(err[i])}
        for i, r in enumerate(recorded)
    ]
    return Calibration(H, float(xyz[:, 2].mean()), float(err.max()), float(np.sqrt((err**2).mean())), samples)
