"""Fit the simulator's working rectangle to the corners recorded on the REAL arm (calibration_poses.json).

The arm's base frame (x forward, y left) is the same in the simulator and on the real arm, so the real corner_00 is where the
simulator's (0, 0) should be. The rectangle stays orthogonal: a rotation plus one stretch along each side, no shear. Needs only numpy.
"""

from __future__ import annotations

import json
import math
from pathlib import Path

import numpy as np


def load_points(path: str | Path) -> list[tuple[float, float, float, float]]:
    """[(u, v, x, y)]: unit coordinates of each recorded corner and the tip position [m] in the arm base frame."""
    return [(float(r["u"]), float(r["v"]), float(r["xyz_m"][0]), float(r["xyz_m"][1]))
            for r in json.loads(Path(path).read_text()) if "xyz_m" in r and "u" in r and "v" in r]


def _forward(p, u, v):
    tx, ty, th, su, sv = p
    return np.array([tx + sv * v * math.cos(th) + su * u * math.sin(th), ty + sv * v * math.sin(th) - su * u * math.cos(th)])


def fit(points: list[tuple[float, float, float, float]]) -> dict:
    """-> {origin: (x, y) of (0, 0) in the base frame, theta, su_m, sv_m, residual_mm}. Needs at least 3 points."""
    if len(points) < 3:
        raise ValueError("at least 3 recorded corners are needed")
    uv = np.array([[p[0], p[1]] for p in points])
    xy = np.array([[p[2], p[3]] for p in points])
    coeff, *_ = np.linalg.lstsq(np.column_stack([uv, np.ones(len(uv))]), xy, rcond=None)
    p = np.array([coeff[2][0], coeff[2][1], math.atan2(coeff[1][1], coeff[1][0]), np.linalg.norm(coeff[0]), np.linalg.norm(coeff[1])])

    def residuals(q):
        return np.concatenate([_forward(q, u, v) - t for (u, v), t in zip(uv, xy, strict=True)])

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
    worst = max(float(np.linalg.norm(_forward(p, u, v) - [x, y]) * 1000) for u, v, x, y in points)
    return {"origin": (float(p[0]), float(p[1])), "theta": float(p[2]), "su_m": float(p[3]), "sv_m": float(p[4]), "residual_mm": worst}
