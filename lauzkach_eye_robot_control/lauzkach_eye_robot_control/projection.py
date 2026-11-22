"""Picture pixel <-> table position, for a point at a known height above the table.

The detector's rectangle gives an exact map for points ON the table (pixel -> cm, a homography `M`). A point above the table
appears shifted in an angled camera view (parallax), so a tip hovering 3 cm up is not where its table position says.
For a pinhole camera the shift is linear in the height in homogeneous coordinates:

    pixel_h  ~  M^-1 [X, Y, 1]  +  height * lift          (X, Y: table cm; height: cm; `lift`: a 3-vector)

`lift` is unknown until it is measured (see `feature_model.py`); with `lift=None` the correction is off, which is exact for
points on the table (and good to a few mm up to about 5 mm height).
"""

from __future__ import annotations

import numpy as np


def table_from_pixel(px_to_cm: list[list[float]], px: float, py: float, height_cm: float = 0.0,
                     lift: np.ndarray | list[float] | None = None) -> tuple[float, float]:
    """(X, Y) in table cm of the point that appears at pixel (px, py) and is `height_cm` above the table."""
    m = np.array(px_to_cm, float)
    q = np.array([px, py, 1.0])
    lf = np.zeros(3) if lift is None else np.asarray(lift, float)
    lam = (1.0 + height_cm * (m @ lf)[2]) / (m @ q)[2]
    p = m @ (lam * q - height_cm * lf)
    return float(p[0] / p[2]), float(p[1] / p[2])


def pixel_from_table(px_to_cm: list[list[float]], x_cm: float, y_cm: float, height_cm: float = 0.0,
                     lift: np.ndarray | list[float] | None = None) -> tuple[float, float]:
    """The pixel at which the point (X, Y) cm, `height_cm` above the table, appears."""
    inv = np.linalg.inv(np.array(px_to_cm, float))
    lf = np.zeros(3) if lift is None else np.asarray(lift, float)
    v = inv @ np.array([x_cm, y_cm, 1.0]) + height_cm * lf
    return float(v[0] / v[2]), float(v[1] / v[2])
