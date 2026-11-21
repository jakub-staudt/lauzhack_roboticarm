"""The paper grid is rectilinear: every line meets the next at a right angle, and only the camera's perspective makes it look
skewed. So all detected line pixels must fit ONE perspective model of a right-angled grid, and that model gives

* the exact corners of the 20 x 20 cm square (right angles by construction),
* the mapping from picture pixels to table centimetres (and back),
* a check on every line: a line that does not belong to the grid (an arm edge, a cable) leaves a large residual.

World frame (centimetres), seen as in the picture: x to the right, y down, origin at the left square's top-left corner:
top line y = 0, bottom line y = 20, left line x = 0, middle line x = 20, right line x = 20 + right_cm.
On the real table the right square is about 21 cm wide (measured from live frames, +-1 cm), not 20.
"""

from __future__ import annotations

import cv2
import numpy as np

SIDE_CM = 20.0
RIGHT_SQUARE_CM = 21.1      # measured on live frames with all five lines visible: 21.1 cm +- 0.24 (one frame gave 22.0)
WORLD_CORNERS = np.array([[0, 0], [SIDE_CM, 0], [SIDE_CM, SIDE_CM], [0, SIDE_CM]], np.float32)   # TL, TR, BR, BL


class GridFit:
    """Perspective model image -> table cm for one picture."""

    def __init__(self, matrix: np.ndarray, centre: tuple[float, float], scale: float, rms_mm: float, per_line_mm: dict[str, float]):
        self.matrix, self.centre, self.scale = matrix, centre, scale
        self.rms_mm, self.per_line_mm = rms_mm, per_line_mm
        self.inverse = np.linalg.inv(matrix)

    def to_cm(self, points: np.ndarray) -> np.ndarray:
        """Picture pixels (N, 2) -> table cm (N, 2)."""
        p = np.atleast_2d(np.asarray(points, float))
        q = self.matrix @ np.stack([(p[:, 0] - self.centre[0]) / self.scale, (p[:, 1] - self.centre[1]) / self.scale, np.ones(len(p))])
        return np.stack([q[0] / q[2], q[1] / q[2]], 1)

    def to_px(self, cm: np.ndarray) -> np.ndarray:
        """Table cm (N, 2) -> picture pixels (N, 2)."""
        c = np.atleast_2d(np.asarray(cm, float))
        q = self.inverse @ np.stack([c[:, 0], c[:, 1], np.ones(len(c))])
        return np.stack([q[0] / q[2] * self.scale + self.centre[0], q[1] / q[2] * self.scale + self.centre[1]], 1)

    def corners_px(self) -> np.ndarray:
        """TL, TR, BR, BL of the 20 x 20 cm square in the picture."""
        return self.to_px(WORLD_CORNERS)


def world_line(name: str, right_cm: float) -> tuple[str, float]:
    return {"top": ("y", 0.0), "bottom": ("y", SIDE_CM), "left": ("x", 0.0), "mid": ("x", SIDE_CM),
            "right": ("x", SIDE_CM + right_cm)}[name]


def fit_grid(points: dict[str, np.ndarray], init_corners_px: np.ndarray, shape: tuple[int, int], right_cm: float = RIGHT_SQUARE_CM,
             iterations: int = 60) -> GridFit | None:
    """Least-squares perspective fit of the named lines' pixels (`points[name]`: (N, 2)) to the grid.

    `init_corners_px` (TL, TR, BR, BL of the square in the picture) only starts the search. Needs two horizontal and two
    vertical lines; returns None if the fit cannot be made."""
    names = [n for n, p in points.items() if len(p) >= 8]
    if sum(world_line(n, right_cm)[0] == "x" for n in names) < 2 or sum(world_line(n, right_cm)[0] == "y" for n in names) < 2:
        return None
    height, width = shape[:2]
    centre, scale = (width / 2.0, height / 2.0), max(width, height) / 2.0
    norm = lambda p: np.stack([(p[:, 0] - centre[0]) / scale, (p[:, 1] - centre[1]) / scale], 1)  # noqa: E731
    h0 = cv2.getPerspectiveTransform(norm(np.asarray(init_corners_px, float)).astype(np.float32), WORLD_CORNERS)
    p = (h0 / h0[2, 2]).ravel()[:8].astype(float)
    data = {n: (norm(points[n]), *world_line(n, right_cm)) for n in names}

    def residuals(params: np.ndarray) -> np.ndarray:
        m = np.append(params, 1.0).reshape(3, 3)
        out = []
        for xy, kind, value in data.values():
            q = m @ np.stack([xy[:, 0], xy[:, 1], np.ones(len(xy))])
            out.append((q[0] if kind == "x" else q[1]) / q[2] - value)
        return np.concatenate(out)

    r = residuals(p)
    cost, damping = float(r @ r), 1e-3
    for _ in range(iterations):
        jac = np.empty((len(r), 8))
        for j in range(8):
            delta = np.zeros(8)
            delta[j] = 1e-6 * max(1.0, abs(p[j]))
            jac[:, j] = (residuals(p + delta) - r) / delta[j]
        a, g = jac.T @ jac, jac.T @ r
        improved = False
        for _try in range(25):
            try:
                trial = p + np.linalg.solve(a + damping * np.diag(np.diag(a)), -g)
            except np.linalg.LinAlgError:
                return None
            rt = residuals(trial)
            if np.all(np.isfinite(rt)) and float(rt @ rt) < cost:
                improved, p, r, cost, damping = True, trial, rt, float(rt @ rt), damping / 3
                break
            damping *= 4
        if not improved:
            break
    matrix = np.append(p, 1.0).reshape(3, 3)
    fit = GridFit(matrix, centre, scale, 0.0, {})
    per_line, total, count = {}, 0.0, 0
    for n, (xy, kind, value) in data.items():
        q = matrix @ np.stack([xy[:, 0], xy[:, 1], np.ones(len(xy))])
        err = (q[0] if kind == "x" else q[1]) / q[2] - value
        per_line[n] = float(np.sqrt(np.mean(err ** 2)) * 10)
        total, count = total + float(err @ err), count + len(err)
    fit.rms_mm, fit.per_line_mm = float(np.sqrt(total / count) * 10), per_line
    return fit
