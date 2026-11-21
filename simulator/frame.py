"""Coordinates of the gaze station: the two 200 x 200 mm blocks (squares) and the overhead camera.

World frame: x forward from the robot (it faces +x), y to its left, z up; the tabletop is z = 0.75 m.

Each block has its own frame in centimetres, seen from the robot's point of view: (0, 0) is the near-left corner,
the first coordinate runs to the robot's right (-y), the second away from the robot (+x), (20, 20) is the far-right
corner. The left block's (0, 0) is at world (x -0.24, y 0.2); the right block's at (x -0.24, y 0).
"""

from __future__ import annotations

TABLE_Z = 0.75
SIDE_CM = 20.0
NEAR_X = -0.24  # x of the near edge of both blocks [m]
ORIGIN_Y = {"left": 0.20, "right": 0.0}  # y of each block's near-left corner [m]

# Overhead camera (scene.xml / station.json): straight down, image right = +x, image up = +y.
CAM_POS = (-0.14, 0.0, 1.27)  # centred over the two blocks, 0.52 m above the table
CAM_FOCAL = 820.0
CAM_SIZE = (960, 720)


def block_to_world(block: str, cx: float, cy: float) -> tuple[float, float]:
    """(cx, cy) in cm in a block's frame -> world (x, y) in metres."""
    return NEAR_X + cy / 100.0, ORIGIN_Y[block] - cx / 100.0


def world_to_block(x: float, y: float, margin_cm: float = 0.0) -> tuple[str, float, float] | None:
    """World (x, y) -> (block name, cx, cy) in cm, or None when the point is outside both blocks (± margin)."""
    for block, y0 in ORIGIN_Y.items():
        cx, cy = (y0 - y) * 100.0, (x - NEAR_X) * 100.0
        if -margin_cm <= cx <= SIDE_CM + margin_cm and -margin_cm <= cy <= SIDE_CM + margin_cm:
            return block, cx, cy
    return None


def pixel_to_world(px: float, py: float, z: float = TABLE_Z) -> tuple[float, float]:
    """Overhead-camera pixel -> world (x, y) of the point at height z."""
    depth = CAM_POS[2] - z
    return (CAM_POS[0] + (px - (CAM_SIZE[0] - 1) / 2) / CAM_FOCAL * depth,
            CAM_POS[1] - (py - (CAM_SIZE[1] - 1) / 2) / CAM_FOCAL * depth)


def world_to_pixel(x: float, y: float, z: float = TABLE_Z) -> tuple[float, float]:
    depth = CAM_POS[2] - z
    return ((x - CAM_POS[0]) / depth * CAM_FOCAL + (CAM_SIZE[0] - 1) / 2,
            -(y - CAM_POS[1]) / depth * CAM_FOCAL + (CAM_SIZE[1] - 1) / 2)


def describe(x: float, y: float) -> str:
    """'(12.0, 8.5)' for a point inside the left block (the working block), 'R(...)' in the right one, else world cm."""
    hit = world_to_block(x, y, margin_cm=0.5)
    if hit:
        block, cx, cy = hit
        return f"({cx:.1f}, {cy:.1f})" if block == "left" else f"R({cx:.1f}, {cy:.1f})"
    return f"world({x * 100:.1f}, {y * 100:.1f}) cm"

