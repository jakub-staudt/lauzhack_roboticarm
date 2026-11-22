"""Coordinates of the gaze station: the two 200 x 200 mm blocks (squares) and the overhead camera.

World frame: x forward from the robot (it faces +x), y to its left, z up; the tabletop is z = 0.75 m.

Each block has its own frame in centimetres, seen from the robot's point of view: (0, 0) is the near-left corner,
the first coordinate runs to the robot's right (-y), the second away from the robot (+x), (20, 20) is the far-right
corner. The left block's (0, 0) is at world (x -0.24, y 0.2); the right block's at (x -0.24, y 0).
"""

from __future__ import annotations

import math

TABLE_Z = 0.75
SIDE_CM = 20.0
NEAR_X = -0.24  # x of the near edge of both blocks [m]
ORIGIN_Y = {"left": 0.20, "right": 0.0}  # y of each block's near-left corner [m]

# Overhead camera (scene.xml / station.json): straight down, image right = +x, image up = +y.
CAM_POS = (-0.14, 0.0, 1.27)  # centred over the two blocks, 0.52 m above the table
CAM_FOCAL = 820.0
CAM_SIZE = (960, 720)


# The working (left) block as a rectangle: origin = its (0, 0) corner in world metres, rotation about z, and the length of its two
# sides. The default is the ideal 200 x 200 mm square; `configure_left` fits it to the real arm's recorded corners (still a
# rectangle: a rotation and a stretch along each side, no shear).
LEFT = {"origin": (NEAR_X, ORIGIN_Y["left"]), "theta": 0.0, "su_m": SIDE_CM / 100.0, "sv_m": SIDE_CM / 100.0}
RIGHT_ENABLED = True


def configure_left(origin: tuple[float, float], theta: float, su_m: float, sv_m: float) -> None:
    """Move/rotate/stretch the working block; the right block is no longer used."""
    global RIGHT_ENABLED
    LEFT.update(origin=(float(origin[0]), float(origin[1])), theta=float(theta), su_m=float(su_m), sv_m=float(sv_m))
    RIGHT_ENABLED = False


def left_corners_world() -> list[tuple[float, float]]:
    """World (x, y) of the working block's corners (0,0), (20,0), (20,20), (0,20) in block coordinates."""
    return [block_to_world("left", cx, cy) for cx, cy in ((0, 0), (SIDE_CM, 0), (SIDE_CM, SIDE_CM), (0, SIDE_CM))]


def block_to_world(block: str, cx: float, cy: float) -> tuple[float, float]:
    """(cx, cy) in cm in a block's frame (0..20 each) -> world (x, y) in metres."""
    if block == "left":
        (ox, oy), th = LEFT["origin"], LEFT["theta"]
        u, v = cx / SIDE_CM * LEFT["su_m"], cy / SIDE_CM * LEFT["sv_m"]
        return ox + v * math.cos(th) + u * math.sin(th), oy + v * math.sin(th) - u * math.cos(th)
    return NEAR_X + cy / 100.0, ORIGIN_Y[block] - cx / 100.0


def world_to_block(x: float, y: float, margin_cm: float = 0.0) -> tuple[str, float, float] | None:
    """World (x, y) -> (block name, cx, cy) in cm, or None when the point is outside the block(s) (± margin)."""
    (ox, oy), th = LEFT["origin"], LEFT["theta"]
    dx, dy = x - ox, y - oy
    cx = (dx * math.sin(th) - dy * math.cos(th)) / LEFT["su_m"] * SIDE_CM
    cy = (dx * math.cos(th) + dy * math.sin(th)) / LEFT["sv_m"] * SIDE_CM
    if -margin_cm <= cx <= SIDE_CM + margin_cm and -margin_cm <= cy <= SIDE_CM + margin_cm:
        return "left", cx, cy
    if RIGHT_ENABLED:
        cx, cy = (ORIGIN_Y["right"] - y) * 100.0, (x - NEAR_X) * 100.0
        if -margin_cm <= cx <= SIDE_CM + margin_cm and -margin_cm <= cy <= SIDE_CM + margin_cm:
            return "right", cx, cy
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

