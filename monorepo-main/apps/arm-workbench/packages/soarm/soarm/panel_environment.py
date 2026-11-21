"""Static, nominal workspace geometry for the measured-pose viewer.

This module never changes the robot model or supplies collision geometry to a
controller. Layout names select a documented physical assembly, not calibration.
"""
from __future__ import annotations

import hashlib
import json
import struct
from pathlib import Path
from typing import Any

STANDARD_LAYOUT = "so101_overhead_32x32_right"
STANDARD_TABLE_Z_M = -0.0074
ASSET_DIR = Path(__file__).parent / "environment_assets" / "overhead_32x32"


def environment_spec(layout: str | None) -> dict[str, Any] | None:
    """Load the one supported physical assembly; empty/None means unspecified."""
    if not layout:
        return None
    if layout != STANDARD_LAYOUT:
        raise ValueError(f"Unknown workspace layout: {layout}")
    return json.loads((ASSET_DIR / "provenance.json").read_text(encoding="utf-8"))


def environment_status(spec: dict[str, Any] | None) -> dict[str, Any] | None:
    if spec is None:
        return None
    return {"layout": spec["kind"], "table_z_m": spec["table_z_m"],
            "camera_stand_side": "right (-Y)", "source_commit": spec["source_commit"],
            "visual_only": True, "placement": "nominal CAD assembly; not surveyed calibration"}


def mesh_triangles(
    asset: dict[str, Any],
) -> tuple[list[tuple[float, float, float]], list[tuple[int, int, int]]]:
    """Read the small binary STL without adding a runtime mesh-library dependency."""
    data = (ASSET_DIR / asset["file"]).read_bytes()
    if hashlib.sha256(data).hexdigest() != asset["sha256"]:
        raise ValueError(f"Environment asset checksum mismatch: {asset['file']}")
    count, = struct.unpack_from("<I", data, 80)
    if len(data) != 84 + count * 50:
        raise ValueError(f"Invalid binary STL: {asset['file']}")
    vertices = []
    for facet in struct.iter_unpack("<12fH", data[84:]):
        vertices.extend((tuple(facet[3:6]), tuple(facet[6:9]), tuple(facet[9:12])))
    faces = [(i, i + 1, i + 2) for i in range(0, len(vertices), 3)]
    return vertices, faces


def add_environment(scene: Any, spec: dict[str, Any]) -> list[Any]:
    """Add always-visible meshes outside /measured; no pose or hardware inputs."""
    import numpy as np

    handles = [scene.add_frame("/environment", show_axes=False)]
    for asset in spec["assets"]:
        vertices, faces = mesh_triangles(asset)
        name = Path(asset["file"]).stem
        handles.append(scene.add_mesh_simple(
            f"/environment/{name}",
            np.asarray(vertices, dtype=np.float32) * spec["scale_to_m"],
            np.asarray(faces, dtype=np.uint32),
            position=tuple(asset["position_m"]), wxyz=tuple(spec["rotation_wxyz"]),
            color=(72, 117, 126), flat_shading=True,
        ))
    return handles
