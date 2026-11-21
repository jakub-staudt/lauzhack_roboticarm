"""Physical assembly registration and the read-only viewer contract."""
import math

import pytest

from soarm.panel_environment import (
    STANDARD_LAYOUT,
    STANDARD_TABLE_Z_M,
    environment_spec,
    mesh_triangles,
)


def transformed(asset, point):
    # The documented source axes are X horizontal, Y vertical, Z lateral.
    x, y, z = point
    return tuple(a + b for a, b in zip((x * .001, -z * .001, y * .001), asset["position_m"], strict=True))


def test_nominal_base_seats_on_five_mm_riser_and_mount_is_on_right():
    spec = environment_spec(STANDARD_LAYOUT)
    plate, bottom, middle, top = spec["assets"]
    assert spec["table_z_m"] == STANDARD_TABLE_Z_M == pytest.approx(-.0074)
    assert transformed(plate, (0, 0, 0))[2] == pytest.approx(-.0024)
    assert transformed(plate, (0, -5, 0))[2] == pytest.approx(spec["table_z_m"])
    # Front mounting holes coincide with the SO101 MJCF holes: native mesh
    # Z=62.27499104mm, X=+/-27.77649706mm, geom translation X=-6.365mm.
    for source_z, target_y in [(-25.65396881, -.02777649706), (-81.20696449, .02777649706)]:
        assert transformed(plate, (-17.04593086, 0, source_z)) == pytest.approx(
            (.06227499104 - .006365, target_y, -.0024), abs=1e-8)
    # Adjacent tongues overlap their sockets by 7.85mm, not entire uprights.
    assert transformed(bottom, (18.7, 223.1, 36.513)) == pytest.approx(
        transformed(middle, (0, 35, 0)), abs=1e-8)
    assert middle["position_m"] == top["position_m"]
    assert middle["position_m"][1] < -.1


def test_all_source_meshes_are_pinned_valid_and_stand_on_table():
    spec = environment_spec(STANDARD_LAYOUT)
    for asset in spec["assets"]:
        vertices, faces = mesh_triangles(asset)  # verifies every exact source SHA256
        assert len(faces) > 500
        assert all(math.isfinite(v) for point in vertices for v in point)
        assert min(transformed(asset, p)[2] for p in vertices) >= spec["table_z_m"] - 1e-8
        assert spec["source_commit"] in asset["source_url"]
    top = spec["assets"][-1]
    vertices, _ = mesh_triangles(top)
    assert max(transformed(top, p)[2] for p in vertices) == pytest.approx(.553038, abs=1e-7)


def test_layout_must_be_named_and_unknown_layout_cannot_silently_render():
    assert environment_spec(None) is None
    assert environment_spec("") is None
    with pytest.raises(ValueError, match="Unknown workspace layout"):
        environment_spec("guess-camera-position")
