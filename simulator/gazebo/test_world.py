"""The generated Gazebo world must be valid SDF and consistent with the MuJoCo model."""

import re
import shutil
import subprocess

import pytest

from simulator.gazebo.build_world import OUT, build
from simulator.run import build_model, load_station


def test_sdf_matches_mujoco_model():
    sdf = build()
    m = build_model(load_station())
    robot_joints = [m.joint(j).name.split("/")[-1] for j in range(m.njnt) if m.joint(j).name.startswith("robot/")]
    for name in robot_joints:
        assert f'<joint name="{name}" type="revolute">' in sdf
    robot_links = re.findall(r'<link name="(\w+_link)">', sdf)
    assert len(robot_links) == sum(m.body(b).name.startswith("robot/") for b in range(m.nbody))
    for color in ("red", "blue", "yellow"):
        assert f'<model name="{color}_block">' in sdf
    assert '<joint name="camera_mount_link_fixed" type="fixed">' in sdf  # jointless bodies must be welded, not left loose


@pytest.mark.skipif(shutil.which("gz") is None, reason="Gazebo is not installed")
def test_sdf_is_valid_for_gazebo():
    OUT.write_text(build())
    result = subprocess.run(["gz", "sdf", "--check", str(OUT)], capture_output=True, text=True)
    assert "Valid" in result.stdout, result.stdout + result.stderr
