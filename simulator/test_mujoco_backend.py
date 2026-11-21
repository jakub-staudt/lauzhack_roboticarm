"""The MuJoCo backend of the web UI, run without any window, camera or ROS."""

import math

import pytest

from simulator.frame import block_to_world
from simulator.mujoco_backend import JOINTS, MujocoStation
from simulator.planner import Planner


@pytest.fixture()
def sim():
    return MujocoStation(render=False)


def test_station_starts_at_home_with_blocks_on_the_table(sim):
    assert set(sim.joints()) == set(JOINTS)
    for color in ("red", "blue", "yellow"):
        assert sim.block(color)[2] == pytest.approx(0.76, abs=0.002)


def test_joint_commands_move_the_arm(sim):
    target = sim.joints()["shoulder_pan"] + 0.3
    sim.command({"shoulder_pan": target})
    sim.advance(1.5)
    assert sim.joints()["shoulder_pan"] == pytest.approx(target, abs=0.02)


def test_pick_and_place_through_the_backend_puts_the_block_at_the_destination(sim):
    planner = Planner()
    src, dst = sim.block("red")[:2], block_to_world("left", 16, 6)
    plan = planner.pick_and_place(sim.joints(), src, dst)
    for _label, joints, dwell in plan:
        sim.command(joints)
        sim.advance(dwell)
    x, y, z = sim.block("red")
    assert math.hypot(x - dst[0], y - dst[1]) < 0.01
    assert z == pytest.approx(0.76, abs=0.003)


def test_reset_blocks_restores_the_start_positions(sim):
    start = {c: sim.block(c) for c in ("red", "blue", "yellow")}
    sim.reset_blocks({"red": [-0.1, 0.1, 0.9]})
    sim.advance(0.2)
    assert sim.block("red")[2] < 0.9  # dropped
    sim.reset_blocks({c: list(p) for c, p in start.items()})
    assert sim.block("red") == pytest.approx(start["red"], abs=1e-6)
