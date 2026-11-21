"""Range estimates on the shipped numerical model, without physical commands."""
import math
import time

import pytest

import soarm
from soarm.panel import RobotDescription
from soarm.panel_cartesian import CartesianSolver, Workspace
from soarm.panel_cartesian_ranges import estimate_ranges

pytestmark = pytest.mark.twin


@pytest.mark.parametrize("measured_fixture", [False, True], ids=["home", "measured-effective-limits"])
def test_model_ranges_preserve_measured_tilt_and_respect_floor(measured_fixture):
    description = RobotDescription.load(soarm.ROBOT_DIR)
    limits = dict(description.limits)
    pose = dict(description.home)
    if measured_fixture:
        # Frozen read-only observation and effective software/EEPROM intersection;
        # this fixture never opens a connection or commands the physical arm.
        limits.update(shoulder_pan=(-105.0, 64.599609375), shoulder_lift=(-95.0, 95.0),
                      elbow_flex=(-92.0, 87.1875), wrist_flex=(-95.0, 95.0),
                      wrist_roll=(-113.818359375, 95.185546875), gripper=(7.748538011695906, 100.0))
        pose.update(shoulder_pan=-1.2305, shoulder_lift=-10.9863, elbow_flex=34.7168,
                    wrist_flex=-14.5898, wrist_roll=-13.0078, gripper=98.9766)
    solver = CartesianSolver(soarm.ROBOT_DIR, limits)
    origin = solver.position(pose)
    floor = 0.0126 if measured_fixture else origin[2] - 0.025
    started = time.monotonic()
    result = estimate_ranges(solver, pose, limits,
                             Workspace(table_z_m=floor, tcp_margin_m=0.0))
    elapsed = time.monotonic() - started
    assert result["status"] == "ready", (elapsed, result)
    heading = solver.heading(pose)
    pitch = solver.pitch(pose)
    for name, entry in result["directions"].items():
        distance = entry["distance_m"]
        assert 0 <= distance <= 0.4
        # Independently replay the reported continuous line in small increments.
        current = dict(pose)
        steps = max(1, math.ceil(distance / 0.002))
        for index in range(1, steps + 1):
            travel = distance * index / steps
            target = list(origin)
            if name in ("up", "down"):
                target[2] += travel * (1 if name == "up" else -1)
            else:
                for axis in (0, 1):
                    target[axis] += travel * heading[axis] * (1 if name == "forward" else -1)
            current = solver.solve(current, tuple(target), pitch=pitch)
            assert math.dist(solver.position(current), target) <= 0.00003
            assert abs(solver._angle(solver.pitch(current) - pitch)) < math.radians(0.011)
            assert solver.position(current)[2] >= floor - 0.00003
            assert all(limits[joint][0] <= value <= limits[joint][1]
                       for joint, value in current.items())
    assert result["directions"]["down"]["distance_m"] <= origin[2] - floor + 1e-9
