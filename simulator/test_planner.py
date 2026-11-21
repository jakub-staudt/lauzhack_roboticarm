"""Pick-and-place planning on the MuJoCo kinematics (no simulator needed)."""

import numpy as np
import pytest

from simulator.frame import block_to_world
from simulator.planner import GRASP_SHIFT, GRIP_Z, GRIPPER_CLOSED, GRIPPER_OPEN, Planner, Unreachable


@pytest.fixture(scope="module")
def planner():
    return Planner()


def test_pick_and_place_plan_grasps_at_the_source_and_releases_at_the_destination(planner):
    home = planner.to_rad({**planner.home, "gripper": 35.0})
    src, dst = (-0.11, 0.05), block_to_world("left", 16, 6)
    plan = planner.pick_and_place(home, src, dst)
    labels = [label for label, _, _ in plan]
    assert [labels.index(x) for x in ("open", "to source", "grasp", "carry", "release", "home")] == sorted(
        labels.index(x) for x in ("open", "to source", "grasp", "carry", "release", "home"))

    def tip(step):  # world position of the tool after a waypoint
        return planner.kin.tcp(planner.to_app(plan[step][1]))[:3, 3] + planner.base

    grasp = max(i for i, w in enumerate(plan) if w[0] == "grasp")
    release = max(i for i, w in enumerate(plan) if w[0] == "release")
    assert np.allclose(tip(grasp)[:2], src, atol=0.004) and tip(grasp)[2] == pytest.approx(GRIP_Z, abs=0.004)
    # the tip releases GRASP_SHIFT beyond the target along the jaw axis, because the held cube sits against the fixed jaw
    assert np.linalg.norm(tip(release)[:2] - dst) == pytest.approx(GRASP_SHIFT, abs=0.004)
    gripper = lambda i: planner.to_app(plan[i][1])["gripper"]  # noqa: E731
    assert gripper(grasp) == pytest.approx(GRIPPER_CLOSED, abs=0.5)
    assert gripper(release) == pytest.approx(GRIPPER_OPEN, abs=0.5)
    # the arm returns to its home pose and no waypoint jumps far from the previous one
    assert planner.to_app(plan[-1][1])["shoulder_pan"] == pytest.approx(planner.home["shoulder_pan"], abs=0.5)
    steps = [max(abs(plan[i][1][j] - plan[i - 1][1][j]) for j in plan[i][1]) for i in range(1, len(plan))]
    assert max(steps) < 0.25


def test_unreachable_spots_are_refused_with_a_reason(planner):
    home = planner.to_rad({**planner.home, "gripper": 35.0})
    with pytest.raises(Unreachable, match="source"):
        planner.pick_and_place(home, (0.45, 0.3), block_to_world("left", 16, 6))
    with pytest.raises(Unreachable, match="destination"):
        planner.pick_and_place(home, (-0.11, 0.05), (0.45, -0.3))
