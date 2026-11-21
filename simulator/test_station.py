"""Geometry and grasp tests for the simulator station."""

import mujoco
import numpy as np
import pytest

from simulator.run import build_model, load_station
from soarm_lab.kinematics import RobotKinematics
from soarm_lab.station import PREFIX


def test_gaze_station_geometry_and_overhead_camera():
    station = load_station()
    model = build_model(station)
    assert model.nu == 6
    assert model.geom("gaze/grid_right").pos[0] - model.geom("gaze/grid_center").pos[0] == pytest.approx(0.3)
    assert model.geom("gaze/grid_center").pos[0] - model.geom("gaze/grid_left").pos[0] == pytest.approx(0.3)
    assert model.geom("gaze/grid_top").pos[1] - model.geom("gaze/grid_bottom").pos[1] == pytest.approx(0.3)
    camera, = station.cameras
    np.testing.assert_allclose(model.camera(camera.name).pos,
                               (station.T_base2world @ camera.T_cam2base)[:3, 3])
    assert model.camera("robot/wrist_cam").id >= 0


@pytest.mark.parametrize("color", ["red", "blue", "yellow"])
def test_so101_can_pick_up_each_gaze_cube(color):
    station = load_station()
    model = build_model(station)
    robot = station.spec()
    kinematics = RobotKinematics(robot, model, prefix=PREFIX)
    data = mujoco.MjData(model)
    actuators = [model.actuator(PREFIX + name).id for name in robot.joints]

    cube = model.geom(f"{color}_block")
    assert cube.size == pytest.approx([0.01] * 3)
    mujoco.mj_forward(model, data)
    start_z = data.geom_xpos[cube.id, 2]
    grasp = data.geom_xpos[cube.id] - station.T_base2world[:3, 3] + [0, 0, 0.002]
    hover = grasp + [0, 0, 0.05]
    # Tool X points down and tool Z (jaw closing) points along world X.
    rotation = np.array([[0., 0., 1.], [0., 1., 0.], [-1., 0., 0.]])

    def step(joints, steps):
        data.ctrl[actuators] = kinematics.to_q(joints)
        mujoco.mj_step(model, data, nstep=steps)

    def move(start, end, seed):
        for position in np.linspace(start, end, 50):
            seed = kinematics.ik_step(position, rotation, seed, rot_gain=0.8, tol=0.00012, iters=250).joints
            step(seed, 6)
        return seed

    seed = {**(station.home or robot.home), "gripper": 45.}
    joints = kinematics.ik(hover, rotation, seed, rot_gain=0.8, tol=0.00012, iters=250).joints
    kinematics.write_qpos(data.qpos, kinematics.to_q(joints))
    step(joints, 200)

    joints = move(hover, grasp, joints)
    step(joints, 100)
    for opening in np.linspace(joints["gripper"], 0, 30):
        joints["gripper"] = float(opening)
        step(joints, 6)
    step(joints, 100)
    joints = move(grasp, hover, joints)
    step(joints, 80)
    assert data.geom_xpos[cube.id, 2] - start_z > 0.03
