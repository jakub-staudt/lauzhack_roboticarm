"""Open the gaze station in the MuJoCo viewer."""

from pathlib import Path

import mujoco

from soarm import ROBOT_DIR
from soarm_lab.kinematics import RobotKinematics
from soarm_lab.station import PREFIX, Station


def load_station() -> Station:
    """Read the gaze station's robot, workspace, and camera configuration."""
    return Station.load(Path(__file__).resolve().parent)


def build_model(station: Station) -> mujoco.MjModel:
    """Compile the gaze scene with the workbench's existing SO-101 assets."""
    scene = mujoco.MjSpec.from_file(str(station.scene_path))
    # MuJoCo resolves included mesh paths against the original scene's directory.
    for mesh in scene.meshes:
        mesh.file = str(ROBOT_DIR / "assets" / Path(mesh.file).name)
    for name, rgba in station.data["material_rgba"].items():
        scene.material(name).rgba = rgba
    return scene.compile()


def main() -> None:
    from mujoco import viewer

    station = load_station()
    model = build_model(station)
    data = mujoco.MjData(model)
    robot = station.spec()
    kinematics = RobotKinematics(robot, model, prefix=PREFIX)
    home = kinematics.to_q(station.home or robot.home)
    kinematics.write_qpos(data.qpos, home)
    for name, angle in zip(robot.joints, home, strict=True):
        data.ctrl[model.actuator(PREFIX + name).id] = angle
    mujoco.mj_forward(model, data)
    viewer.launch(model, data)


if __name__ == "__main__":
    main()
