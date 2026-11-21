# SO-101 eye-tracking test station

The station uses the arm and physics from the existing SO-101 MuJoCo workbench.
It has a 900 × 600 mm white surface with two adjacent 300 × 300 mm squares,
measured between the line centers. The red, blue, and yellow 20 mm cubes can be
picked up directly with the gripper.

## Run the simulator

From the repository root, with [uv](https://docs.astral.sh/uv/) installed:

```sh
uv run --project monorepo-main/apps/arm-workbench python -m simulator.run
```

In the MuJoCo viewer, use the **Rendering** camera menu to switch between
`overhead` and `robot/wrist_cam`. Use the **Control** sliders to move the arm
and gripper.

For programmatic access, import `load_station` and `build_model` from
`simulator.run`. Pass the station returned by `load_station()` to
`build_model()`. The station's `cameras` property exposes the overhead
camera's intrinsics and pose in the robot base frame.

## Open the CAD model

Open [gaze_station.scad](gaze_station.scad) in OpenSCAD, press **F5** to preview
in color, and choose **View → View All** to fit the model. It uses millimetres
with tabletop Z=0, models the board as a 15 mm plate, and reads the arm meshes
from this checkout.

## Test

```sh
uv run --project monorepo-main/apps/arm-workbench python -m pytest -q simulator/test_station.py
```
