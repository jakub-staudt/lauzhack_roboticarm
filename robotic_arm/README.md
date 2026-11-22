# robotic_arm

Turns table-rectangle points `(u, v)` (each 0..1, origin at tag 0) into arm motion on the SO-101, through the
controller's HTTP API (`WorkbenchClient` from `soarm_lab`).

## Setup
```bash
python3 -m venv .venv && .venv/bin/pip install mujoco numpy scipy pyyaml pytest
.venv/bin/pip install --no-deps -e ../monorepo-main/apps/arm-workbench/packages/soarm \
                                -e ../monorepo-main/apps/arm-workbench/packages/soarm_lab -e .
cp config.example.yaml config.yaml   # set controller_url (Tailscale IP, or http://127.0.0.1:8765 via SSH tunnel)
```
Tests: `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 .venv/bin/python -m pytest -q tests`

## 1. Calibrate (do this first, redo if the table or arm moves)
Keep torque ON. Jog the arm to each point from the dashboard (Cartesian or joint controls) so the gripper **tip** is on the marked point, then press Enter in the terminal. The script only reads the arm's state; it never changes torque or sends motion.
```bash
.venv/bin/python scripts/calibrate.py          # 9 points: corners, edge midpoints, centre
```
It prints a residual per point; redo anything over ~8 mm. It writes `calibration.json`.
`u` runs from tag 0 along the first edge, `v` along the second (`robotic_arm/calibration.py:POINTS`).

## 2. Go to a point (rise to 3 cm, glide, lower)
```bash
.venv/bin/python scripts/goto.py 0.5 0.5 --dry-run   # plan only
.venv/bin/python scripts/goto.py 0.5 0.5             # asks for confirmation, then moves
```
Needs torque on and the E-stop clear. Release control in the dashboard first, because the script claims control itself and only one owner is allowed. Keep the STOP button reachable.

## Layout
`calibration.py` homography fit · `planner.py` IK + trajectories (pure) · `executor.py` sends to the controller ·
`config.py` all tunables (heights, gripper values, speeds) · `planner.pick_place` plan exists but is not wired up yet.
