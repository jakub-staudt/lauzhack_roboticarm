"""Gripper opening <-> fingertip gap, from the arm's model (read-only; reads GET /api/state, sends nothing).

    python scripts/gripper_gap.py            # gap for the arm's current gripper value
    python scripts/gripper_gap.py --watch    # print every second, with the smallest value seen (grasp a block by hand and read the width)
    python scripts/gripper_gap.py 35         # gap for a given gripper value
"""
import argparse
import json
import sys
import time
import urllib.request

import mujoco
import numpy as np
from soarm_lab.kinematics import RobotKinematics
from soarm_lab.robot import load_robot

robot = load_robot()
kin = RobotKinematics(robot)
model, data = kin.model, mujoco.MjData(kin.model)
TIPS = list(robot.fingertips)
HOME = {"shoulder_pan": 0.0, "shoulder_lift": -23.3, "elbow_flex": 46.4, "wrist_flex": 3.3, "wrist_roll": 0.0}


def gap_mm(gripper: float) -> float:
    """Distance between the two fingertip contact spheres [mm] for a gripper value (0..100)."""
    kin.write_qpos(data.qpos, kin.to_q({**HOME, "gripper": gripper}))
    mujoco.mj_forward(model, data)
    names = {mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, i): i for i in range(model.ngeom)}
    pts = []
    for tip in TIPS:
        gid = next(i for n, i in names.items() if n and n.endswith(tip))
        pts.append(data.geom_xpos[gid].copy())
    return float(np.linalg.norm(pts[0] - pts[1]) * 1000)


def read_gripper(url: str) -> float:
    with urllib.request.urlopen(url.rstrip("/") + "/api/state", timeout=6) as r:
        return float(json.loads(r.read())["measured"]["gripper"])


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("value", nargs="?", type=float)
    ap.add_argument("--url", default="http://100.85.56.14:8780")
    ap.add_argument("--watch", action="store_true")
    a = ap.parse_args()
    if a.value is not None:
        print(f"gripper {a.value:.1f} -> fingertip gap {gap_mm(a.value):.1f} mm")
        sys.exit()
    low = None
    while True:
        g = read_gripper(a.url)
        low = g if low is None else min(low, g)
        print(f"{time.strftime('%H:%M:%S')} gripper {g:5.1f} -> gap {gap_mm(g):5.1f} mm   (lowest seen {low:.1f} -> {gap_mm(low):.1f} mm)", flush=True)
        if not a.watch:
            break
        time.sleep(1.0)
