"""Live read-only view of the arm's measured joints and the tip position (base frame). Ctrl-C to quit.

Sends nothing to the arm; it only polls GET /api/state. Use it while jogging to see the tip height settle.
"""

import argparse
import time

import numpy as np

from soarm_lab.kinematics import RobotKinematics
from soarm_lab.robot import load_robot

from robotic_arm.config import Config
from robotic_arm.executor import make_client

ORDER = ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper")


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--config", default="config.yaml")
    ap.add_argument("--hz", type=float, default=4.0)
    ap.add_argument("--once", action="store_true")
    args = ap.parse_args()
    client = make_client(Config.load(args.config))
    kin = RobotKinematics(load_robot())
    print("pan    lift   elbow  wflex  wroll  grip  |  tip x      y      z [m]   | pitch | torque estop")
    while True:
        try:
            s = client.state()
            j = {k: float(v) for k, v in s["measured"].items()}
            x, y, z = kin.tcp(j)[:3, 3]
            pitch = float(np.degrees(np.arcsin(-kin.tool_axes(j)[0][2])))
            vals = "  ".join(f"{j[k]:6.1f}" for k in ORDER)
            print(f"{vals} | {x:7.3f} {y:7.3f} {z:7.3f} | {pitch:5.1f} | {s.get('torque')}  {s['emergency_stop']['latched']}")
        except Exception as exc:  # keep watching through a dropped request
            print(f"read failed: {exc}")
        if args.once:
            return
        time.sleep(1 / args.hz)


if __name__ == "__main__":
    main()
