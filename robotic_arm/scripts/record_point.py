"""Record the arm's current measured pose as one calibration point (read-only; sends nothing to the arm).

  record_point.py corner_00        # add/replace this point in calibration_poses.json
  record_point.py --list           # show what is recorded
  record_point.py --reset          # start over
Then fit with:  calibrate.py --offline calibration_poses.json
"""

import argparse
import json
import time

import numpy as np
from pathlib import Path

from soarm_lab.kinematics import RobotKinematics
from soarm_lab.robot import load_robot

from robotic_arm.calibration import POINTS
from robotic_arm.config import Config
from robotic_arm.executor import make_client

FILE = Path("calibration_poses.json")


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("name", nargs="?", choices=list(POINTS))
    ap.add_argument("--config", default="config.yaml")
    ap.add_argument("--list", action="store_true")
    ap.add_argument("--reset", action="store_true")
    args = ap.parse_args()
    recorded = json.loads(FILE.read_text()) if FILE.exists() else []
    if args.reset:
        FILE.write_text("[]")
        print("reset")
        return
    kin = RobotKinematics(load_robot())
    if args.list or not args.name:
        for r in recorded:
            tip = kin.tcp(r["joints"])[:3, 3]
            print(f"{r['name']:10s} u={r['u']} v={r['v']}  tip x={tip[0]:.3f} y={tip[1]:.3f} z={tip[2]:.3f}")
        return
    client = make_client(Config.load(args.config))
    # Take several samples and require the arm to be still, so a point is never recorded mid-motion.
    samples = []
    for _ in range(5):
        samples.append({k: float(v) for k, v in client.state()["measured"].items()})
        time.sleep(0.3)
    spread = max(max(s[j] for s in samples) - min(s[j] for s in samples) for j in samples[0])
    if spread > 0.5:
        raise SystemExit(f"Arm is still moving (joint spread {spread:.2f} deg). Not recorded; try again.")
    joints = {j: sum(s[j] for s in samples) / len(samples) for j in samples[0]}
    u, v = POINTS[args.name]
    recorded = [r for r in recorded if r["name"] != args.name]
    recorded.append({"name": args.name, "u": u, "v": v, "joints": joints,
                     "xyz_m": [round(float(c), 4) for c in kin.tcp(joints)[:3, 3]]})  # tip position, arm base frame
    FILE.write_text(json.dumps(recorded, indent=2))
    tip = kin.tcp(joints)[:3, 3]
    pitch = float(np.degrees(np.arcsin(-kin.tool_axes(joints)[0][2])))
    others = [float(np.degrees(np.arcsin(-kin.tool_axes(r['joints'])[0][2]))) for r in recorded if r['name'] != args.name]
    print(f"recorded {args.name} (u={u}, v={v}): tip x={tip[0]:.3f} y={tip[1]:.3f} z={tip[2]:.3f}  pitch={pitch:.0f}deg  "
          f"[{len(recorded)} points saved]")
    if others and abs(pitch - others[0]) > 5:
        print(f"WARNING: tilt differs from the first point ({others[0]:.0f} deg) by {abs(pitch - others[0]):.0f} deg. "
              "Re-set the wrist tilt and record this point again.")


if __name__ == "__main__":
    main()
