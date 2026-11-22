"""Record the arm at the rectangle's corners, edge midpoints and centre, then fit and save calibration.json.

Torque stays on. Jog the arm with the dashboard so the gripper TIP sits on each marked point, then press Enter
here. This script only reads GET /api/state; it sends no motion and never changes torque.
Use --offline poses.json to refit from a saved list instead of talking to the arm.
"""

import argparse
import json
import sys
from pathlib import Path

from soarm_lab.kinematics import RobotKinematics
from soarm_lab.robot import load_robot

from robotic_arm.calibration import POINTS, fit_calibration
from robotic_arm.config import Config
from robotic_arm.executor import make_client


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--config", default="config.yaml")
    ap.add_argument("--offline", help="JSON list of {name,u,v,joints} instead of reading the arm")
    ap.add_argument("--save-poses", default="calibration_poses.json")
    ap.add_argument("--corners-only", action="store_true", help="only the 4 corners (faster, less accurate)")
    args = ap.parse_args()
    cfg = Config.load(args.config)
    kin = RobotKinematics(load_robot())

    if args.offline:
        recorded = json.loads(Path(args.offline).read_text())
    else:
        client = make_client(cfg)
        recorded = []
        names = [n for n in POINTS if not args.corners_only or n.startswith("corner")]
        for name in names:
            u, v = POINTS[name]
            input(f"Jog the gripper tip onto {name} (u={u}, v={v}) in the dashboard, then press Enter... ")
            state = client.state()
            joints = {k: float(x) for k, x in state["measured"].items()}
            recorded.append({"name": name, "u": u, "v": v, "joints": joints})
            tip = kin.tcp(joints)[:3, 3]
            print(f"  recorded; tip at x={tip[0]:.3f} y={tip[1]:.3f} z={tip[2]:.3f} m")
        Path(args.save_poses).write_text(json.dumps(recorded, indent=2))

    cal = fit_calibration(kin, recorded)
    cal.save(cfg.calibration_file)
    print(f"\nz_table = {cal.z_table:.4f} m   residual: max {cal.residual_max_m*1000:.1f} mm, "
          f"rms {cal.residual_rms_m*1000:.1f} mm")
    for s in cal.samples:
        print(f"  {s['name']:10s} residual {s['residual_m']*1000:5.1f} mm   z {s['xyz'][2]:.3f}")
    zs = [s["xyz"][2] for s in cal.samples]
    print(f"table-height spread: {(max(zs) - min(zs))*1000:.1f} mm (large = tilted table or sloppy placement)")
    if cal.residual_max_m > 0.008:
        print("WARNING: worst residual > 8 mm. Redo the worst points.")
    print(f"saved {cfg.calibration_file}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
