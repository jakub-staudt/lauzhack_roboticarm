"""Move the tip to a table point: rise to 3 cm, glide, lower. Use --dry-run first.

  python scripts/goto.py 0.5 0.5            # centre of the rectangle
  python scripts/goto.py 0.5 0.5 --dry-run  # plan and print, send nothing
"""

import argparse
import sys

from soarm_lab.kinematics import RobotKinematics
from soarm_lab.robot import load_robot

from robotic_arm.calibration import Calibration
from robotic_arm.config import Config
from robotic_arm.executor import make_client, run_plan
from robotic_arm.planner import PlanError, Planner


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("u", type=float)
    ap.add_argument("v", type=float)
    ap.add_argument("--config", default="config.yaml")
    ap.add_argument("--dry-run", action="store_true")
    args = ap.parse_args()
    cfg = Config.load(args.config)
    cfg.dry_run = cfg.dry_run or args.dry_run
    kin = RobotKinematics(load_robot())
    planner = Planner(kin, Calibration.load(cfg.calibration_file), cfg)
    client = make_client(cfg)
    state = client.state()
    start = {k: float(x) for k, x in state["measured"].items()}
    try:
        plan = planner.goto(start, (args.u, args.v))
    except PlanError as exc:
        print(f"refused: {exc}")
        return 1
    x, y = planner.cal.xy(args.u, args.v)
    print(f"target ({args.u}, {args.v}) -> base frame x={x:.3f} y={y:.3f} m, pitch {plan.pitch_deg['dest']:g} deg")
    print(f"{len(plan.rows)} steps, {plan.duration_s(cfg.hz):.1f} s: " +
          ", ".join(f"{s.name}={s.end - s.start}" for s in plan.segments))
    if cfg.dry_run:
        print("dry run: nothing sent")
        return 0
    input("Arm is supervised and the STOP button is within reach? Press Enter to move (Ctrl-C to abort)... ")
    run_plan(client, plan, cfg)
    print("done")
    return 0


if __name__ == "__main__":
    sys.exit(main())
