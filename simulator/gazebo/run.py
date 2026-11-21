"""Open the gaze station in Gazebo: build the world from the MuJoCo model, start gz sim, move the arm to its home pose.

    python3 -m simulator.gazebo.run              # with the Gazebo GUI
    python3 -m simulator.gazebo.run --headless   # physics and cameras only (no window)

Run it with the system python3 (it has the Gazebo bindings). The world is built by the robotic_arm venv, which has MuJoCo.
"""

from __future__ import annotations

import argparse
import subprocess
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
VENV_PYTHON = ROOT / "robotic_arm" / ".venv" / "bin" / "python"


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--headless", action="store_true", help="no GUI window")
    ap.add_argument("--no-build", action="store_true", help="reuse the existing station.sdf")
    args = ap.parse_args()

    if not args.no_build:
        subprocess.run([str(VENV_PYTHON), "-m", "simulator.gazebo.build_world"], cwd=ROOT, check=True)
    cmd = ["gz", "sim", "-r", "-v", "1", str(HERE / "station.sdf")]
    if args.headless:
        cmd.insert(2, "-s")
    gz = subprocess.Popen(cmd)
    try:
        sys.path.insert(0, str(ROOT))
        from simulator.gazebo.client import GazeboStation

        station = GazeboStation()
        station.wait_ready(timeout=90)
        station.go_home()
        print("home pose reached" if station.wait_until_reached(tol=0.05) else "home pose not reached (check the arm)")
        gz.wait()
    except KeyboardInterrupt:
        pass
    finally:
        gz.terminate()


if __name__ == "__main__":
    main()
