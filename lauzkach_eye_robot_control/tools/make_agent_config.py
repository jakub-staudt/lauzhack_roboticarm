"""Write the small config the arm agent needs (the calibrated area) from the recorded calibration points.

    python tools/make_agent_config.py ../robotic_arm/calibration_poses.json agent_config.json
"""
import json
import sys

from lauzkach_eye_robot_control.robot_map import RobotMap

rm = RobotMap.load(sys.argv[1])
json.dump({"points_xy": [[p["x"], p["y"]] for p in rm.points], "z_table": rm.z_table}, open(sys.argv[2], "w"), indent=1)
print(f"wrote {sys.argv[2]}: {len(rm.points)} points, z_table {rm.z_table * 1000:.1f} mm")
