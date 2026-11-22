#!/usr/bin/env bash
# Build a small tarball of just what the arm agent needs (no deploy, no network): arm_agent + realarm + the arm's own click-move code.
set -euo pipefail
here="$(cd "$(dirname "$0")/.." && pwd)"; root="$(dirname "$here")"
out="${1:-/tmp/arm_agent_bundle.tgz}"; tmp="$(mktemp -d)"
mkdir -p "$tmp/arm_agent/lauzkach_eye_robot_control" "$tmp/arm_agent/robotic_arm"
touch "$tmp/arm_agent/lauzkach_eye_robot_control/__init__.py"
cp "$here/grasp_profiles.json" "$here/park_pose.json" "$tmp/arm_agent/"
cp "$here/lauzkach_eye_robot_control/arm_agent.py" "$here/lauzkach_eye_robot_control/realarm.py" "$tmp/arm_agent/lauzkach_eye_robot_control/"
cp "$root/robotic_arm/robotic_arm/"{__init__,clickmove,planner,calibration,config}.py "$tmp/arm_agent/robotic_arm/"
pk="$root/monorepo-main/apps/arm-workbench/packages"
mkdir -p "$tmp/arm_agent/soarm" "$tmp/arm_agent/soarm_lab"
cp "$pk/soarm/soarm/"{__init__,settings}.py "$tmp/arm_agent/soarm/"; cp -r "$pk/soarm/soarm/robot" "$tmp/arm_agent/soarm/robot"
cp "$pk/soarm_lab/soarm_lab/"{__init__,robot,kinematics,paths}.py "$tmp/arm_agent/soarm_lab/"
find "$tmp" -name __pycache__ -prune -exec rm -rf {} +
python3 "$here/tools/make_agent_config.py" "$root/robotic_arm/calibration_poses.json" "$tmp/arm_agent/agent_config.json" 2>/dev/null \
  || (cd "$here" && PYTHONPATH=. python3 tools/make_agent_config.py "$root/robotic_arm/calibration_poses.json" "$tmp/arm_agent/agent_config.json")
tar -C "$tmp" -czf "$out" arm_agent && echo "bundle: $out ($(du -h "$out" | cut -f1))"; tar -tzf "$out" | head -30
