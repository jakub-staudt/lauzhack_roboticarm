#!/bin/zsh
# Gaze pick and place with the REAL SO-101 arm in Poland. Run from Terminal (or double-click).
#
#   1. eye tracker on the Logitech camera (started here, so it gets Terminal's camera permission)   :8830
#   2. camera page server in real-arm mode: overhead camera from Poland, rectangle + block detection,
#      pick and place through the arm agent next to the arm                                          :8820
#   3. the app: one full-screen window, the live camera from Poland with the gaze overlay
#
# Needs (from the arm team, not in git):
#   ~/.gazestation/agent_token                 the arm agent's shared secret   (or AGENT_TOKEN_FILE=...)
#   robotic_arm/calibration_poses.json         the real arm's calibration      (or ROBOT_POINTS=...)
# Options: GAZE_CAMERA=<index> (eye camera; default: 1 when a USB camera is plugged in, else 0).
#
# SAFETY: this moves the physical arm. Someone must watch it (the camera page or the RAMION dashboard) and keep
# Stop in reach: Esc in the app, or look at the red STOP box while the arm moves.
cd "$(dirname "$0")/.."
ROOT="$(pwd)"
export JAVA_HOME="${JAVA_HOME:-$(/opt/homebrew/bin/brew --prefix openjdk@17)/libexec/openjdk.jdk/Contents/Home}"
TOKEN="${AGENT_TOKEN_FILE:-$HOME/.gazestation/agent_token}"
POINTS="${ROBOT_POINTS:-$ROOT/robotic_arm/calibration_poses.json}"
AGENT="${ARM_AGENT_URL:-http://100.85.56.14:8790}"
OVERHEAD="${OVERHEAD_CAMERA:-http://100.85.56.14:8780/cameras/overhead.mjpg?fps=10&profile=preview}"

missing=0
[ -s "$TOKEN" ]  || { echo "Missing the arm agent token: $TOKEN (ask the arm team)"; missing=1; }
[ -s "$POINTS" ] || { echo "Missing the arm calibration: $POINTS (ask the arm team)"; missing=1; }
[ -x .venv/bin/python ] || { echo "Missing .venv (python3.11 -m venv .venv && .venv/bin/pip install -r requirements.txt)"; missing=1; }
[ $missing = 0 ] || exit 1
curl -s -m 5 -o /dev/null "http://100.85.56.14:8780/api/state" || { echo "The RAMION server (100.85.56.14) does not answer: is Tailscale on?"; exit 1; }

if [ -z "$GAZE_CAMERA" ]; then
  if system_profiler SPCameraDataType 2>/dev/null | grep -E "^ {4}\S" | grep -qv "MacBook"; then GAZE_CAMERA=1; else GAZE_CAMERA=0; fi
fi
echo "Eye camera: index $GAZE_CAMERA ($(system_profiler SPCameraDataType 2>/dev/null | grep -E '^ {4}\S' | sed 's/^ *//;s/:$//' | tr '\n' ',' | sed 's/,$//'))"

pids=()
cleanup() { kill "${pids[@]}" 2>/dev/null; }
trap cleanup EXIT INT TERM

.venv/bin/python -m gaze_service.server --camera "$GAZE_CAMERA" --port 8830 &
pids+=($!)
(cd lauzkach_eye_robot_control && exec ../.venv/bin/python -m lauzkach_eye_robot_control.server \
    --real-agent "$AGENT" --agent-token-file "$TOKEN" --robot-points "$POINTS" --camera "$OVERHEAD") &
pids+=($!)

echo
echo "  *** REAL ARM: keep Stop ready (Esc in the app, or look at STOP while the arm moves). ***"
echo "  Supervisor view: http://127.0.0.1:8820  (the team's camera page: same server, also has Stop)"
echo
sleep 4
GAZE_ARM=real GAZE_ARM_URL=http://127.0.0.1:8820 GAZE_SERVICE_URL=http://127.0.0.1:8830 ./kmp_app/gradlew -p kmp_app --no-daemon :composeApp:run
