#!/bin/zsh
# Run from Terminal (or double-click) to start the eye tracker and the app (control + eye tracking windows).
# The eye tracker is started by this shell, not by the app, so it gets Terminal's camera permission directly.
# Needs the simulator running (python -m simulator.web.server).
cd "$(dirname "$0")"
ROOT="$(cd .. && pwd)"
export JAVA_HOME="${JAVA_HOME:-$(/opt/homebrew/bin/brew --prefix openjdk@17)/libexec/openjdk.jdk/Contents/Home}"
curl -s -m 2 -o /dev/null http://127.0.0.1:8810/api/state || echo "Simulator not running on :8810 (python -m simulator.web.server)"

if ! curl -s -m 1 -o /dev/null http://127.0.0.1:8820/api/gaze; then
  echo "Starting the eye tracker on camera ${GAZE_CAMERA:-0} (MacBook camera)..."
  (cd "$ROOT" && exec .venv/bin/python -m gaze_service.server --camera "${GAZE_CAMERA:-0}") &
  TRACKER=$!
  trap 'kill $TRACKER 2>/dev/null' EXIT INT TERM
  sleep 3
fi

# --no-daemon: a Gradle daemon started elsewhere would launch the app outside Terminal
./gradlew --no-daemon :composeApp:run
