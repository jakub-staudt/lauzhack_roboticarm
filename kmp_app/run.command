#!/bin/zsh
# Run from Terminal (or double-click) to start the app (control + eye tracking windows), so the eye tracker
# (started by the app on the MacBook camera) gets Terminal's camera permission. Needs the simulator running.
cd "$(dirname "$0")"
export JAVA_HOME="${JAVA_HOME:-$(/opt/homebrew/bin/brew --prefix openjdk@17)/libexec/openjdk.jdk/Contents/Home}"
curl -s -m 2 -o /dev/null http://127.0.0.1:8810/api/state || echo "Simulator not running on :8810 (python -m simulator.web.server)"
# --no-daemon: a Gradle daemon started elsewhere would launch the app outside Terminal, without its camera permission
./gradlew --no-daemon :composeApp:run
