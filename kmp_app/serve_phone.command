#!/bin/zsh
# Laptop side of the Android app: runs the simulator and the eye tracker for the phone over Tailscale.
# They listen on all network interfaces (the macOS Tailscale app does not let programs bind to its 100.x address),
# so the phone reaches them through Tailscale; devices on the same Wi-Fi could too. macOS may ask to allow
# incoming connections for Python: click Allow.
# Run from Terminal (or double-click), keep the window open, and type the address it prints into the phone app.
cd "$(dirname "$0")/.."
TS=$(command -v tailscale || echo /Applications/Tailscale.app/Contents/MacOS/Tailscale)
IP=$("$TS" ip -4 2>/dev/null | head -1)
if [ -z "$IP" ]; then echo "Tailscale is not running on this laptop: start it and sign in, then run this again."; exit 1; fi

.venv-sim/bin/python -m simulator.web.server --host 0.0.0.0 --port 8810 &
SIM=$!
.venv/bin/python -m gaze_service.server --phone --host 0.0.0.0 --port 8820 &
GAZE=$!
trap 'kill $SIM $GAZE 2>/dev/null' EXIT INT TERM

echo
echo "  ==============================================="
echo "   In the phone app, enter this laptop address:"
echo "       $IP"
echo "  ==============================================="
echo "  (simulator :8810, eye tracker :8820; Ctrl+C here stops both)"
echo
wait
