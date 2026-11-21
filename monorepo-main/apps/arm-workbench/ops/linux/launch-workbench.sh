#!/usr/bin/env bash
# Run from the deployed source; preview never opens the robot serial bus.
set -euo pipefail
app_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
mode="${1:---preview}"
bind_ip="${RAMION_BIND_IP:-$(/usr/bin/tailscale ip -4)}"
[[ -n "$bind_ip" && "$bind_ip" != 0.0.0.0 ]] || { echo 'Set a reachable private bind IP.' >&2; exit 2; }
stop_state_file="${RAMION_STOP_STATE_FILE:-${HOME}/.local/state/ramion/emergency-stop.json}"
[[ "$stop_state_file" = /* ]] || { echo 'RAMION_STOP_STATE_FILE must be an absolute path.' >&2; exit 2; }
export RAMION_STOP_STATE_FILE="$stop_state_file"
args=(--listen "$bind_ip:8765" --viewer-port 8766 --no-browser
  --overhead-camera "${RAMION_OVERHEAD_CAMERA:-/dev/v4l/by-id/usb-Innomaker_Innomaker-U20CAM-720P_SN0001-video-index0}"
  --wrist-camera "${RAMION_WRIST_CAMERA:-/dev/v4l/by-id/usb-Innomaker_Innomaker-U20CAM-1080p-S1_SN0001-video-index0}")
observer_camera="${RAMION_OBSERVER_CAMERA-/dev/v4l/by-id/usb-046d_0825_E8445E80-video-index0}"
if [[ -n "$observer_camera" ]]; then
  args+=(--observer-camera "$observer_camera")
fi
case "$mode" in
  --preview) args+=(--preview) ;;
  --arm)
    : "${RAMION_ARM_PORT:?Set RAMION_ARM_PORT to the verified /dev/serial/by-id path.}"
    [[ -c "$RAMION_ARM_PORT" && -r "$RAMION_ARM_PORT" && -w "$RAMION_ARM_PORT" ]] ||
      { echo 'Arm device is absent or inaccessible; check dialout membership.' >&2; exit 2; }
    args+=(--port "$RAMION_ARM_PORT")
    ;;
  *) echo 'Usage: launch-workbench.sh --preview|--arm' >&2; exit 2 ;;
esac
cd "$app_root"
exec "$app_root/.venv/bin/soarm-panel" "${args[@]}"
