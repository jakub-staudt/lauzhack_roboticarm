# Linux workbench on ramion-server-01

Source: the private ramion-cloud/monorepo workspace at apps/arm-workbench.
Server source: /home/kwiscion/apps/ramion-workbench/src.
Operator UI at http://100.85.56.14:8780/ (Tailscale required). The controller API uses port 8765 and the viewer uses port 8766.

## One-time host prerequisites

Run on the server as kwiscion. Camera/serial group changes also require a new user-service manager;
reboot once after these commands.

```bash
sudo apt install git curl libgl1 v4l-utils
sudo usermod -aG video,dialout kwiscion
sudo loginctl enable-linger kwiscion
sudo reboot
```

The current host has completed these steps. Both cameras and preview autostart were verified after reboot.
The arm adapter is identified; all six servos responded at about 12 V with no errors.
Live arm feedback and the measured-pose viewer are verified. Manual motion/STOP acceptance remains untested.

## Deploy a release

Use the [team handoff](../../../../docs/TEAM_HANDOFF.md#8-deploy-a-reviewed-release)
or [deployment guide](../../../../docs/deployment.md) from a clean committed monorepo
checkout. `scripts/deploy.sh` packages reviewed source, builds a versioned release,
switches the active link and retains the previous release for rollback.

Do not extract a source tarball over the active `src` link or edit the live release
in place. Service templates require separate installation when changed; the release
script does not overwrite the host environment or stop-state file. Arm mode remains
a separate explicit operator action after deployment.

## Modes and devices

Preview is enabled at boot. It never opens the arm port, reports disconnected/no measured pose,
and rejects motion/control requests. Torque is unknown: preview has not read or changed the servos.
A blank 3D scene is intentional until real measurements exist.
The full arm service has no boot enablement and Restart=no.

Persistent assignments live in ~/.config/ramion/workbench.env:

| Role | Stable device |
|---|---|
| Overhead | /dev/v4l/by-id/usb-Innomaker_Innomaker-U20CAM-720P_SN0001-video-index0 |
| Wrist | /dev/v4l/by-id/usb-Innomaker_Innomaker-U20CAM-1080p-S1_SN0001-video-index0 |
| Arm | /dev/serial/by-id/usb-1a86_USB_Single_Serial_5AAF219949-if00 |

Both cameras currently deliver 640x480. Requested FPS is 15; actual device delivery may differ.
Changes in the Session drawer last only until restart; edit the environment file for persistent roles.
No other process (including uStreamer) should capture these same camera nodes.

## Start arm control only while supervised

First identify the controller with lsusb and ls -l /dev/serial/by-id/. A missing adapter in lsusb is
a USB/power/cable issue to resolve before serial configuration. Do not guess another device.

Set RAMION_ARM_PORT in the environment file, clear the workspace, and have motor power/stop reachable.
The direct backend assumes the arm's existing EEPROM calibration; do not remotely zero/recalibrate it.
Starting arm mode writes measured positions as goals and enables torque; it does not command Home.

Preflight before switching. A failed arm start automatically restores camera-only preview through
OnFailure; it does not clear the stop latch. If the UI reports a latched stop, inspect the arm,
click Reset stop in preview, and only then start arm mode. Starting arm mode before resetting
will fail and return to preview.

```bash
(
set -euo pipefail
set -a
. "$HOME/.config/ramion/workbench.env"
set +a
: "${RAMION_ARM_PORT:?Identify and configure the arm adapter first.}"
test -c "$RAMION_ARM_PORT"
test -r "$RAMION_ARM_PORT"
test -w "$RAMION_ARM_PORT"
systemctl --user start ramion-workbench-arm.service
)
```

Then manually Take control, select a 1-degree step and perform a supervised acceptance test.
STOP is a position hold, not power removal. Service shutdown keeps torque on intentionally.
Do not change torque or disconnect power while an unsupported arm/load could fall.

Return to preview (stops the arm service without releasing torque):

```bash
systemctl --user start ramion-workbench-preview.service
```

## Checks / logs

```bash
systemctl --user status ramion-workbench-preview.service ramion-workbench-arm.service --no-pager
journalctl --user -u ramion-workbench-preview.service -u ramion-workbench-arm.service -n 80 --no-pager
curl -fsS http://100.85.56.14:8765/api/state
ls -l /dev/v4l/by-id/ /dev/serial/by-id/
```

Health: camera status live, age_s stays low, both snapshot URLs return JPEG.
In arm mode also require connected/loop_ok true, advancing sample sequence and no faults.
Do not automate jog/Home/torque for a deployment health check.
