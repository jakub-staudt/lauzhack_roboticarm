# RAMION team handoff

Last checked: 2026-11-04. Internal team document; keep in the private repository.

**Start here:** use your own Linux account, GitHub identity and checkout. The running services are separate from development checkouts. Editing code or pulling Git does not deploy the web app.

Quick navigation: [join/develop](#3-join-the-team-and-create-your-checkout), [notebooks](#5-run-notebooks-with-your-own-credentials), [operate](#7-operate-the-arm-and-stop), [deploy/rollback](#8-deploy-a-reviewed-release), [troubleshoot](#9-check-status-and-troubleshoot).

## 1. What is running

- Private source of truth: https://github.com/ramion-cloud/monorepo
- Server: `ramion-server-01` / Tailscale `100.85.56.14`.
- Main app: http://100.85.56.14:8780/ — cameras, 3D measured-pose view, six joint sliders, gripper Open/Close and STOP ROBOT.
- Direct controller API at http://100.85.56.14:8765/api/state. The Viser backend uses port 8766.
- Main-app viewer: `/viewer/` on port `8780`, including its WebSocket. No additional viewer tunnel is needed.
- Board collection: `/map` or `/scan`; manual mapping: `/map/manual`.
- Hardware: one SO-101 arm; overhead, wrist and observer cameras; RTX 3060 with 12 GB VRAM.
- Host: Ubuntu Server 24.04.5 LTS; NVIDIA driver 595.91.07; Python 3.12. Release tooling has been used with uv 0.13.6.
- Current deployment uses Python environments and systemd user services. There is no container orchestration or multi-arm scheduling.

~~~mermaid
flowchart LR
  B["Browser :8780"] --> D["Dashboard UI and proxy"]
  D --> C["Controller :8765"]
  N["Personal notebook kernels"] --> C
  C --> A["SO-101 serial adapter"]
  C --> K["Overhead / wrist / observer cameras"]
  C -. measured pose .-> V["Viser :8766"]
  D --> V
~~~

The 3D view displays measured joint feedback, not a prediction of model behavior.
The observer camera is for people watching; existing VLA observations use overhead and wrist.

The app has no separate user-login layer. Access is currently through the trusted tailnet or an existing SSH relay. One operator/model owns control at a time.

## 2. Where things live

### Source map

Paths below are relative to the monorepo root.

| Change | Source |
| --- | --- |
| Main UI layout, styles and browser controls | `apps/arm-dashboard/src/arm_dashboard/static/panel.html`, `panel.css`, `panel.js` |
| Controller API, ownership, stop latch and model sessions | `apps/arm-workbench/packages/soarm/soarm/panel.py` |
| Non-actuating preview and camera capture | Same directory: `panel_preview.py`, `panel_cameras.py` |
| Measured-pose viewer | Same directory: `panel_viewer.py` |
| Dashboard app (FastAPI): routes, settings, assembly | `apps/arm-dashboard/src/arm_dashboard/` (`api/`, `core/`, `main.py`) |
| Controller/camera proxy | `apps/arm-dashboard/src/arm_dashboard/controller/`, `cameras/` |
| Viewer HTTP/WebSocket relay | `apps/arm-dashboard/src/arm_dashboard/viewer/` |
| Map/scan UI and collector | `apps/arm-dashboard/src/arm_dashboard/static/`, `src/arm_dashboard/scan/` |
| Legacy stdlib proxy (no longer deployed) | `apps/arm-dashboard/legacy/` |
| Four model/setup notebooks | `apps/arm-workbench/notebooks/` |
| Shared notebook clients and adapters | `apps/arm-workbench/packages/soarm_lab/soarm_lab/notebooks/` |
| Workbench service templates and launcher | `apps/arm-workbench/ops/linux/` |
| Package, deploy and rollback | `scripts/deploy.sh` |
| Tests | `tests/` (Node UI), `apps/arm-dashboard/tests/` and each workspace package's `tests/` |

The dashboard serves the operator UI on port 8780. The controller advertises supported operator commands, and the dashboard reports the subset it forwards. Port 8765 serves the direct controller API and redirects its browser entry to port 8780.

### Current server layout

| Purpose | Current path / owner |
| --- | --- |
| Existing canonical development checkout | `/home/kwiscion/src/ramion-monorepo` |
| Workbench active release link | `/home/kwiscion/apps/ramion-workbench/src` — `kwiscion` |
| Workbench releases / previous link | `/home/kwiscion/apps/ramion-workbench/releases/` and `previous` |
| Dashboard active release link | `/home/kwikiel/ramion-dashboard` — `kwikiel` |
| Dashboard releases / previous link | `/home/kwikiel/apps/ramion-dashboard/releases/` and `previous` |
| Host/device configuration | `/home/kwiscion/.config/ramion/workbench.env` |
| Persistent STOP state | `/home/kwiscion/.local/state/ramion/emergency-stop.json` |
| Installed user units | Each service owner's `~/.config/systemd/user/` |
| Scan artifacts | `/home/kwikiel/ramion-scans/` |
| Personal model caches, taught poses and experiment output | `apps/arm-workbench/workspace/` inside that person's checkout |
| Legacy checkout (older Jupyter instances may point here) | `/home/kwiscion/src/ramion-workbench` — migrate notebook work to personal monorepo clones |

The legacy notebooks received the BYO credential fix, but the legacy checkout/public fork is not the source of truth. Do not develop or publish new work from it.

At inspection, workbench release was `83b07a7`, dashboard release `52f661c`, and the installed arm unit included the `c82407f` preview-recovery fix. Documentation and unit changes can be newer than application releases. Check live `.release-commit` files rather than assuming the checkout HEAD is deployed.

### Shared-location decision

Personal checkouts belong in `/home/<user>/src/`. Do not give everyone one writable Git checkout or share the `kwiscion` login.

The [system-wide deployment runbook](deployment.md) moves releases to `/srv/ramion`, state to `/var/lib/ramion`, and configuration to `/etc/ramion`. The paths and user-service commands in this handoff apply before the first cutover and when restoring the old units.

## 3. Join the team and create your checkout

An administrator grants three separate things: a personal SSH account/key, network access, and private GitHub repository access. Normal development/notebook users do not need sudo, `video` or `dialout` access; supported notebooks use the controller API.

On your laptop, replace `alice` with your assigned Linux username:

~~~bash
ssh alice@ramion-server-01
~~~

If the hostname does not resolve, use `100.85.56.14`. For VS Code, use Remote SSH and open your own checkout.

On the server, in **your own account** (`gh` is installed system-wide). Replace the Git name/email placeholders with your identity:

~~~bash
gh auth login --hostname github.com --git-protocol https --web
gh auth setup-git --hostname github.com
mkdir -p "$HOME/src"
git clone https://github.com/ramion-cloud/monorepo.git "$HOME/src/ramion-monorepo"
cd "$HOME/src/ramion-monorepo"
git config user.name 'YOUR NAME'
git config user.email 'YOUR_GITHUB_EMAIL_OR_NOREPLY_ADDRESS'
~~~

Use personal branches and pull requests. Repository Write access does not grant service deployment rights. An operator packages reviewed commits; the runtime itself needs no personal GitHub credential.

Install uv in your own account if missing, using the [official installer](https://docs.astral.sh/uv/getting-started/installation/):

~~~bash
curl -LsSf https://astral.sh/uv/0.13.6/install.sh | sh
export PATH="$HOME/.local/bin:$PATH"
~~~

## 4. Modify and verify code

Start a branch in your own clone:

~~~bash
cd "$HOME/src/ramion-monorepo"
git switch -c my-change
cd apps/arm-workbench
uv sync --locked --group workbench
~~~

Run focused hardware-free checks from the workbench directory:

~~~bash
.venv/bin/python -m pytest packages/soarm/tests \
  packages/soarm_lab/tests/test_notebook_control.py \
  packages/soarm_lab/tests/test_notebook_models.py -m 'not render' -q
.venv/bin/ruff check packages/soarm/soarm/panel.py \
  packages/soarm_lab/soarm_lab/notebooks
~~~

From the repository root, test the proxy and UI. Node.js is needed for the last command:

~~~bash
cd "$HOME/src/ramion-monorepo"
(cd apps/arm-dashboard && uv sync --locked --extra scan && uv run pytest -q)
node --test tests/test_dashboard_ui.cjs
~~~

Use fakes for control changes. Do not launch a second hardware process against the live arm/cameras. The deployed app owns those devices. A code test passing does not validate physical motion or collision avoidance.

Commit source and reviewed dependency locks. Clear notebook outputs and turn motion gates off before committing. Keep keys, authentication profiles, environments, recordings and model weights out of Git. Preserve upstream LICENSE/NOTICE files.

Last implementation validation: 253 controller/notebook tests, 9 executable UI tests and 15 proxy/viewer tests passed. The viewer relay and camera feeds were checked in the browser. New BYO authentication was tested offline; no provider calls were made for that change.

## 5. Run notebooks with your own credentials

In your personal checkout:

~~~bash
cd "$HOME/src/ramion-monorepo/apps/arm-workbench"
bash notebooks/setup.sh all
RAMION_JUPYTER_PORT=8889 bash notebooks/launch.sh
~~~

Use `core`, `vla` or `dream` instead of `all` to install fewer environments. Choose a free server port; existing Jupyter instances may already occupy 8888 or 8889.

From another laptop terminal:

~~~bash
ssh -N -L 8889:127.0.0.1:8889 alice@ramion-server-01
~~~

Open the token URL printed by Jupyter. Keep the main workbench open alongside it.

| Notebook | Purpose |
| --- | --- |
| `00_setup.ipynb` | Camera, GPU and controller checks |
| `01_local_vla.ipynb` | SmolVLA inference benchmark; base checkpoint outputs are not ready-to-execute robot actions |
| `02_codex.ipynb` | Codex selects previously taught, constrained skills |
| `03_dream.ipynb` | Experimental single-arm Dream preview and bounded runs |

Codex and Dream inference send observations to their providers and use the operator's account/key. Keep inference and motion flags off until you deliberately start that workflow.

**Codex:** install the [CLI](https://developers.openai.com/codex/cli/) in your own account if needed. Authenticate a separate file-based profile; do not copy anyone's auth file:

~~~bash
install -d -m 700 "$HOME/.local/state/ramion/codex"
CODEX_HOME="$HOME/.local/state/ramion/codex" codex \
  -c 'cli_auth_credentials_store="file"' login --device-auth
chmod 600 "$HOME/.local/state/ramion/codex/auth.json"
~~~

Sign in as yourself and set `CODEX_PROFILE` in notebook 02 to `~/.local/state/ramion/codex`. The helper refuses the default/inherited profile and does not inherit ambient API credentials. [Device login reference](https://learn.chatgpt.com/docs/auth#login-on-headless-devices).

**Dream:** notebook 03 asks for your API key through a hidden prompt when you enable inference. Do not paste the key into a code cell. The supplied key is used for requests; the SDK still uses its local configuration to resolve service endpoints. The notebook does not fall back to the operator's old `~/.dreamlabs/key`.

After helper/notebook updates, reopen the notebook from disk before saving, then restart its kernel. Already-running kernels retain old imported code. Separate Linux accounts are needed to keep personal credentials private; separate profiles within one shared Unix account are not an access-control boundary.

**Known limitation:** Dream previously folded inward and hit a tracking watchdog. A command-feedback correction was added, but the trajectory/frame problem is not conclusively resolved. Keep execution disabled pending supervised validation; do not relax watchdogs to bypass it.

## 6. Configure the host and cameras

Only the current controller owner/operator edits:

~~~bash
nano /home/kwiscion/.config/ramion/workbench.env
~~~

Current non-secret assignments:

~~~bash
RAMION_BIND_IP=100.85.56.14
RAMION_OVERHEAD_CAMERA=/dev/v4l/by-id/usb-Innomaker_Innomaker-U20CAM-720P_SN0001-video-index0
RAMION_WRIST_CAMERA=/dev/v4l/by-id/usb-Innomaker_Innomaker-U20CAM-1080p-S1_SN0001-video-index0
RAMION_OBSERVER_CAMERA=/dev/v4l/by-id/usb-046d_0825_E8445E80-video-index0
RAMION_ARM_PORT=/dev/serial/by-id/usb-1a86_USB_Single_Serial_5AAF219949-if00
RAMION_STOP_STATE_FILE=/home/kwiscion/.local/state/ramion/emergency-stop.json
~~~

Use stable by-id paths, not `ttyACM0` or camera indices. An empty `RAMION_OBSERVER_CAMERA=` disables the optional observer. Both preview and arm modes must share the same STOP-state path.

After ending the physical session, apply environment changes as `kwiscion`:

~~~bash
systemctl --user start ramion-workbench-preview.service
systemctl --user restart ramion-workbench-preview.service
~~~

The first command leaves arm mode; the restart reloads configuration even if preview was already running. Camera assignments made in the browser are temporary; the env file controls restart behavior.

Changing the server IP also requires updating `~/.config/ramion/dashboard.env` (template `apps/arm-dashboard/deploy/ramion-dashboard.env`), reviewing `scripts/deploy.sh` and notebook `RAMION_WORKBENCH_URL` overrides. Some controller URLs are still fixed in code.

For SSH-only browser access, forward both existing dashboard transports from your laptop:

~~~bash
ssh -N -L 8780:100.85.56.14:8780 -L 8781:100.85.56.14:8780 alice@ramion-server-01
~~~

Open `http://localhost:8780/`. Proxied localhost camera streams use 8781; the viewer uses `/viewer/` on 8780. Existing VPS relay users should follow the [dashboard relay notes](../apps/arm-dashboard/README.md#access-from-another-tailnet); do not launch duplicate forwards.

## 7. Operate the arm and STOP

Before the cutover, controller commands below run **as `kwiscion`**. After the cutover, use `sudo systemctl` with the same unit names.

- Preview starts at boot and never opens/energizes the arm.
- Arm mode is manual and does not automatically restart.
- STOP ROBOT revokes browser/notebook control and attempts to hold a fresh measured pose with torque retained. It blocks further commands until explicit reset.
- Reset does not resume a notebook or trajectory. A new claim/run is required.
- Leaving/restarting connected arm mode causes a persistent restart guard. Reset from preview before reconnecting.
- Failed arm startup restores preview automatically through the installed `OnFailure=` unit setting.
- A software hold cannot guarantee stopping through network, USB or controller failure. Have local physical stop/power control available. Torque removal can let an unsupported arm fall.
- Direct serial tools bypass this API contract; do not run them alongside the controller.

Start/recover preview:

~~~bash
systemctl --user start ramion-workbench-preview.service
~~~

Inspect the physical workspace. If the latch is set, click **Reset stop** in the browser and wait for confirmation. The button is hidden when the latch is already clear. Then start arm mode while supervised:

~~~bash
systemctl --user start ramion-workbench-arm.service
~~~

Starting arm mode sets the measured pose as its initial goal and enables torque; it does not command Home. The 3D model receives live measured feedback once the arm connects.

Never clear the stop by deleting/editing its state file. If the latch is still set, startup will be refused. If the UI is unavailable, restore preview first.

## 8. Deploy a reviewed release

For the system-wide units, follow the [deployment runbook](deployment.md). The workflow below is for the legacy user units.

Use `scripts/deploy.sh` for routine releases. Do not extract tar files over the active release, edit deployed files directly, or use the legacy notebook packaging workflow for routine deployment.

The script requires a clean committed checkout, packages both apps at one Git revision, verifies checksums, builds each environment in its final release directory, switches the active link and checks HTTP health. Failed application health checks attempt to restore the prior release. Switch the dashboard first when migrating UI ownership. During rollback across that migration, restore the workbench first.

Both service owners need uv in their own PATH or at `~/.local/bin/uv`; both user managers and their installed units must be available.

**Current admin workflow — run as `kwiscion` with sudo rights for the dashboard account.** First finish physical operation, release notebook/browser control, support the arm as needed, and let scan collection finish. Select the reviewed revision before packaging; do not blindly pull during an experiment.

~~~bash
(
set -euo pipefail
cd /home/kwiscion/src/ramion-monorepo
git status --short
revision=$(git rev-parse HEAD)
printf 'Packaging reviewed commit: %s\n' "$revision"
incoming="$HOME/apps/ramion-releases/incoming"
dashboard_incoming=/home/kwikiel/apps/ramion-releases/incoming
dashboard_runtime="/run/user/$(id -u kwikiel)"
dashboard_group=$(id -gn kwikiel)

as_dashboard() {
  sudo -H -u kwikiel env \
    XDG_RUNTIME_DIR="$dashboard_runtime" \
    DBUS_SESSION_BUS_ADDRESS="unix:path=$dashboard_runtime/bus" "$@"
}

as_dashboard test -S "$dashboard_runtime/bus"
test "$(as_dashboard systemctl --user show -p LoadState --value ramion-dashboard.service)" = loaded
test "$(systemctl --user show -p LoadState --value ramion-workbench-preview.service)" = loaded
command -v uv >/dev/null || test -x "$HOME/.local/bin/uv"
as_dashboard sh -c 'command -v uv >/dev/null || test -x "$HOME/.local/bin/uv"'
bash scripts/deploy.sh package "$incoming"
as_dashboard mkdir -p "$dashboard_incoming"
dashboard_archive="ramion-arm-dashboard-$revision.tar.gz"
sudo install -m 600 -o kwikiel -g "$dashboard_group" \
  scripts/deploy.sh \
  "$incoming/$dashboard_archive" \
  "$incoming/$dashboard_archive.sha256" \
  "$dashboard_incoming/"

systemctl --user start ramion-workbench-preview.service
as_dashboard bash "$dashboard_incoming/deploy.sh" deploy arm-dashboard \
  "$dashboard_incoming/$dashboard_archive"
bash scripts/deploy.sh deploy arm-workbench \
  "$incoming/ramion-arm-workbench-$revision.tar.gz"
)
~~~

If GitHub authentication is unavailable to the operator, another authorized developer can package the reviewed commit and transfer the two archives, checksums and same-revision deploy script. The deploy subcommand itself needs no GitHub credential.

After deployment, verify the page, all three cameras and the viewer. Arm mode stays off; reset/start separately when ready. If the dashboard deploy fails after the controller deploy succeeded, inspect both revisions and decide whether to roll back the controller.

### Service-template changes are separate

The release script does not install changed systemd templates. For a reviewed controller-unit change, as `kwiscion` while in preview:

~~~bash
(
set -euo pipefail
cd /home/kwiscion/src/ramion-monorepo
systemd-analyze --user verify \
  apps/arm-workbench/ops/linux/ramion-workbench-preview.service \
  apps/arm-workbench/ops/linux/ramion-workbench-arm.service
install -m 644 apps/arm-workbench/ops/linux/ramion-workbench-preview.service \
  "$HOME/.config/systemd/user/"
install -m 644 apps/arm-workbench/ops/linux/ramion-workbench-arm.service \
  "$HOME/.config/systemd/user/"
systemctl --user daemon-reload
)
~~~

Do the analogous dashboard-unit update under `kwikiel` only when its template changed. The FastAPI dashboard additionally needs its settings file; install it before the unit (as `kwikiel`):

~~~bash
(
set -euo pipefail
cd /path/to/reviewed/checkout/apps/arm-dashboard
install -D -m 600 deploy/ramion-dashboard.env "$HOME/.config/ramion/dashboard.env"
systemd-analyze --user verify deploy/ramion-dashboard.service
install -m 644 deploy/ramion-dashboard.service "$HOME/.config/systemd/user/"
systemctl --user daemon-reload
)
~~~

The new unit also starts a legacy dashboard release, so it can be installed before the switch and stays valid after a rollback. Do not enable arm mode at boot.

### Roll back

After ending the physical session, as `kwiscion`:

~~~bash
cd /home/kwiscion/src/ramion-monorepo
systemctl --user start ramion-workbench-preview.service
bash scripts/deploy.sh rollback arm-workbench
~~~

For the dashboard, from the existing admin account:

~~~bash
dashboard_runtime="/run/user/$(id -u kwikiel)"
sudo -H -u kwikiel env \
  XDG_RUNTIME_DIR="$dashboard_runtime" \
  DBUS_SESSION_BUS_ADDRESS="unix:path=$dashboard_runtime/bus" \
  bash /home/kwikiel/apps/ramion-releases/incoming/deploy.sh rollback arm-dashboard
~~~

Keep the previous releases. STOP state, host env and scan artifacts live outside releases and are not rolled back. Notebook edits and kernel state in personal checkouts are also outside application rollback.

## 9. Check status and troubleshoot

As `kwiscion`, these checks do not command motion:

~~~bash
systemctl --user status ramion-workbench-preview.service ramion-workbench-arm.service --no-pager
journalctl --user -u ramion-workbench-arm.service -u ramion-workbench-preview.service -n 60 --no-pager
curl -fsS http://100.85.56.14:8780/api/state
ls -l /dev/serial/by-id/ /dev/v4l/by-id/
cat /home/kwiscion/apps/ramion-workbench/src/.release-commit
nvidia-smi
~~~

Dashboard status/logs/revision from the admin account:

~~~bash
dashboard_runtime="/run/user/$(id -u kwikiel)"
sudo -H -u kwikiel env XDG_RUNTIME_DIR="$dashboard_runtime" \
  DBUS_SESSION_BUS_ADDRESS="unix:path=$dashboard_runtime/bus" \
  systemctl --user status ramion-dashboard.service --no-pager
sudo -H -u kwikiel env XDG_RUNTIME_DIR="$dashboard_runtime" \
  DBUS_SESSION_BUS_ADDRESS="unix:path=$dashboard_runtime/bus" \
  journalctl --user -u ramion-dashboard.service -n 60 --no-pager
sudo -u kwikiel cat /home/kwikiel/ramion-dashboard/.release-commit
~~~

| Symptom | First action |
| --- | --- |
| 502 / controller unavailable | Check controller units and journal; start preview. A latched arm start should automatically fall back to preview. |
| Arm startup refused: stop latched | Reset in preview, wait for acknowledgement, then explicitly start arm mode. |
| 3D says Waiting for arm | Preview has no measured joints. Check connection; no fake pose is displayed. |
| Camera feed missing | Check by-id nodes and env mapping. Do not start another capture process on the same device. |
| USB disconnect / EIO / descriptor errors | Check cable, connector, port and hub; do not guess tty indices or automate arm reconnection. |
| Notebook permission/session error | Check ownership, latch, camera/feedback freshness and kernel version. Reopen/restart after updates. |
| Localhost UI works but cameras do not | Check the separate 8781 SSH forward. |
| Git pull/push fails | Authenticate in your own Linux account and confirm private repository membership. |

Before host maintenance, coordinate with users, stop experiments and save checkpoints. `tmux` survives SSH disconnects, not reboots. SSH/Tailscale and both service owners' lingering are configured; preview is boot-enabled. See [host operations](server-ops.md) for user creation, updates and reboot procedures.

## 10. What must survive a rebuild

Back up the private Git repository, host configuration/service units, persistent STOP state, taught poses and needed captures/datasets. Personal authentication belongs to its owner and must not be exported in the team handoff. Capture robot EEPROM/calibration state through a deliberate calibration workflow; do not remotely zero it as a deployment step.

The older `source-snapshot.json` and imported deployment snapshots are historical evidence. Current releases are identified by `.release-commit` and their archive checksums.

Further detail: [team accounts](team.md), [deployment internals](deployment.md), [control contract](control-and-notebooks.md), [notebook guide](../apps/arm-workbench/notebooks/README.md), [dashboard/relay guide](../apps/arm-dashboard/README.md), [host ops](server-ops.md).
