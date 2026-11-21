# Deploy RAMION on ramion-server-01

The system-wide deployment uses `/srv/ramion` and two dedicated service accounts.
Use the [legacy deployment](#legacy-user-services) until the first cutover. Each
maintainer packages a reviewed commit from their own clean Git checkout and uses
their own SSH login with sudo to install or switch releases.

| Path | Purpose |
| --- | --- |
| `/srv/ramion/releases/<app>/` | Built releases and their Python environments |
| `/srv/ramion/current/<app>` | Active release symlink |
| `/etc/ramion/` | Host settings and device assignments |
| `/var/lib/ramion/workbench/emergency-stop.json` | STOP state shared by preview and arm mode |
| `/var/lib/ramion/scans/` | Dashboard captures |

## Prepare the host and releases

On the server, create the service accounts once. The workbench units grant camera
access to preview and arm mode, and serial access to arm mode. Install the unit
files without starting or enabling them.

```bash
sudo useradd --system --user-group --no-create-home --shell /usr/sbin/nologin ramion-workbench
sudo useradd --system --user-group --no-create-home --shell /usr/sbin/nologin ramion-dashboard
sudo install -d -m 0755 /etc/ramion
sudo install -m 0600 /home/kwiscion/.config/ramion/workbench.env /etc/ramion/workbench.env
sudo install -m 0600 /home/kwikiel/.config/ramion/dashboard.env /etc/ramion/dashboard.env
sudoedit /etc/ramion/workbench.env /etc/ramion/dashboard.env
sudo install -m 0644 ops/systemd/ramion-{dashboard,workbench-preview,workbench-arm}.service /etc/systemd/system/
sudo systemctl daemon-reload
```

Keep the current private endpoints, device assignments, and Cartesian settings.
Remove the old home-directory STOP and scan overrides from the copied settings.
The new units enforce the state paths in the table above. The service endpoints
remain dashboard `8780`, controller `8765`, and viewer `8766` at `100.85.56.14`.
Keep the legacy user units and release links for the first cutover rollback.
The existing VPS relay units continue forwarding to the same endpoints.

From the clean, reviewed checkout, package and build both apps before stopping
anything. `prepare` prints the absolute release directory for `activate`. The
build uses `/usr/bin/python3.12` and uv 0.13.6 or newer from the system PATH
or the invoking maintainer's `~/.local/bin/uv`. From a root shell, set
`RAMION_BUILD_USER` to the unprivileged account that should build.

```bash
revision=$(git rev-parse HEAD)
incoming="$HOME/apps/ramion-releases/incoming"
bash scripts/deploy.sh package "$incoming"
dashboard_archive="$incoming/ramion-arm-dashboard-$revision.tar.gz"
workbench_archive="$incoming/ramion-arm-workbench-$revision.tar.gz"
dashboard_release=$(sudo bash scripts/deploy.sh --system prepare arm-dashboard "$dashboard_archive")
workbench_release=$(sudo bash scripts/deploy.sh --system prepare arm-workbench "$workbench_archive")
```

With the workbench in disconnected preview, run the prepared dashboard alongside
the old one on `8790` with independent scan storage. Use the same shell as above.

```bash
sudo install -d -o ramion-dashboard -g ramion-dashboard -m 0750 /var/lib/ramion/staging-scans
sudo systemd-run --unit=ramion-dashboard-staging --uid=ramion-dashboard \
  --working-directory="$dashboard_release" \
  --setenv=RAMION_DASHBOARD_SERVER__HOST=100.85.56.14 \
  --setenv=RAMION_DASHBOARD_SERVER__PORT=8790 \
  --setenv=RAMION_DASHBOARD_CONTROLLER__BASE_URL=http://100.85.56.14:8765 \
  --setenv=RAMION_DASHBOARD_VIEWER__BASE_URL=http://100.85.56.14:8766 \
  --setenv=RAMION_DASHBOARD_SCAN__STORAGE_DIR=/var/lib/ramion/staging-scans \
  "$dashboard_release/.venv/bin/arm-dashboard"
curl -fsS --retry 10 --retry-connrefused --retry-delay 1 http://100.85.56.14:8790/healthz
python3.12 apps/arm-dashboard/tests/manual/live_smoke.py \
  --dashboard http://100.85.56.14:8790 --controller http://100.85.56.14:8765
sudo systemctl stop ramion-dashboard-staging.service
```

## First cutover

Work with an operator at the arm. End the arm session, return to disconnected
preview, and let production scans finish. Confirm `/api/state` on port `8765`
reports `preview_only=true`, `connected=false`, and `port=null`. Coordinate a pause
in new scans and arm starts until the cutover checks pass.

Use the following helper to reach the old owners' user managers from your login.
The old units must release their ports and cameras before activating the new
units. The legacy arm unit is manual. Stop it explicitly and leave it disabled.

```bash
legacy_unit() {
  local owner="$1" runtime
  shift
  runtime="/run/user/$(id -u "$owner")"
  sudo -H -u "$owner" env XDG_RUNTIME_DIR="$runtime" \
    DBUS_SESSION_BUS_ADDRESS="unix:path=$runtime/bus" systemctl --user "$@"
}
```

Stop the old writers and copy the latest STOP file and scans. Preserve
the entire STOP file, including its generation and hardware-session marker.
The new workbench requires a nonempty STOP file. If the expected file is absent,
check the old unit's settings before continuing.
Use these copy commands for the initial move, when the old paths contain the
authoritative state. If activation fails after copying state, either resume
activation with the copied state or follow the first-cutover rollback below.
A system release that is already selected can be activated again after stopping
its legacy service.

```bash
(
  set -e
  sudo test -s /home/kwiscion/.local/state/ramion/emergency-stop.json
  if sudo test -e /var/lib/ramion/workbench/emergency-stop.json ||
      sudo test -L /var/lib/ramion/workbench/emergency-stop.json; then
    echo 'Shared STOP state already exists. Determine which copy is current before retrying.' >&2
    exit 1
  fi
  legacy_unit kwiscion stop ramion-workbench-arm.service
  legacy_unit kwiscion disable --now ramion-workbench-preview.service
  legacy_unit kwikiel disable --now ramion-dashboard.service
  sudo install -d -o ramion-workbench -g ramion-workbench -m 0700 /var/lib/ramion/workbench
  sudo install -d -o ramion-dashboard -g ramion-dashboard -m 0750 /var/lib/ramion/scans
  sudo install -o ramion-workbench -g ramion-workbench -m 0600 \
    /home/kwiscion/.local/state/ramion/emergency-stop.json \
    /var/lib/ramion/workbench/emergency-stop.json
  sudo cp -a /home/kwikiel/ramion-scans/. /var/lib/ramion/scans/
  sudo chown -hR ramion-dashboard:ramion-dashboard /var/lib/ramion/scans
  sudo chmod 0700 /var/lib/ramion/scans
  sudo bash scripts/deploy.sh --system activate arm-dashboard "$dashboard_release"
  sudo bash scripts/deploy.sh --system activate arm-workbench "$workbench_release"
)
```

Confirm both system units are active and their `.release-commit` markers match
the planned SHA. Check the dashboard UI, direct and proxied preview state,
camera frames, viewer WebSocket, and a scan that produces a fresh artifact. The
dashboard's `/healthz` checks its own process. Use the smoke command above with
port `8780` for the dashboard, and verify the notebook API and VPS relay from
their usual clients. Once these checks pass, enable the dashboard and preview for
boot and verify the next planned boot returns to preview.

```bash
(
  set -e
  sudo systemctl is-active ramion-dashboard.service
  sudo systemctl is-active ramion-workbench-preview.service
  test "$(sudo cat /srv/ramion/current/arm-dashboard/.release-commit)" = "$revision"
  test "$(sudo cat /srv/ramion/current/arm-workbench/.release-commit)" = "$revision"
  sudo systemctl enable ramion-dashboard.service ramion-workbench-preview.service
)
```

When an operator is ready to reconnect, follow the [arm and STOP procedure](TEAM_HANDOFF.md#7-operate-the-arm-and-stop). Start arm mode explicitly with
`sudo systemctl start ramion-workbench-arm.service`. Return to disconnected
preview with `sudo systemctl start ramion-workbench-preview.service`.

## Routine releases and rollback

Package from a clean, reviewed checkout as above. After collection finishes and
the workbench is in disconnected preview, deploy the app you changed. If
changing both, deploy the dashboard first.

```bash
sudo bash scripts/deploy.sh --system deploy arm-dashboard "$dashboard_archive"
sudo bash scripts/deploy.sh --system deploy arm-workbench "$workbench_archive"
```

These commands build and activate in one step. A failed health check restores
the previous release if one exists. To roll back explicitly, use the commands
below for the affected apps. Persistent state stays in `/var/lib/ramion`.

```bash
sudo bash scripts/deploy.sh --system rollback arm-workbench
sudo bash scripts/deploy.sh --system rollback arm-dashboard
```

If the first cutover fails, the previous apps are still under their old user
managers. End any arm session and wait for scans to finish. Stop the new units
before transferring state.

```bash
(
  set -e
  sudo systemctl stop ramion-workbench-arm.service
  sudo systemctl disable --now ramion-workbench-preview.service ramion-dashboard.service
)
```

If the state was copied into `/var/lib/ramion` during cutover, transfer it back
before restarting the old units. Keep both sets of writers stopped until the
transfer succeeds. If the new directory was never populated, the old state is
still authoritative.

```bash
(
  set -e
  sudo install -o kwiscion -g "$(id -gn kwiscion)" -m 0600 \
    /var/lib/ramion/workbench/emergency-stop.json \
    /home/kwiscion/.local/state/ramion/emergency-stop.json
  sudo cp -a /var/lib/ramion/scans/. /home/kwikiel/ramion-scans/
  sudo chown -hR "kwikiel:$(id -gn kwikiel)" /home/kwikiel/ramion-scans
)
```

Start the old preview and dashboard once the latest state is in their paths.
Define the `legacy_unit` helper above if you opened a new shell.

```bash
(
  set -e
  legacy_unit kwiscion enable --now ramion-workbench-preview.service
  legacy_unit kwikiel enable --now ramion-dashboard.service
)
```

Check the direct controller reports disconnected preview, then verify the
dashboard and cameras. Leave arm startup to the operator.

## Legacy user services

Select a reviewed commit in the private monorepo. The server's workbench checkout
is `/home/kwiscion/src/ramion-monorepo`. Deployment builds from committed source
and uses the existing user services and their original paths.

| App | Service account | Service | Live path |
| --- | --- | --- | --- |
| `arm-workbench` | `kwiscion` | `ramion-workbench-preview.service` | `~/apps/ramion-workbench/src` |
| `arm-dashboard` | `kwikiel` | `ramion-dashboard.service` | `~/ramion-dashboard` |

These are the service paths before the system-wide cutover. Keep day-to-day development in personal
checkouts under `~/src/ramion-monorepo`. Existing service owners deploy reviewed
release archives using the procedure below. GitHub repository write access does
not grant deployment rights.

End the supervised arm session and return to preview. Let any dashboard camera
collection finish. Run release operations under each app's service account.
The host needs Bash, Git, curl, GNU coreutils, `flock`, Python 3 and uv. Use uv
0.13.6 or a compatible newer version for the workbench and dashboard lockfiles. The
workbench expects its existing `~/.config/ramion/workbench.env` and camera/serial
access. The dashboard expects `~/.config/ramion/dashboard.env` and its user unit from
`apps/arm-dashboard/deploy/`; install both once before the first FastAPI dashboard
release (see the [dashboard README](../apps/arm-dashboard/README.md#deploy-on-the-server)).

### Package and switch

Deploy the dashboard first when moving UI ownership. It works with the older
controller. Deploy the workbench next to enable its redirect to the dashboard.

From the clean committed monorepo checkout:

```bash
revision=$(git rev-parse HEAD)
incoming="$HOME/apps/ramion-releases/incoming"
bash scripts/deploy.sh package "$incoming"
```

Packaging creates a separate archive and SHA-256 file for each app at the same
commit. Transfer the archive and its `.sha256` file together when packaging on
another machine or deploying under another account. Use an archive from the
reviewed checkout. SHA-256 detects transfer corruption.

Run this from a checkout of the same commit as `kwikiel`, with `incoming` set to
the directory containing that user's dashboard archive:

```bash
revision=$(git rev-parse HEAD)
incoming="$HOME/apps/ramion-releases/incoming"
bash scripts/deploy.sh deploy arm-dashboard \
  "$incoming/ramion-arm-dashboard-$revision.tar.gz"
```

Then switch the workbench as its service owner, using that account’s incoming
directory.

```bash
revision=$(git rev-parse HEAD)
incoming="$HOME/apps/ramion-releases/incoming"
bash scripts/deploy.sh deploy arm-workbench \
  "$incoming/ramion-arm-workbench-$revision.tar.gz"
```

The script verifies the archive and builds a fresh environment in its final
release directory before stopping the running service. It uses the workbench's
uv lock and the dashboard's own uv lock, with its OpenCV collector extra. It preserves
the original live directory on the first switch and keeps the prior release.
Workbench deployment starts disconnected preview. Dashboard deployment starts
its HTTP proxy. Host settings, notebooks, existing notebook environments and
dashboard capture storage stay outside the release directories.

After a switch, the script checks the service over HTTP. For workbench it
requires disconnected preview state. For dashboard it checks `GET /healthz`
(the dashboard alone, independent of the controller); a legacy dashboard release,
which has no `/healthz`, is checked on `/`.
Check camera feeds and the viewer separately. If the HTTP check fails, the script
restores the previous release when available.

### Roll back

When rolling back across the UI ownership change, restore the workbench first.
Earlier dashboards require the workbench to serve the operator assets.

From the monorepo checkout, run the relevant command as that app's owner:

```bash
bash scripts/deploy.sh rollback arm-workbench
bash scripts/deploy.sh rollback arm-dashboard
```

Rollback switches to the previous release and retains the displaced release as
the next rollback target. Keep both release directories. A failed build leaves
the existing service running and retains the staging directory for inspection.

The installed services and relay units remain host configuration. Review service
template changes separately before installing them. For manual physical
operation, see the [workbench guide](../apps/arm-workbench/ops/linux/README.md).
