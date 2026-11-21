# Arm dashboard

The dashboard on port 8780 serves its own SO-101 operator panel. The workbench on port 8765 owns the controller API and redirects browser visits to the dashboard. It proxies the controller API and rate-limited camera streams, and keeps the local `/map` and `/scan` tools. The read-only Viser pose viewer is relayed through `/viewer/` on 8780.

It is a FastAPI application (`src/arm_dashboard`). Existing control and camera routes follow the earlier standard-library proxy contract. That proxy is kept, not deployed, in [`legacy/`](legacy/README.md) as the test reference and for rollback until it is retired.

The workbench advertises supported commands in `capabilities.operator_commands`. The dashboard returns the subset it forwards as `operator_commands` in `/api/state`. The browser combines that list with workbench readiness and ownership state.

## Running deployment

```text
Browser -> dashboard :8780 -> soarm-panel :8765 -> SO-101 arm + cameras
                           -> Viser read-only measured-pose viewer :8766
```

- Dashboard process: `/home/kwikiel/ramion-dashboard/.venv/bin/arm-dashboard`, owned by `kwikiel`, one process.
- User service: `ramion-dashboard.service`, settings in `/home/kwikiel/.config/ramion/dashboard.env`.
- Dashboard address: `100.85.56.14:8780`. Liveness: `GET /healthz`.
- Controller upstream: `100.85.56.14:8765`.
- Controller process: `soarm-panel`, owned by `kwiscion`, from `/home/kwiscion/apps/ramion-workbench/src/.venv/bin/soarm-panel`.
- The controller source is under [`../arm-workbench`](../arm-workbench), a separate uv workspace.

The workbench provides overhead, wrist and optional observer camera views, measured-pose 3D, all-joint sliders, gripper Open/Close buttons and the latched software stop. The observer feed remains outside the overhead/wrist VLA input contract. The controller remains responsible for hardware access and enforcement of movement constraints.

## Layout

| Path | Contents |
| --- | --- |
| `src/arm_dashboard/core/` | Settings (the only reader of the environment), logging, base exceptions, shared types |
| `src/arm_dashboard/controller/` | HTTP client for soarm-panel and the enriched `/api/state` |
| `src/arm_dashboard/cameras/` | Snapshot fetch and the paced MJPEG stream |
| `src/arm_dashboard/viewer/` | Viser page and WebSocket relay |
| `src/arm_dashboard/scan/` | Scan sessions (storage, collector process); `scan/worker/` is the OpenCV collector |
| `src/arm_dashboard/api/` | Routes, same-origin middleware, error mapping, assembly; `main.py` builds the app |
| `tests/contract/` | Black-box HTTP/WebSocket contract, run against both this app and the legacy proxy |
| `tests/unit/` | Unit tests, including the import-tier rules (`test_architecture.py`) |
| `src/arm_dashboard/static/` | The operator panel and map/scan pages (`scan.html`, `board-map.*`) |
| `tests/manual/` | Latency, footprint and slow-reader measurements, run by hand |
| `legacy/` | The previous standard-library proxy and its tests — reference only |

## Configuration

Every setting is an environment variable `RAMION_DASHBOARD_<SECTION>__<FIELD>`, or a line in `.env` in the working directory. [`.env.example`](.env.example) lists all of them with their defaults. Production values are in [`deploy/ramion-dashboard.env`](deploy/ramion-dashboard.env).

The upstream defaults are the production addresses, so a dashboard started on a machine in the tailnet without settings talks to the live controller. For local work, always set `RAMION_DASHBOARD_CONTROLLER__BASE_URL` and `RAMION_DASHBOARD_VIEWER__BASE_URL`. The bind address defaults to `127.0.0.1`.

## Develop

Python 3.12. With uv, from this directory:

```bash
uv sync --locked --extra scan        # app, OpenCV collector extra and dev tools
uv run pytest -q                     # contract + unit tests, no hardware
uv run ruff check . && uv run ruff format --check .
uv run pyright
```

Without uv, any venv works: `pip install -e '.[scan]' --group pyproject.toml:dev`.

To run it locally against a controller in disconnected preview (no arm, no serial port), install soarm-panel in a separate venv — `pip install "machinekind-twin-api @ git+https://github.com/machinekind/twin.git@fb3838f9ab72d6a2bb9cc34d2fdbb113580a4afc#subdirectory=packages/machinekind_twin_api" "./apps/arm-workbench/packages/soarm[panel]"` from the repository root — and start both:

```bash
RAMION_STOP_STATE_FILE=/tmp/ramion-stop.json soarm-panel --preview --listen 127.0.0.1:8765 --no-viewer --no-browser
RAMION_DASHBOARD_CONTROLLER__BASE_URL=http://127.0.0.1:8765 \
RAMION_DASHBOARD_VIEWER__BASE_URL=http://127.0.0.1:8766 \
RAMION_DASHBOARD_SCAN__STORAGE_DIR=/tmp/ramion-scans \
uv run arm-dashboard                 # http://127.0.0.1:8780
```

## Deploy on the server

For `/srv/ramion` releases and the system-wide unit, follow the [deployment runbook](../../docs/deployment.md). The setup below is for the legacy user unit.

`scripts/deploy.sh` builds each release with `uv sync --locked --no-dev --extra scan` in its final release directory, checks the imports and pages, switches `~/ramion-dashboard` and checks `GET /healthz`; see [deployment](../../docs/deployment.md).

The unit and its settings file are host configuration, installed once by `kwikiel` before the first FastAPI release:

```bash
install -D -m 600 deploy/ramion-dashboard.env ~/.config/ramion/dashboard.env
install -D -m 644 deploy/ramion-dashboard.service ~/.config/systemd/user/ramion-dashboard.service
systemd-analyze --user verify ~/.config/systemd/user/ramion-dashboard.service
systemctl --user daemon-reload
```

The unit starts `.venv/bin/arm-dashboard` when the active release has it and the legacy `server.py` otherwise, so it works before the switch and after a rollback to a legacy release. Persistent operation after logout requires user lingering; it is already enabled for `kwikiel` on this host. Run one process only: the scan session state lives in its memory.

## Run in Docker

The image holds the app and the OpenCV collector, installed from `uv.lock`, running as uid 10001 with a `/healthz` health check. [`deploy/compose.yaml`](../../deploy/compose.yaml) at the repository root runs it with a fixed resource budget: 1 CPU, 1 GB, read-only root, no capabilities, rotated logs, scan sessions on a volume. The arm controller stays a systemd service on the host.

```bash
# from the repository root
docker compose -f deploy/compose.yaml build dashboard
# side by side with the systemd dashboard on the server:
DASHBOARD_PUBLISH=100.85.56.14:8790 docker compose -f deploy/compose.yaml up -d dashboard
# on a laptop, against a controller on the same machine:
DASHBOARD_PUBLISH=127.0.0.1:8780 \
CONTROLLER_URL=http://host.docker.internal:8765 VIEWER_URL=http://host.docker.internal:8766 \
docker compose -f deploy/compose.yaml up -d dashboard
```

The port is always published on one explicit host address: Docker bypasses `ufw`. The container and `ramion-dashboard.service` cannot both own `100.85.56.14:8780`; run one of them. A host directory for `SCAN_DIR` must be writable by uid 10001.

`tests/manual/live_smoke.py` checks both direct and proxied disconnected preview, then verifies the dashboard UI, camera and viewer routes, and scan collection. The contract suite tests STOP, reset, and control commands against a fake controller.

## Access from another tailnet

The server maintains a loopback-only reverse web tunnel on VPS port 18780. With the existing VPS SSH alias, forward that port directly:

```bash
ssh -NT -L 127.0.0.1:8780:127.0.0.1:18780 kacperwikiel
```

Then open <http://localhost:8780>. The web route avoids nesting an SSH session inside the reverse SSH tunnel. The separate `kwiscion` SSH alias still uses port 2223 for shell access. A Mac LaunchAgent already maintains this forward on the operator's Mac; do not start a second process on the same port. SSH keys and local tunnel credentials are not included here.

## Current limitations

- The proxy pulls fresh camera snapshots at a maximum of 2 fps per stream by default to reduce SSH tunnel congestion; `?fps=10` and `?fps=15` opt in to more. The controller camera capture rate is unchanged.
- The UI uses 1.8-second request timeouts. VPS-path status requests were measured at roughly 1.2–1.9 seconds, so intermittent disconnects remain possible.
- This proxy has no independent login/authentication layer. Keep it on a trusted private interface or behind an authenticated SSH tunnel; it forwards control requests to the controller. Browser POSTs and WebSocket upgrades must come from a page it served (same-origin check).
- A stalled reader can still hold several MJPEG parts in socket buffers; the legacy proxy behaved the same (measured).

## Verification

```bash
uv run pytest -q
```

The contract suite drives the app over real sockets against a fake controller and a fake Viser. It checks shared behavior against the legacy proxy and verifies that the dashboard serves its assets independently. Nothing connects to hardware or issues real commands. `source-snapshot.json` is the historical record of the files imported from the server on 2026-11-03.

### Workbench UI and control lifecycle

The UI ignores status/command responses from older control sessions. Heartbeats start immediately after a claim and the next heartbeat is scheduled 100 ms after its acknowledgement. Hiding the page releases control; the controller's 1.5-second lease and all movement checks remain unchanged. Delays can still revoke control safely.

Node's built-in test runner checks the dashboard UI contract with mocked state and HTTP actions, including all joint sliders, gripper endpoints, read-only joint highlighting, claim generations, STOP/reset races and the localhost camera relay:

```bash
node --test tests/test_dashboard_ui.cjs
```

Run that command from the repository root. These tests issue no real hardware requests and do not prove physical movement.

### Board map and camera transport

The dashboard links to `/map`, a read-only whole-visible-plane XY mapping page. It uses a frozen overhead snapshot, four measured reference corners and dimensions; it exports a rectified PNG and calibration JSON. See [whole-board mapping](../../docs/board-mapping.md). It does not infer unobserved board geometry, claim calibrated accuracy or move the robot.

When opened through the localhost relay, the panel requests camera streams through the separate port 8781 tunnel. If that stream fails, it falls back to same-origin snapshots on port 8780. Direct Tailscale access uses same-origin camera streams. Previews remain capped at 2 fps through the dashboard proxy unless a higher rate is selected.

### Automatic collection

`/map` and `/scan` show automatic server-side collection; manual corner selection is optional at `/map/manual`. A separate OpenCV subprocess (`python -m arm_dashboard.scan.worker`) captures both existing HTTP camera feeds, filters telemetry, checks feature alignment and writes a candidate mosaic without opening camera devices or issuing robot commands. See [automatic collection](../../docs/automatic-collection.md) for artifact storage, tests and limitations. The result remains in pixels until physical scale is known.
