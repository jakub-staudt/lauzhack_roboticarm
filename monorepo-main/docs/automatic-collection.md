# Automatic board collection

Open <http://localhost:8780/map> (also `/scan`). This is now the default map page; manual corner calibration moved to `/map/manual`.

The operator requested automatic collection instead of clicking corners or saving individual frames. The server collector therefore runs a bounded **60-second session** independently of the browser:

1. Every second it concurrently requests cached JPEGs from both existing camera feeds and cached controller state.
2. It saves both raw frames, SHA-256, request timing, image dimensions, sharpness, and the associated measured-joint feedback. HTTP requests are not synchronized camera exposures; this is marked explicitly.
3. It automatically matches SIFT features to the first overhead anchor, fits a RANSAC planar transform, and rejects weak, spatially localized or implausible matches.
4. Accepted novel views can contribute to a candidate image mosaic. Unknown pixels remain blank. Repeated observations of the same view do not establish new physical coverage.
5. Live status and the latest images appear in the browser. Captures continue if the tab is closed. Closing the dashboard service interrupts collection; partial data remain stored.

This is a **read-only camera collector**. It opens no USB camera, serial device, robot backend or ownership lease, and sends no movement POSTs. It reads only the already-running controller's cached HTTP endpoints. Capture ownership remains with `soarm-panel`.

## Storage and scale

Sessions are stored privately under `/home/kwikiel/ramion-scans/scan-<UTC timestamp>-<id>/`:

- `00000-overhead.jpg`, `00000-wrist.jpg`, etc.: raw cached frames.
- `frames.jsonl`: per-frame quality, registration decisions and filtered telemetry; control tokens are not retained.
- `status.json`: counters, phase, errors and candidate pixel coverage.
- `mosaic.jpg`, `overhead.jpg`, `wrist.jpg`: latest browser artifacts.
- `collector.log`: worker errors.

Raw camera imagery is not committed to GitHub. The HTTP image endpoint only exposes the current session's three named preview artifacts, not arbitrary paths or metadata files.

The mosaic is currently in **anchor-camera pixels**, not millimetres. A single-plane fit does not verify physical accuracy or remove lens distortion, parallax and above-plane objects. No board dimensions are guessed. Absolute XY scale still requires an observed, known-size reference. That can eventually be detected automatically using fiducials; manual corner clicks are not required by the collection workflow.

Automatic collection alone cannot observe board regions that remain outside both views or behind the arm. A wrist-camera sweep needs a separately verified motion path and supervised physical acceptance; it is not silently enabled by this collector.

## Runtime

The collector is a separate subprocess (`python -m arm_dashboard.scan.worker`) started by the dashboard with its own interpreter. Its OpenCV and NumPy come from the dashboard's `scan` extra, pinned in `apps/arm-dashboard/uv.lock`; the dashboard reports `Scanner runtime is not installed` when the extra is missing:

```bash
cd apps/arm-dashboard
uv sync --locked --extra scan
```

Dependencies are locked separately from the controller environment. Only the collector imports OpenCV; the web process never does. OpenCV is limited to one compute thread in the worker. Sessions are time-bounded, image reads and mosaic dimensions are bounded, and at most 16 non-anchor candidate layers are retained in memory. Raw session files are preserved for review.

Read-only capture can also be run directly:

```bash
.venv/bin/python -m arm_dashboard.scan.worker --base http://100.85.56.14:8765 --output /path/to/session --seconds 60
```

`--seconds` must be between 1 and 120. SIGTERM or SIGINT stop it after the current cycle with a final `stopped` status.

## HTTP interface

| Endpoint | Purpose |
| --- | --- |
| `GET /api/scan/status` | Current or most recent session status. |
| `POST /api/scan/start` | Start a fixed 60-second collection session; reject concurrent starts. |
| `POST /api/scan/stop` | Stop only the collector, retaining partial files. |
| `GET /api/scan/images/{mosaic,overhead,wrist}.jpg` | Read current preview artifact. |

Browser POSTs retain the existing same-origin restriction. On the operator's Mac, preview images use the separate localhost:8781 video tunnel; status uses localhost:8780.

## Verification

From the monorepo root:

```bash
(cd apps/arm-dashboard && uv sync --locked --extra scan && uv run pytest -q)
node --test tests/test_dashboard_ui.cjs tests/test_board_map.cjs
```

The collector tests also run the legacy `apps/arm-dashboard/legacy/scan_worker.py` on the same inputs and require identical registration results, mosaics and session files. Tests cover synthetic known-transform recovery, featureless-frame rejection, blank unobserved mosaic gaps, collection against a fake HTTP source without commands or tokens, concurrent-job rejection, preview path restrictions and interrupted-job reporting. Synthetic success does not validate the real camera geometry.

## First live run

The first deployed session completed in 60.01 seconds: **120 frames saved**, 60 overhead observations passed registration, 60 wrist observations were rejected and retained separately, and zero capture errors occurred. The candidate mosaic contained only its overhead anchor layer, so **coverage did not expand**. No physical scale or robot-base registration was established. The collector issued no movement commands. See the [sanitized session summary](measurements/2026-11-03-automatic-collection.json); raw frames and measured joint feedback remain on the server.
