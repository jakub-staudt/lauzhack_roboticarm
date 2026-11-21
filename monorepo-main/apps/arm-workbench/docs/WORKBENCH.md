# Arm workbench

The `soarm-panel` entry point provides manual hardware control, two optional USB camera views, a read-only measured-pose 3D view, and activity/diagnostics. The browser's Model tab remains a placeholder. The [model notebooks](../notebooks/README.md) provide local SmolVLA inference, a Codex skill planner, and Dream-1 through the HTTP API while the workbench remains the sole hardware owner.

Read [safety.md](safety.md) and [first-contact.md](first-contact.md) before powering the arm.

## Install and launch

From the workbench workspace root:

```powershell
uv sync --group workbench
uv run --group workbench soarm-panel --listen 0.0.0.0:8765 --overhead-camera 1 --wrist-camera 2 --no-browser
```

Start the operator UI separately using the [dashboard run instructions](../../arm-dashboard/README.md#develop). The example camera indices are specific to the development laptop. Linux also accepts stable device paths under /dev/v4l/by-id/. The controller refuses duplicate devices, including Linux aliases. Session drawer changes affect all observers and last until restart. Use the Linux service environment file for persistent assignments. See [Linux deployment and service commands](../ops/linux/README.md).

On Windows, stop an existing `soarm-panel` process before `uv sync`: a running executable can lock the installed `.exe` and prevent replacement. Restart with the intended command afterward.

For the base manual panel without the workbench dependency group:

```powershell
uv sync
uv run soarm-panel
```

The default bind is `127.0.0.1:8765`. Choose a serial port if automatic single-device selection is ambiguous:

```powershell
uv run --group workbench soarm-panel --port COM5 --overhead-camera 1 --wrist-camera 2 --no-browser
```

Camera capture and rendering are optional. Missing OpenCV or renderer dependencies surface as unavailable views while base panel controls remain usable. `--no-viewer` disables 3D; `--viewer-port 8766` selects its separate port. The requested viewer port must differ from the panel port; the renderer may select another free port if occupied, and its actual port is reported in telemetry.

## Disconnected preview

Add `--preview` to serve the same cameras and read-only viewer without detecting, opening or
energizing an arm. Robot measurements remain empty and controls stay disabled; no pose is fabricated.
Without this flag, startup connects and enables torque after setting goals to measured positions.
The Linux deployment enables preview at boot and keeps arm control an explicit manual service start.

## Laptop and phone

Open http://localhost:8780 on the host. For a phone, open port 8780 on the laptop's trusted network address after configuring the dashboard to listen there. Use `--dashboard-url` to set the controller’s browser redirect when the dashboard is on another address.

Direct clients use port 8765 for the JSON API, camera streams and snapshots. The dashboard relays the read-only viewer from port 8766 by default. Restrict access to the intended trusted devices. Controller leases coordinate operators. Keep the dashboard and controller on a trusted private network.

Laptop: overhead view leads, wrist and 3D sit alongside, and controls occupy a rail. Phone: Overhead/Wrist/3D view buttons select the media above the controls. This layout applies at width up to 760px, or width up to 1000px together with height up to 500px for short landscape screens. Gripper feedback is compact and its slider/Open/Close actions avoid a duplicate jog row. The 3D enlarge action expands the viewer in place and preserves access to manual controls. Narrow frames scale an iframe viewport with a minimum logical width of 640px; in mobile layout the iframe loads only when 3D view is selected and unloads when another media view is selected. Session contains camera assignment and advanced torque. Activity & diagnostics expands below the workspace.

## Control and stop behavior

1. Observe state, camera freshness, and surroundings.
2. Select **Take control**. Exactly one browser owns the motion lease.
3. Select a joint and use a deliberate jog or release a slider at the intended target. Adjust speed as needed. Gripper controls remain separate.
4. Select **Release control** when finished.

The owning browser sends a heartbeat every 400ms. The lease expires after 1.5s without an accepted owner heartbeat. Hiding or leaving the page releases control; tokens remain in browser memory and are discarded on loss of ownership or connection. After interruption, reconnect and claim again. Observers polling state do not renew control.

**STOP** and Escape request a measured-pose hold and revoke ownership. STOP needs no token and is available to any reachable client. With torque enabled, hold keeps it on; STOP does not enable disabled torque and is not a power disconnect. Ctrl-C exits while keeping torque on. Keep the physical power disconnect reachable. Support the arm before disabling torque in Session > Advanced.

Commands are limited by configured robot limits and servo EEPROM limits. They do not establish collision clearance. Loss of communication may prevent a stop command from reaching hardware.

## Cameras and 3D semantics

Each assigned camera has one capture worker; HTTP clients consume cached JPEGs rather than opening additional capture devices. On Windows, capture uses DirectShow, requests 640×480 at 15fps, and applies MJPG after resolution negotiation. That ordering was required for the development Innomaker cameras; actual delivered resolution/rate comes from capture metadata. Failed reads trigger retry; a blocked native driver may still require unplugging a device or restarting the process.

Camera metadata distinguishes unconfigured, connecting, live, stale, and unavailable states. Last-frame age and source errors explain interruptions. A camera becomes stale after two seconds without a fresh frame. The UI labels a retained last frame rather than presenting it as current.

The 3D viewer mirrors cached **measured** joint feedback, including calibrated gripper mapping, at approximately 10Hz. Joint selection highlights the joint visually; orbiting is a viewing interaction. It does not create another hardware backend, run a physics control loop, accept motion through the scene, or replace measurements with a requested/home pose. Feedback older than 0.75s or unhealthy/disconnected feedback is marked stale; invalid/incomplete poses are not treated as live. Geometry is not a collision or physical-safety guarantee.

## Diagnostics

Robot state displays robot name, serial port, loop frequency, state age, sample sequence, torque, clipping, and browser render time. “Updated” is the browser's local display time, not an independent sensor timestamp. Joint tracking shows measured angle, requested target, and their difference. Faults and recent server activity are shown separately.

Telemetry includes configured limits, speed, commanded/ramped positions, targets, feedback health, ownership, camera metadata, and viewer availability. Temperature, voltage, current, force, and torque measurements are not supplied by this panel; the torque flag means enable/disable state.

## HTTP contract

These are payload examples, not instructions to issue live commands during documentation or testing. Use a hardware-free test fixture for automated API validation. Joint angles are in degrees, gripper positions in percent, and speed in degrees per second.

| Request | Body / result |
| --- | --- |
| `GET /api/state` or `/api/telemetry` | Cached snapshot; no lease renewal or direct hardware read. |
| `POST /api/control/claim` | `{"label":"Laptop browser"}` → `{"token":"<opaque-token>","owner_id":"<opaque-id>"}`. Refused if ownership or feedback conditions prevent claiming. |
| `POST /api/control/heartbeat` | `{"token":"<opaque-token>"}` renews the current lease. |
| `POST /api/control/release` | `{"token":"<opaque-token>"}` holds and revokes ownership. |
| `POST /api/stop` | `{}` holds and revokes ownership without a token. |
| `POST /api/target` | `{"token":"<opaque-token>","joint":"<joint-name>","value":10}`. |
| `POST /api/jog` | `{"token":"<opaque-token>","joint":"<joint-name>","delta":1}`. |
| `POST /api/home` | `{"token":"<opaque-token>"}`. |
| `POST /api/speed` | `{"token":"<opaque-token>","value":30}`. |
| `POST /api/model/arm` | `{"token":"<opaque-token>","duration_s":5,"speed":8}` → `{"session_id":"<opaque-session>","duration_s":5,"speed":8,"action_timeout_s":0.5,"observation_max_age_s":3}`. |
| `POST /api/targets` | `{"token":"<opaque-token>","session_id":"<opaque-session>","observation_sequence":123,"targets":{"shoulder_pan":0,"shoulder_lift":0,"elbow_flex":0,"wrist_flex":0,"wrist_roll":0,"gripper":50}}`. The sequence must come from recent measured feedback; values must satisfy current limits. |
| `POST /api/model/disarm` | `{"token":"<opaque-token>"}` holds and ends the model session while retaining the control lease. Release that lease separately when finished. |
| `POST /api/torque` | `{"token":"<opaque-token>","on":true}`. |
| `POST /api/cameras` | `{"token":"<opaque-token>","overhead":1,"wrist":2}`; use `null` to unassign. |
| `POST /api/view` | `{"joint":"<joint-name>"}` selects a visual highlight without motion ownership. |
| `GET /cameras/overhead.mjpg`, `/cameras/wrist.mjpg` | MJPEG stream from the assigned capture worker. |
| `GET /cameras/overhead.jpg`, `/cameras/wrist.jpg` | Fresh cached JPEG; HTTP 503 when none is available. |

A snapshot carries `robot`, `port`, `joints`, `gripper`, `limits`, `measured`, `command`, `target`, `torque`, `speed`, `clip`, `faults`, `loop_hz`, `age_s`, `sequence`, `loop_ok`, `connected`, `control`, `sample`, `events`, `cameras`, and `viewer`. `control` exposes owner identity/label and remaining lease time, never the token. `sample` reports age, sequence, connection and loop health. Camera slots report source index or device path, status, frame age, dimensions, capture FPS, errors and URLs; viewer metadata reports availability, actual port, freshness and selected joint.

### Model sessions

Model execution first claims the existing exclusive controller lease, then explicitly
arms a bounded session. Arming starts from the measured pose; it does not request
Home or enable torque. Every model control tick passes through the twin's actual
`machinekind_twin.safety.SafetySupervisor`, configured from the workbench's joint
limits and velocity limits. The workbench dependency group supplies that supervisor.

`targets` replaces all six joint targets atomically. Missing/extra joints, booleans,
non-finite numbers and out-of-limit values are rejected. Arm units are degrees;
gripper units are percent. Model speed is capped at 15 degrees/s for arm joints and
15 percentage points/s for the gripper, further constrained by each joint's configured
velocity limit. The maximum session duration is 60 seconds.

| Guard | Behavior |
| --- | --- |
| Action watchdog | No accepted action for more than 0.5 seconds ends the session. Heartbeats alone cannot keep old model motion running. |
| Control lease | The existing 1.5-second owner heartbeat expiry still applies. |
| Observation reference | `observation_sequence` must identify server feedback sampled within the previous 3 seconds. |
| Cameras | Both overhead and wrist must be live, with frame ages at most 1 second, throughout the session. |
| Tracking | More than 15 degrees of commanded-to-measured error on an arm joint ends the session. |
| Feedback/faults/deadline | Unhealthy feedback, a servo fault, torque loss or the session deadline ends execution. |

Watchdog failures, STOP and ownership release hold the measured pose, clear the model
session and revoke control. A stale token/session cannot restart that run. Explicit
`model/disarm` holds and clears only the model session; `control/release` is still
needed to relinquish its lease. Manual target, jog, Home, speed/torque and camera
reconfiguration requests are refused while a model session is armed. STOP remains
available without a token.

Updated state responses also include:

- `capabilities`: `model_control`, `model_control_version` (1), `atomic_targets`,
  the supervisor name, and the configured `model_max_duration_s`, `model_max_speed`,
  `model_action_timeout_s`, `model_observation_max_age_s`, `model_camera_max_age_s`
  and `model_max_tracking_deg` values.
- `model_readiness`: `ready`, explanatory `reasons`, and `requires_control_lease`.
  Readiness reports cached prerequisites; it does not claim ownership or certify
  collision clearance.
- `model_session`: `active`, `expires_in_s`, `speed` and `last_action_age_s`.
  The session ID is returned only to the client that arms it.

Read-only snapshots and inference previews do not claim a lease. Notebook motion is
disabled by default and requires the explicit `ARM ROBOT` confirmation. A frozen or
interrupted kernel cannot renew the workbench's independent action deadline through
a hidden background heartbeat. All these checks supplement physical supervision;
joint interpolation and camera freshness do not establish a collision-free route.

## Implementation boundaries

`panel.py` owns HTTP/control state and the existing entry point. The dashboard package owns the browser workspace. `panel_cameras.py` owns capture workers and `panel_viewer.py` owns read-only pose rendering. Source files and hardware-free tests are the authority for exact API behavior. Documentation does not certify physical behavior, camera latency, or model readiness.

## Validation status

The development session verified the live arm connection, camera feeds, read-only viewer, and responsive presentation. The final hardware-free suite passed 109 tests. The agent did not issue physical arm movement; manual supervised physical acceptance remains independently unverified. A user may move the arm separately during observation, so visible pose changes alone do not establish an agent-commanded motion test. Camera latency, mechanical clearance, STOP under physical motion, and torque-release behavior still require supervised acceptance with the actual setup.

The model-notebook additions have hardware-free API/helper tests and default-cell
notebook checks. Physical execution, taught-pose clearance and task success remain
reserved for a supervised session. The development checkout and deployed source
snapshot are separate; an existing running deployment may not yet expose the model
session endpoints.
