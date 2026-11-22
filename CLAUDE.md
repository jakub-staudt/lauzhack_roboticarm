# Eye-controlled pick-and-place for the SO-101 arm (LauzHack, 12-hour build)

Read this first when resuming. It records the goal, what exists, what was learned, and what is next.

## Goal
Paralysed users control a robot arm remotely. A Logitech camera tracks the user's eyes on a screen (pointer); dwelling on a
point selects it; a button resets the selected points. Task: pick a block (red/blue/green, same size) from the selected
point and put it at the selected destination. The table has a rectangle with AprilTags at its corners; selections are
in rectangle coordinates `(u, v)`, each 0..1. Only the overhead camera is used. A teammate builds the eye tracking and
sends `(u, v)` start/end points; the user builds the arm side and the web UI.

## Current status (2026-11-22)
- **The real arm picks and places blocks with step buttons** (camera page on :8820, grasp card): open, hover, lower, grip, lift, put back and release, park. Each step is
  pressed and confirmed by the user, who keeps Stop (Esc) ready. The arm agent runs next to the controller on the server (`~/arm_agent`, tmux session `arm_agent`, port 8790,
  token in `~/arm_agent/token`), see `lauzkach_eye_robot_control/docs/AGENT_DEPLOY.md`. Not built: fully automatic pick-and-place, eye-tracking input, gripper-camera
  alignment before lowering (only the lowering stop), moving blocks to a chosen destination.

## Hard rules
- **A working version of the arm software runs on the RAMION server** (`ramion-server-01`, Tailscale `100.85.56.14`,
  controller `:8765`, dashboard `:8780`). Never break it; the user must be able to revive it. Only read-only calls
  (`GET /api/state`) without asking. Ask before SSH, deploys, restarting services or editing anything on the server.
  Recovery path: `monorepo-main/docs/deployment.md`, `scripts/deploy.sh` (release symlink + `previous`), STOP latch reset.
- **Never run real arm motion unsupervised.** The user supervises and keeps the STOP button within reach. Do not run
  `goto.py` or send trajectories yourself. Claiming control for debugging is OK only briefly and without motion.
- **Keep torque ON** (the user controls the arm remotely). Nothing here should change torque.
- The microcontroller is an ESP32-C6, not WROOM (see global CLAUDE.md); there is no firmware in this repo.
- Only one control owner at a time. The `/stack` game page on the server (label "STACK player") and the original
  dashboard also claim control; close them before taking control here.

## Layout (`/home/jakub-staudt/Documents/lauzhack_roboticarm/`)
- `monorepo-main/` copy of RAMION's monorepo (snapshot of 2026-11-03; the server runs newer code, e.g. `/stack`, not in the copy).
  `apps/arm-workbench` (controller `soarm`, port 8765; `soarm_lab` kinematics/sim), `apps/arm-dashboard` (FastAPI UI, 8780).
- `robotic_arm/` **our arm code** (own venv `.venv`; `soarm` and `soarm_lab` installed editable, no deps).
- `web_ui/` **our new control page** (stdlib server + static files).
- `eye_tracking/` teammate's area; README holds the message contract (`select`/`reset` with `u,v`).
- `calibration_poses.json`/`calibration.json` live in `robotic_arm/` (gitignored).

## How to run
```bash
# tests (the ROS pytest plugin breaks pytest unless autoload is off)
cd robotic_arm && PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 .venv/bin/python -m pytest -q tests     # 6 passed

# watch joints + tip position + pitch (read-only)
.venv/bin/python scripts/watch_joints.py            # --once for a single line

# calibration: user jogs the tip to a point (torque on), then record it (read-only, needs the arm still)
.venv/bin/python scripts/record_point.py corner_00  # also --list, --reset
.venv/bin/python scripts/calibrate.py --offline calibration_poses.json   # fit + residuals -> calibration.json

# goto: rise to 3 cm, glide, lower (asks for confirmation; --dry-run first)
.venv/bin/python scripts/goto.py 0.5 0.5 --dry-run

# web UI (do not use pkill -f with the command line: it kills your own shell)
cd web_ui && nohup ../robotic_arm/.venv/bin/python server.py --port 8800 > /tmp/ui.log 2>&1 &   # robotic_arm venv enables click-to-move; plain python3 = UI only
# then open http://127.0.0.1:8800/?name=Jakub   (find the proxy pid with: ss -ltnp | grep :8800)
```
Controller address: default `http://100.85.56.14:8765` (`robotic_arm/config.py`, override in `config.yaml`).

## What is built
- `robotic_arm/calibration.py`: 9 calibration points (`POINTS`: 4 corners, 4 edge midpoints, centre), homography (u,v)->(x,y) in the
  arm base frame fitted from FK of the measured joints, `z_table` = mean tip height. Residuals reported per point.
- `robotic_arm/planner.py`: pure planning on `soarm_lab.kinematics.RobotKinematics` (MuJoCo IK, position-priority, 5-DOF).
  `goto()` (rise/glide/lower) and `plan()` (full pick-and-place, not yet wired to a service). Tries pitches 90/75/60/45 deg.
  Joint speed 12 deg/s, gripper 15 units/s (the controller caps a model session at 15 and stops at 15 deg tracking error).
- `robotic_arm/executor.py`: runs a plan via `soarm_lab.notebooks.client.WorkbenchClient.run_targets` (claim, arm, heartbeat,
  release; sessions <= 60 s so long plans are chunked; the first row must be within 2 deg of the measured pose).
- `web_ui/`: same-origin proxy (the controller rejects cross-origin browser POSTs) forwarding only state, overhead camera,
  control claim/heartbeat/release, cartesian jog and stop/reset. Page: camera snapshots (~2 fps), take/release control, keyboard
  hold-to-move (W/S A/D R/F I/K J/L Q/E, Esc = STOP), speed menu, Recovery mode checkbox, joints readout, labels the control owner
  (`?name=` or random tag), wrist-roll nudge buttons. Tolerates a few missed polls, overlapping heartbeats every 250 ms, retries stale-observation starts,
  releases control on page close. Proxy logs the controller's refusal reasons (`/tmp/claude-1000/ui.log` last session).
- Not built: pick-and-place service (`/select`, `/reset`, state machine), overlay of rectangle/points/gaze on the camera view,
  AprilTag detection, colour-based block snapping.

## MVP in progress: click-to-move (built, NOT yet tried on the real arm)
- User decision: for now the user clicks points on the overhead camera image; the arm goes 25 mm above that table point, lowers, rises, then waits for
  the next click. **Gripper stays closed** (moves are refused if gripper > 25; the code never commands the gripper). Harder pick-and-place comes later.
- Calibration for this: web UI "Click to move" card, Calibrate mode: put the tip on a spot visible in the image, click that spot -> stores the tip's 3D position
  (FK, base frame) with the click (`web_ui/mvp_plane.json`, gitignored). 4+ spots -> homography image->table xy (5+ gives an error check). z_table = mean spot z.
- `robotic_arm/robotic_arm/clickmove.py`: `ImagePlane`, `plan_click` (IK waypoints: rise, glide at >= hover height, above, down, up; keeps the current
  tilt/roll first, falls back to fixed pitches; refuses joint jumps > 35 deg or joint-limit violations; keeps tip above the controller's floor + 1 mm), `Mover`.
- Mover sends joint targets with `POST /api/target` (the dashboard-slider command; the controller ramps to each target itself) and waits until each is reached;
  heartbeats every 250 ms. This was chosen over `WorkbenchClient.run_targets` because that streams ~20 Hz and fetches both camera images per tick,
  and the controller aborts if an action is > 0.5 s late: not viable over a laggy link. Joint targets bypass the controller's tip floor, hence the check in `plan_click`.
- `web_ui/server.py` exposes `/api/mvp/{status,calpoint,calundo,calclear,click,cancel}`; the page has an "Allow the arm to move when I click" checkbox (off by default).
  STOP / release / "Cancel move" cancel a running move (cancel calls the controller's `stop`, holding the pose). Tests: `robotic_arm/tests/test_clickmove.py` (13 pass overall).

## Controller facts learned (apps/arm-workbench/packages/soarm/soarm/panel.py, panel_cartesian.py)
- API: `GET /api/state`, `/cameras/{overhead,wrist,observer}.{jpg,mjpg}`; `POST /api/control/{claim,heartbeat,release}`,
  `/api/emergency-stop[/reset]`, `/api/cartesian/{start,renew,stop}`, `/api/model/{arm,disarm}`, `/api/targets`, plus jog/target/torque/speed/home.
  Claim needs `emergency_stop_generation`; lease expires after **1.5 s** without a heartbeat. STOP latches; reset needs the generation.
- Hold-jog: `cartesian/start` needs `observation_sequence` of an observation **under 0.75 s old**, then renew about every 100 ms.
- Joints are degrees, gripper 0..100. Home pose: pan 0, lift -23.3, elbow 46.4, wrist_flex 3.3, roll 0, gripper 35.
- **TCP floor:** the controller refuses Cartesian moves below `table_z + margin` = -0.0074 + 0.005 = **-0.0024 m** (base frame). If the
  tip starts below it, normal moves are refused; use wrist tilt (I) or Recovery mode (<=3 deg tilt, 2 mm drift). The floor is set in the
  server's environment (`RAMION_CARTESIAN_TABLE_Z_M`); do not change it.
- **Hardware (EEPROM) limits:** any measured joint outside them blocks every Cartesian start ("Measured X is outside its EEPROM limit").
  wrist_roll's range is asymmetric, about **-113.8 to +95.2**, so rolling left to the end stop triggers it. Keep roll near 0.
  Recovery mode does NOT help: the EEPROM check in `start_cartesian` runs before the mode is considered. Use a plain joint jog
  (`POST /api/jog`, which the controller accepts and clamps to the allowed range). The web UI has a wrist-roll nudge (+/-10 deg) for this;
  the proxy allows `/api/jog` only for wrist_roll with |delta| <= 20.

- The RAMION `/stack` game moves the arm itself (it claims control as "STACK player" each round). After it was interrupted mid-game the arm
  was left with wrist_roll at its end stop (-114 deg, which blocks Cartesian moves) and the gripper wide open (98), tip about 12 cm above the
  table. After using the game, restore roll to about 0 and gripper to about 10 before calibrating. Poses already saved in
  `calibration_poses.json` are unaffected by later movement.

## Workspace decision (user, 2026-11-21)
- **We work only in the LEFT SQUARE of the drawn grid.** The paper has blue lines: two horizontal ones and a vertical one at about mid-page in the overhead
  image (a second slanted vertical line sits near the right-hand tags). The arm cannot reach the right part (max tip reach about 0.38 m from the pivot;
  the old far corner measured x 0.386, y -0.165), so the right tags are outside the usable area. Aim for every target within about 0.34 m of the pivot.
  (From one look at the overhead image: the square lies left of the mid-page vertical line, between the two horizontal lines; exact edges not measured.)
- Consequences: the earlier `corner_10` and `corner_11` entries in `calibration_poses.json` belong to the old, larger rectangle and must be redone
  for the left square; `corner_11` is flagged as max reach. The eye-tracking teammate's (u,v) must use the left square's corners, not the four tags.

## Calibration protocol (in progress)
- User jogs the **gripper tip** (torque on) onto each marked point via the web UI keyboard, then I record it with `record_point.py NAME`.
- Minimum set (agreed with the user): the 4 corners + `centre`. The centre is the independent check (4 corners alone always fit exactly, so
  their residual is 0); add edge midpoints only if the centre residual is large (> ~5 mm). `calibrate.py` fits whatever is recorded (>= 4).
- Full list of names: corner_00 (0,0), corner_10 (1,0), corner_11 (1,1), corner_01 (0,1), edge_u_v0, edge_u1_v, edge_u_v1, edge_u0_v, centre.
  The same feature of each tag (e.g. centre) must be used, and the teammate's (u,v) must use the same convention and origin (tag 0).
- **Keep the same wrist tilt (pitch) and roll at every point** (recorder warns when pitch differs by > 5 deg). The first two points were taken at
  pitch 84.6 and 40.8 deg with different roll, which shifted the contact point and gave a 17 mm height difference; redo `corner_01`.
  Aim for pitch about 55-65 deg if reachable at the farthest corner, roll about 0, gripper opening about 10.
- Status at last check: `calibration_poses.json` holds `corner_00` (tip x 0.210, y 0.066, z 0.012) and a bad `corner_01`
  (x 0.397, y 0.040, z -0.005). Rectangle appears about 19 cm along the (0,0)->(0,1) edge. Re-record both under a fixed wrist pose.
- User decision: only the 3D tip coordinates matter. `calibration_poses.json` stores `xyz_m` (tip in the arm base frame) per point; the planner later
  chooses joints/route by IK, so the pose used while recording is not required to match (small contact-point offsets may show up as residuals).
- Target quality: worst residual under about 5 mm (redo above 8 mm), table-height spread under about 5 mm.

## Network lessons
- **Root cause found (the RAMION game worked flawlessly on the same link):** my proxy talked to the controller on :8765, which closes the connection after every reply, so every poll/heartbeat paid new-connection round trips over the relayed Tailscale path (about 122 ms per call vs about 55 ms on a kept-alive connection), and the extra connections made the path jitter (ping 35-202 ms with the page running vs steady 40 ms without). Fix: the proxy (`web_ui/server.py`, default `--controller http://100.85.56.14:8780`) and the Mover now go through the server's **dashboard (:8780)**, which has the same `/api` and `/cameras` routes, and reuse kept-alive connections (`pooled_http`). State calls through the proxy dropped from 430-870 ms to about 100 ms.
- **Camera streams:** the dashboard streams MJPEG at only 2 fps unless asked: `/cameras/<name>.mjpg?fps=<2|10|15|20>&profile=preview` (the game uses 20). The page uses
  live streams at 20 fps for both cameras (overhead 20 fps ~660 KB/s, wrist ~350 KB/s through the proxy); the proxy only forwards those exact options.
  An earlier version polled snapshots every 0.5-1 s: that was the "laggy camera". Do not reduce camera rates again (user preference).
- **Disk full incident (2026-11-21):** the laptop's disk reached 0 free while `web_ui/server.py` was being rewritten and the file was truncated to 0 bytes; it was rewritten from
  the session history. If the proxy will not start, check `wc -c web_ui/server.py` and `df -h /`. Keep a few GB free (the robotic_arm venv alone is hundreds of MB).
- **Velocity motion (`POST /api/motion/input`) is what the server's panel and the STACK game use for keyboard control** (found by reading `/panel.js`, `/static/stack.js`,
  `/static/motion-input.js` on :8780, saved under /tmp during the session). Claim with `{app:'panel', label, emergency_stop_generation}`; while a key is held POST every ~100 ms
  `{token, sequence (increasing), observation_sequence (state.sample.sequence ?? state.sequence), profile: slow|medium|fast, axes:{pan,reach,z,tilt,roll,grip} in [-1,1], precision:false}`;
  one moving request in flight at a time; one all-zero request stops (neutral may overtake a pending move, its higher sequence fences it); do not send moving input unless
  `state.motion.available` and `observation_sequence > state.motion.input_hold_observation_sequence`; heartbeats every ~400 ms; poll state every ~125 ms. Key map: W/S reach, A/D pan,
  R/F z, I/K tilt, J/L roll, Q/E grip. `web_ui/static/app.js` uses it when `capabilities.operator_commands` lists `/api/motion/input`, falling back to the old `cartesian/start|renew` hold-jog.
  **REVERTED (user asked to return to the working state):** the page was put back to hold-jog keys (`cartesian/start|renew`), 250 ms polling, 20 fps cameras, no auto/velocity menu. Reason: velocity commands are refused with
  "Motion input has stale robot feedback" whenever the link RTT is above roughly 200 ms (the controller wants a very fresh observation), and the relayed Tailscale link often is. Revisit only if the link is reliably fast.
  The old hold-jog needs a fresh observation at start and refuses from below the tip floor ("TCP floor reached or invalid"), which is why R (up) failed and lagged. Not yet tried on the arm.
- The dashboard on the server is newer than the repo copy: its `operator_commands` include `motion/input` and `control/admin-release`; the `/stack` game uses these (its JS: `/static/stack.js`, `motion-input.js`). Not yet used by us.
- The arm is in another country. Tailscale was relayed (DERP Frankfurt) with 360-900 ms spikes, then 40-55 ms. During spikes heartbeats
  (1.5 s lease), 0.75 s observation freshness and 100 ms renewals fail: "controller unreachable", "Control lost", "A fresh observation is
  required to start hold-jog", keys doing nothing. Speed tests (bandwidth) do not reflect this; check `tailscale ping 100.85.56.14`.
  Multiple camera streams can saturate the server's upload; the UI now polls snapshots. A direct Tailscale path was never established.
- If spikes return: run the control loop next to the arm (an agent process on the server, localhost to the controller, with a dead-man timeout;
  browser sends only key down/up and high-level (u,v) requests). Proposed but NOT built; needs the user's OK to run anything on the server, and
  someone physically at the arm able to cut power.

## Next steps
1. Finish calibration (all 9 points, fixed wrist pose), review residuals, save `calibration.json`.
2. `goto.py --dry-run` then supervised real goto at the centre; tune `travel_height` (0.03), `goto_offset` (0.005; consider 0 or negative given the floor).
3. Build the pick-and-place service (HTTP `/select`, `/reset`, `/status`, `/stop`; state idle -> pick set -> place set -> executing -> done; lock selections
   while moving; clamp to the rectangle) wrapping `Planner.plan` + `run_plan`; tune `gripper_open/closed`, `grasp_offset` with a real block.
4. Integrate the teammate's eye-tracking output (contract in `eye_tracking/README.md`), add overlay/reset to the web UI, optional colour snapping.
5. Keep a mouse-click input that sends the same `(u,v)` messages as a fallback demo.
