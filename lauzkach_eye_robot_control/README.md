# lauzkach_eye_robot_control

> **Status (2026-11-22): you can pick and place blocks with the real arm using step buttons.**
> The camera page has a "Grasp a block (REAL arm), one supervised step at a time" card: Open gripper, Hover over the clicked/selected block,
> Lower to grasp height, Grip, Lift, Put back and release, plus Go to park position / Save current arm pose as park. Blocks are found by colour in
> the overhead picture (turned square outline), the hover centres the middle of the open jaws on the block and turns the wrist to its faces, and the
> gripper camera stops the lowering when the block's lower edge reaches the bottom of its picture. Each step is pressed and confirmed by a person;
> a fully automatic pick-and-place (and the eye-tracking input) is not built yet. Settings: `grasp_profiles.json` (`centred`, `sim_elite`, `current`);
> park pose: `park_pose.json`; how the arm agent runs on the server: `docs/AGENT_DEPLOY.md`.

Separate project for the eye-controlled robot arm. It lives next to the RAMION software and does **not** change it.
Step 1 (this version): find the **left rectangle** (the 20 x 20 cm square) in the overhead camera and draw what the
detector believes on a web page, so it can be checked by eye.

## Gaze page: `/gaze` (full-screen camera, controlled by the eyes)
`http://127.0.0.1:8820/gaze` shows the overhead camera across the whole screen. The eye tracker (`gaze_service`, EyeGestures on the laptop's webcam)
gives the gaze point; the page does the rest: **look at a block for 2 s** to pick it, **look at a free spot inside the rectangle for 2 s** to choose
where it goes, then look at **Select** (1.5 s) and confirm with **Yes, move it**; the arm runs every step by itself. Buttons (look at them for 1.5 s, or click):
**Select** (move the chosen block to the chosen spot), **Restart selection**, **Cancel** (stop the arm; Esc does the same) and **Calibrate eyes**
(the tracker's own 25-dot calibration, then the 5-cross correction; K does the same). Mouse clicks work too. F = full screen, R = restart.

Start order: the arm agent (server), then
```bash
python -m gaze_service.server --port 8830          # the eye tracker; add --mock to test without a camera (POST /api/mock {"x":..,"y":..})
python -m lauzkach_eye_robot_control.server --port 8820 ... --tracker http://127.0.0.1:8830
```
The tracker needs its own Python environment (`python3 -m venv gaze_service/.venv && gaze_service/.venv/bin/pip install -r requirements.txt`).

## Safety: read-only

The page server only sends HTTP `GET` requests for camera pictures. It never talks to the arm controller (`:8765`), never
claims control, never moves anything. The camera address is fixed when the server starts.

## Run it

```sh
cd lauzkach_eye_robot_control
python3 -m venv .venv && .venv/bin/pip install -r requirements.txt
.venv/bin/python -m lauzkach_eye_robot_control.server            # live overhead camera -> http://127.0.0.1:8820
.venv/bin/python -m lauzkach_eye_robot_control.server --image tests/fixtures/overhead_1.jpg   # a saved picture instead
```

Options: `--camera URL` (default `http://100.85.56.14:8780/cameras/overhead.mjpg?fps=2&profile=preview`), `--host`, `--port`.
The camera is reached through the RAMION dashboard over Tailscale; 2 fps is gentle on a slow link.

The page shows the camera picture with the tags, the lines used, the green rectangle (corners TL, TR, BR, BL), the
rectangle straightened into a square, and a checklist of what was found. Each layer can be switched on or off, and
**Pause** freezes the picture to inspect it.

## Start-up calibration (5 s)

The camera and the table do not move, so the rectangle is measured once. At start-up the page counts down 5 seconds
while it collects detections, then **locks** the rectangle: the median of the collected frames, with odd frames dropped.
After that the outline is fixed (no jitter) and the live detection only acts as a **drift check**: if the tags or lines
move more than 6 px for several frames, the page warns that something moved. **Recalibrate (5 s)** starts the count again.
Keep the table and camera still during the count; moving the arm out of the way lets the left line show.

## How the rectangle is found (`detect.py`)

1. **AprilTags** (family 36h11) tell which side is which and which horizontal lines belong to the rectangle: only the lines
   between the top and bottom tags count, so the paper edges outside them are ignored. Seen so far: 43 (top right),
   41 (bottom right), 42 (top left).
2. **Thin dark lines** are found with a black-hat filter and a Hough transform. Thick dark things (cables, motors) are
   removed first, and gaps are bridged so faint dotted lines become continuous.
3. The top and bottom lines and the **middle** vertical line are required. The vertical lines are named by where they sit
   between the tag columns.
4. **Left edge:** the printed left line is a thin *dotted* line and is partly hidden by the arm. When it is found (and it is
   in the right place, leans like the other vertical lines and is not pitch black like a cable) it is used. Otherwise
   the left edge is **estimated** from the middle and right lines; on the real table that estimate was 1-2 cm off, so the
   page says clearly when it had to be used.
5. The four corners are mapped to a square to produce the cropped view.

## The right-angle model (`grid.py`)

Every line on the paper meets the next at a right angle; only the camera's perspective makes the grid look skewed. So all
detected line pixels must fit **one perspective model of a right-angled grid** (top line y = 0 cm, bottom y = 20, left x = 0,
middle x = 20, right x = 20 + the right square's width). That one fit gives:

* the exact corners of the 20 x 20 cm square, with right angles by construction;
* the mapping from picture pixels to **table centimetres** (hover the picture on the page to read it, with a 5 cm grid
  drawn inside the rectangle to judge the perspective by eye);
* a consistency check: on the real camera the lines fit the grid to about **1 mm**, so lens distortion is not a problem.

**The right square is not 20 cm wide.** Fitted on live frames with all five lines visible it comes out at about **21.1 cm**
(+-0.24 across 9 frames; 22.0 on one frame). That is `RIGHT_SQUARE_CM` in `grid.py`. It is only used to predict the left
edge when the left line is hidden: with equal squares assumed, that prediction was 1-2 cm off; with 21.1 it is within a few
millimetres. **Please measure the right rectangle on the paper** and put the real value in `RIGHT_SQUARE_CM`.

## Drive panel: click on the camera, move the simulated arm

Below the picture, the page can drive the **simulator** from clicks on the real camera picture (start the simulator first:
`python -m simulator.web.server` from the repo root, then this server).

```sh
.venv/bin/python -m lauzkach_eye_robot_control.server            # --simulator URL, --robot-points FILE, --robot-faces up|right|down|left
```

* **Calibration comes from the robot's own recorded points** (`robotic_arm/calibration_poses.json`, the corners touched with
  the real arm's tip), not from the simulator's drawn blocks, because the paper may not be exactly 20 x 20 cm. The fit is a
  rotation plus a separate stretch per axis (the lines are orthogonal) and the page shows the resulting size and the fit error.
  With today's three points it says the rectangle is about **17.3 x 22.4 cm** (-13% / +12%), more than a slight stretch, with the
  corners 4.4 degrees away from a right angle: the points were recorded at different gripper tilts and should probably be re-recorded.
* **Which way the robot faces** in the picture is a setting (default `up`: the robot's near-left corner is the picture's bottom
  left). The corner markers on the picture show the assumption; change it in the panel if it is wrong.
* **Click target / Corners step by step:** a click only chooses a target; **Move simulator here** moves. **Next corner** steps
  through the four corners, one press per corner.
* **Stop motion (Esc)** is a soft stop: halts the arm and voids moves still in flight, latches nothing.
* Lag protections and the plan for running this next to the arm: `docs/LAG_STRATEGY.md`.

## Closed-loop correction: use the camera to see where the tip really is

The arm's own calibration is accurate to about a centimetre; the camera sees the board to about a millimetre. With
**closed-loop correction** ticked, a tapped point (or a corner) is handled like this: move there, wait until the arm is still,
look at where the tip really is, correct the commanded point by the error, and repeat (at most 4 corrections, stopping as soon as
the tip is within 3 mm). The loop (`servo.py`) has hard limits so a wrong observation can never make the arm run away: each
correction is capped at 15 mm, the commanded point never strays more than 30 mm from what was tapped, observations the
observer is not sure about are ignored, **Stop** ends the loop before its next move, and it gives up instead of chasing noise.

**Works and tested**
* the loop itself, against a simulated arm that misses by 1.4 to 2.5 cm: it converges to under 3 mm, and an observer that lies,
  goes blind or is unsure cannot make it run away (`tests/test_servo.py`);
* the pixel + height -> board position maths (`projection.py`), and the **feature model** (`feature_model.py`) that learns, from
  data, where a tracked part sits on the gripper, how height shifts it in the camera view, and a small correction to the arm
  calibration; on synthetic data it recovers a known set-up and locates unseen tip positions to within 3 mm;
* through the page, on the simulator with a stand-in camera: `server --closed-loop-demo 12,-8` pretends the board is 12, -8 mm off
  from the arm's coordinates. A tap without the correction lands 1.6 cm from the target; with it, two corrections bring the tip to
  1.2 mm.

**Not working yet: finding the tip on the real camera.** Checked on a real picture:
* the white arm and the grey paper cannot be told apart by brightness (paper 140-155, arm plastic 107-160), so a tip cannot be found
  "by itself";
* the markerless dark-feature observer (`DarkBlobObserver`) picked the black cable hanging off the arm instead of a rigid part of
  the gripper, so it is experimental and capped below the confidence the loop accepts: it cannot drive a move.

**What it needs: a marker on the gripper**, a small bright sticker (default colour: magenta) or an AprilTag, attached once by
someone at the arm. Then:
1. `tools/record_pose_samples.py` (read-only) while the arm is jogged over the board: pictures + joint angles;
2. `tools/fit_feature_model.py` (run in the `robotic_arm` environment, which now also has OpenCV) fits the model;
3. `CameraTipObserver` (`closedloop.py`) turns a picture + the arm's pose into the tip's position on the board; it refuses a
   detection more than 4 cm from where the arm believes its tip is.

The real arm is not connected to the drive panel yet, so the real camera observer is not wired in; only the simulator demo runs.

## Tests

```sh
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 .venv/bin/python -m pytest -q tests
```

They draw a synthetic table with known corners (real AprilTags and thin lines) and require the detector to find them,
with the left line visible and hidden, and with a thick cable across the table. Three real frames from the camera are in
`tests/fixtures/`.

## Known limits (what to check next)

* **The left line is dotted and sometimes hidden by the arm.** With the arm in its usual place it was found in 27 of 28 live
  frames; if it is hidden the estimate (1-2 cm off) is used and the page shows an amber banner. `tests/fixtures/overhead_arm_away.jpg`
  is a frame with the arm moved away.
* **Lens distortion is not modelled**, and does not need to be for this: the five lines fit one perspective model to about 1 mm.
* The detector assumes the paper layout of the RAMION table: AprilTags 36h11 at the corners, two horizontal lines and
  left / middle / right vertical lines.

## Not done yet / on the server

The drive panel drives the **simulator only**; the real arm is not connected. Nothing of this project runs on the RAMION server yet. When it does, it will be its own folder and unprivileged user, not
started at boot, and it will not touch `/srv/ramion`, `/etc/ramion` or any `ramion-*` service.

## Real arm (five buttons, off by default)
`--real-arm` (needs the `robotic_arm` venv; `--dashboard` defaults to the RAMION dashboard) lets the five buttons (four corners and Park)
move the physical arm. Hover only (never below 3 cm), rise → glide at 5 cm → straight down, joint steps ≤ 3° with a rest between steps,
control claimed only for the move, refused if the gripper is open / someone else has control / e-stop latched / a step would leave the
controller's joint limits. Every move needs a browser confirmation (`confirm_real`); Stop (Esc) and the dead-man hold the arm.
Tested only against a fake dashboard (`tests/test_realarm.py`); the first real moves must be done supervised.
