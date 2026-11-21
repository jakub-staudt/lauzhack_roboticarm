# lauzkach_eye_robot_control

Separate project for the eye-controlled robot arm. It lives next to the RAMION software and does **not** change it.
Step 1 (this version): find the **left rectangle** (the 20 x 20 cm square) in the overhead camera and draw what the
detector believes on a web page, so it can be checked by eye.

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

Nothing of this project runs on the RAMION server yet. When it does, it will be its own folder and unprivileged user, not
started at boot, and it will not touch `/srv/ramion`, `/etc/ramion` or any `ramion-*` service.
