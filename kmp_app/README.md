# Gaze pick & place: Kotlin Multiplatform app

A Compose Multiplatform desktop app for the simulated SO-101 (`simulator/web/server.py`). It opens two windows on the
same simulator:

- **Gaze station**: the simulation on the left, the controls on the right (tap a block, tap where it goes, Go; Home,
  Reset blocks, Stop; Esc stops the arm). Same layout as the simulator web page.
- **Eye tracking**: the dev branch prototype (`gaze_scene_v2.py`) driven by the MacBook camera: the overhead camera,
  the gaze circle as the only cursor, dwell to choose an object, an operation and a destination. It goes full screen
  while calibrating (the dots span the whole screen) and back to a window afterwards.

The app starts the eye tracker (`gaze_service`) itself on camera 0 (the built-in MacBook camera) if it is not already
running. macOS only lets it see the camera when the app was launched from an app with camera permission, so start it
from Terminal (`./run.command`) and allow Terminal to use the camera. With the Logitech plugged in the camera numbers
can change: `GAZE_CAMERA=1 ./run.command`.

## Run

1. Simulator (from the repository root, see `simulator/README.md`):

   ```sh
   .venv-sim/bin/python -m simulator.web.server        # http://127.0.0.1:8810
   ```

2. The app, from Terminal (JDK 17; on macOS `brew install openjdk@17`). `./run.command` does the following:

   ```sh
   cd kmp_app
   export JAVA_HOME=$(brew --prefix openjdk@17)/libexec/openjdk.jdk/Contents/Home
   ./gradlew :composeApp:run
   ```

   Other servers: `./gradlew :composeApp:run --args="http://SIM:8810 http://GAZE:8820"`, or `GAZE_STATION_URL` /
   `GAZE_SERVICE_URL`.

## Eye tracking window (as in gaze_scene_v2.py)

1. **Calibration**: "Calibration found: L = load, K = calibrate again", or "Press SPACE to start". 25 dots, then
   "Look at the world", SPACE, and 5 crosses for the gaze correction.
2. **Look at an object** (a block) for 2 s: it gets a blue dwell ring, then an orange "1 red block" circle.
3. **Look at an operation button** for 1.5 s: *Pick & place*, *Point at it* or *Cancel*. *Bring to me* and
   *Push towards* from the prototype need the real arm and are left out.
4. **Look at a destination zone** (A1–C2 on the workspace, occupied or unreachable zones are greyed) for 2 s, or at
   Cancel. The command is sent to the simulator straight away and shown as "Sent: …" for 3 s.

Keys (in the eye tracking window): ESC stops the simulated arm (and cancels a calibration), C cancels, D redoes the gaze correction, K recalibrates,
G hides/shows the gaze circle. The orange gaze circle turns grey while the eyes are moving (no dwell then).

## Layout

```
composeApp/src/
  commonMain/kotlin/ch/lauzhack/gaze/
    api/Models.kt, StationApi.kt   simulator API     api/GazeApi.kt   gaze service API
    Geometry.kt                    pixels <-> world <-> workspace cm (port of simulator/frame.py)
    StationController.kt           simulator state and the click-to-move actions (control window)
    GazeController.kt              gaze service polling, screen -> window mapping
    SceneController.kt             the gaze_scene_v2.py loop: stages, targets, dwell, zones, buttons
    ui/App.kt, CameraView.kt, ControlPanel.kt, Theme.kt   control window (simulation left, controls right)
    ui/SceneScreen.kt              eye tracking window (colours, rings, banner as in the prototype)
  desktopMain/kotlin/ch/lauzhack/gaze/
    Main.kt                        the two windows, gaze service auto-start, hidden pointer in the gaze window
    Snapshot.kt                    dev tool: scripted run off-screen with a mock gaze, PNG per step
```

## Off-screen check

```sh
.venv/bin/python -m gaze_service.server --mock --port 8821     # fresh, uncalibrated
./gradlew :composeApp:snapshot --args="out-dir http://127.0.0.1:8810 http://127.0.0.1:8821"
```

Goes through the prompts, calibration, object, operation and destination by gaze and writes `scene-*.png`
and `control-window.png`.
It really moves the simulated arm.

## Packaging

`./gradlew :composeApp:packageDmg` (macOS), `packageMsi` (Windows), `packageDeb` (Linux), each on its own OS.
