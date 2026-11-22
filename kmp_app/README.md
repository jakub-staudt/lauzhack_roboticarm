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

## Setup

The app itself is Kotlin; the eye tracker and the simulator are Python and run next to it. The app finds the project
folder (from `GAZE_REPO`, the working directory, or `~/Documents/LauzHack2026/lauzhack_roboticarm` and similar) and
starts both if they are not already running; their output goes to `<temp>/GazeStation/*.log`.

### Python parts (once per machine; Python 3.11, from the repository root)

macOS / Linux:

```sh
python3.11 -m venv .venv      && .venv/bin/pip install -r requirements.txt             # eye tracker (EyeGestures)
python3.11 -m venv .venv-sim  && .venv-sim/bin/pip install -r simulator/requirements.txt  # simulator (MuJoCo)
```

Windows 10 (PowerShell or cmd):

```bat
py -3.11 -m venv .venv      && .venv\Scripts\pip install -r requirements.txt
py -3.11 -m venv .venv-sim  && .venv-sim\Scripts\pip install -r simulator\requirements.txt
```

### macOS

* **Installed app:** `./gradlew :composeApp:packageMacDmg` builds `composeApp/build/compose/binaries/main/dmg/GazeStation-1.0.0.dmg`.
  Open it and drag GazeStation to Applications. The first start: right-click > Open (it is not notarised), then allow
  the camera when macOS asks. Use `packageMacDmg`, not `packageDmg`: it signs the app with `entitlements.plist`, without
  which macOS silently refuses the camera to the eye tracker.
* **From the checkout:** `./run.command` in Terminal (starts the eye tracker from Terminal, which needs camera access,
  then `./gradlew run`). Needs JDK 17 (`brew install openjdk@17`, the script finds it).

### Windows 10

**Installed from the .msi? Follow [INSTALL_WINDOWS.md](INSTALL_WINDOWS.md)** (Python, project folder, camera).


* **Installed app:** build `./gradlew :composeApp:packageMsi` on a Windows PC (or take it from CI, below). It installs per
  user (no admin) with a Start menu entry. Allow desktop apps to use the camera: Settings > Privacy > Camera.
* **From the checkout:** `run.bat` (starts the simulator and the eye tracker in their own windows, then the app). Needs
  JDK 17 (e.g. Temurin) with `JAVA_HOME` set.

### Ubuntu (22.04 / 24.04)

**Follow [INSTALL_LINUX.md](INSTALL_LINUX.md)** (the .deb, Python 3.11 from deadsnakes, project folder, camera).
Build the .deb yourself with `./gradlew :composeApp:packageDeb` on Ubuntu (or take it from CI).

### Options

| What | How |
|---|---|
| Other servers | `--args="http://SIM:8810 http://GAZE:8820"`, or `GAZE_STATION_URL` / `GAZE_SERVICE_URL` |
| Eye camera | `GAZE_CAMERA=1` (index; with a USB camera plugged in the built-in one may move) |
| Project folder | `GAZE_REPO=/path/to/lauzhack_roboticarm` (needed when the installed app cannot find it) |

### Installers from CI

`.github/workflows/kmp-desktop.yml` builds the macOS (Apple Silicon and Intel), Windows and Ubuntu installers on GitHub Actions:
run it from the Actions tab, or push a tag such as `kmp-v1.1.0` to publish them as a release. Packaging always uses an
Eclipse Temurin 17 that Gradle downloads itself (Homebrew's JDK is refused by jpackage).

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
    Main.kt                        the two windows, hidden pointer in the gaze window
    LocalServices.kt               finds the project and starts the eye tracker / simulator if needed
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

## Android and iOS (next)

All UI and logic are in `commonMain` (no JVM-only code); only `Platform.desktop.kt` (image decoding), `Main.kt` and
`LocalServices.kt` are desktop-specific. To add the mobile targets:

1. **Android:** install Android Studio (SDK), add the `com.android.application` plugin and `androidTarget()`, an
   `androidMain` with `MainActivity` (`setContent { App(...) }`), `actual fun decodeImage` with `BitmapFactory`, and the
   `ktor-client-okhttp` engine. The phone cannot start Python, so the simulator and the gaze service run on a laptop
   and the app connects over the network (Tailscale or Wi-Fi): `--host 0.0.0.0` on the servers.
2. **iOS:** needs a Mac with full Xcode (this one has only the Command Line Tools). Add `iosArm64()` /
   `iosSimulatorArm64()` with a framework, an `iosApp` Xcode project, `actual fun decodeImage` via Skia
   (`Image.makeFromEncoded`) and the `ktor-client-darwin` engine.
3. **Eye tracking on a phone:** EyeGestures is Python, so either stream the phone's front camera to the laptop's gaze
   service, or use the platform face tracking (ARKit on iOS, ML Kit / MediaPipe on Android) and send the gaze point.
