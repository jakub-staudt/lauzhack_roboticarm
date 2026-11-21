# Gaze station: Kotlin Multiplatform UI

A Compose Multiplatform desktop app for the gaze station simulator: the same tap-to-move workflow and look as
`simulator/web/static/` (`index.html`, `app.js`, `style.css`), talking to the same server API.

## Run

1. Start the simulator server (from the repository root, see `simulator/README.md`):

   ```sh
   python3 -m venv .venv-sim && .venv-sim/bin/pip install -r simulator/requirements.txt
   .venv-sim/bin/python -m simulator.web.server        # http://127.0.0.1:8810
   ```

2. Run the app (needs JDK 17; on macOS `brew install openjdk@17`):

   ```sh
   cd kmp_app
   export JAVA_HOME=$(/usr/libexec/java_home -v 17)    # or: $(brew --prefix openjdk@17)/libexec/openjdk.jdk/Contents/Home
   ./gradlew :composeApp:run
   ```

   Another server: `./gradlew :composeApp:run --args="http://HOST:8810"` or `GAZE_STATION_URL=...`.

The first build downloads Gradle and the Compose libraries (a few hundred MB).

## Use

- **Move a block**: tap a cube, then a destination inside the block. **Ask before moving** adds a Go step.
- **Move the gripper**: tap a spot and the gripper goes over it.
- **Stop** (or Esc) freezes the arm, **Home** returns it, **Reset blocks** puts the cubes back.
- Red dots: spots the arm can't reach with a top-down grasp. Coordinates are cm in the block's frame,
  (0, 0) near-left to (20, 20) far-right, from the robot's point of view.

## Layout

```
composeApp/src/
  commonMain/kotlin/ch/lauzhack/gaze/
    api/Models.kt, api/StationApi.kt   server API (Ktor + kotlinx.serialization)
    Geometry.kt                        pixels <-> world <-> block cm (port of simulator/frame.py)
    StationController.kt               UI state and actions (port of app.js)
    ui/Theme.kt                        colours, radii from style.css (light + dark)
    ui/App.kt, CameraView.kt, ControlPanel.kt
    Platform.kt                        expect decodeImage()
  desktopMain/kotlin/ch/lauzhack/gaze/
    Main.kt                            window, Esc = Stop, server URL
    Platform.desktop.kt                Skia image decoding
    Snapshot.kt                        dev tool: render the UI off-screen to PNG
```

Everything except image decoding and the entry point is in `commonMain`, so Android / iOS / web targets can be added
later by adding the target in `composeApp/build.gradle.kts` and an `actual fun decodeImage` for it.

## Packaging

`./gradlew :composeApp:packageDmg` (macOS), `packageMsi` (Windows), `packageDeb` (Linux): each must be built on its own OS.

## Off-screen screenshot

With the server running: `./gradlew :composeApp:snapshot --args="out-dir http://127.0.0.1:8810 --select"` writes
light and dark PNGs (`--select` picks a cube and a destination with "Ask before moving" on, so nothing moves).
