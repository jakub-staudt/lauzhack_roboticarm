package ch.lauzhack.gaze

import androidx.compose.runtime.Composable
import androidx.compose.runtime.DisposableEffect
import androidx.compose.runtime.LaunchedEffect
import androidx.compose.runtime.remember
import androidx.compose.runtime.rememberCoroutineScope
import androidx.compose.ui.geometry.Offset
import androidx.compose.ui.input.key.Key
import androidx.compose.ui.input.key.KeyEventType
import androidx.compose.ui.input.key.key
import androidx.compose.ui.input.key.type
import androidx.compose.ui.platform.LocalDensity
import androidx.compose.ui.unit.DpOffset
import androidx.compose.ui.unit.dp
import androidx.compose.ui.window.Window
import androidx.compose.ui.window.WindowPlacement
import androidx.compose.ui.window.WindowPosition
import androidx.compose.ui.window.WindowState
import androidx.compose.ui.window.application
import androidx.compose.ui.window.rememberWindowState
import ch.lauzhack.gaze.api.GazeApi
import ch.lauzhack.gaze.api.StationApi
import ch.lauzhack.gaze.ui.App
import ch.lauzhack.gaze.ui.SceneScreen
import java.awt.GraphicsEnvironment
import java.awt.Point
import java.awt.Rectangle
import java.awt.Toolkit
import java.awt.image.BufferedImage

/**
 * Desktop entry point. Two modes:
 *
 * - **Real arm** (`--real` or GAZE_ARM=real; run_real.command): one full-screen window with the gaze scene on the live
 *   overhead camera from Poland. Blocks come from the camera page server on this laptop ($GAZE_ARM_URL, default
 *   http://127.0.0.1:8820, `lauzkach_eye_robot_control.server --real-agent ...`), which moves the real SO-101 through
 *   the arm agent. Eye tracking: $GAZE_SERVICE_URL, default http://127.0.0.1:8830 (the Logitech camera). Esc stops the arm.
 * - **Simulator** (default): two windows on the MuJoCo simulator, "Gaze station" (simulation + controls) and
 *   "Eye tracking" (the gaze scene). Station server: first argument / $GAZE_STATION_URL / http://127.0.0.1:8810;
 *   gaze service: second argument / $GAZE_SERVICE_URL / http://127.0.0.1:8820.
 *
 * Local services that are not answering are started from the repository (see LocalServices) and stopped on exit.
 */
fun main(args: Array<String>) {
    val real = "--real" in args || System.getenv("GAZE_ARM") == "real"
    val rest = args.filter { !it.startsWith("--") }
    if (real) realArm() else simulator(rest)
}

// The eye camera sits next to the main display, so gaze is measured on that screen.
private fun screen(): Rectangle = GraphicsEnvironment.getLocalGraphicsEnvironment().defaultScreenDevice.defaultConfiguration.bounds

private fun realArm() {
    val armUrl = System.getenv("GAZE_ARM_URL") ?: "http://127.0.0.1:8820"
    val gazeUrl = System.getenv("GAZE_SERVICE_URL") ?: "http://127.0.0.1:8830"
    application {
        val scope = rememberCoroutineScope()
        val gaze = rememberGaze(gazeUrl, scope)
        val source = remember { RealArmSource(armUrl.trimEnd('/'), scope).also { it.start() } }
        val scene = remember { SceneController(source, gaze, scope).also { it.start() } }
        val state = rememberWindowState(placement = WindowPlacement.Fullscreen)
        GazeWindow("Gaze pick & place: real arm", state, scene, gaze, ::exitApplication) { scene.stopArm() }
    }
}

private fun simulator(args: List<String>) {
    val url = args.getOrNull(0) ?: System.getenv("GAZE_STATION_URL") ?: "http://127.0.0.1:8810"
    val gazeUrl = args.getOrNull(1) ?: System.getenv("GAZE_SERVICE_URL") ?: "http://127.0.0.1:8820"
    application {
        val scope = rememberCoroutineScope()
        val station = remember {
            LocalServices.startSimulator(url)
            StationController(StationApi(url.trimEnd('/')), scope).also { it.start() }
        }
        val gaze = rememberGaze(gazeUrl, scope)
        val scene = remember { SceneController(SimSource(station), gaze, scope).also { it.start() } }

        // Window 1: simulation + controls (the original UI)
        Window(
            onCloseRequest = ::exitApplication,
            title = "Gaze station",
            state = rememberWindowState(width = 1280.dp, height = 860.dp, position = WindowPosition(40.dp, 40.dp)),
            onPreviewKeyEvent = { e ->
                // Esc freezes the arm, as on the web page.
                if (e.type == KeyEventType.KeyDown && e.key == Key.Escape) { station.stop(); true } else false
            },
        ) {
            App(station)
        }

        // Window 2: eye tracking (goes full screen while calibrating: the dots span the whole screen)
        val gazeState = rememberWindowState(width = 1100.dp, height = 720.dp, position = WindowPosition(160.dp, 120.dp))
        val calibrating = scene.stage == Stage.Calibrating
        LaunchedEffect(calibrating) {
            gazeState.placement = if (calibrating) WindowPlacement.Fullscreen else WindowPlacement.Floating
        }
        GazeWindow("Eye tracking", gazeState, scene, gaze, ::exitApplication) { station.stop() }
    }
}

@Composable
private fun rememberGaze(gazeUrl: String, scope: kotlinx.coroutines.CoroutineScope): GazeController = remember {
    GazeController(GazeApi(gazeUrl.trimEnd('/')), scope).also {
        it.screenSize = { screen().let { b -> b.width to b.height } }
        it.launchNote = LocalServices.startGazeService(gazeUrl)
        it.start()
    }
}

/** The gaze scene window: no mouse pointer (the gaze circle is the cursor), keys only as the prompts ask. */
@Composable
private fun GazeWindow(
    title: String,
    state: WindowState,
    scene: SceneController,
    gaze: GazeController,
    onClose: () -> Unit,
    onStop: () -> Unit,
) {
    Window(
        onCloseRequest = onClose,
        title = title,
        state = state,
        onPreviewKeyEvent = { e ->
            if (e.type != KeyEventType.KeyDown) return@Window false
            when (e.key) {
                Key.Escape -> { onStop(); if (gaze.calibrating) gaze.cancelCalibration() }   // Esc stops the arm
                Key.Spacebar -> scene.onKey(' ')
                Key.L -> scene.onKey('L')
                Key.K -> scene.onKey('K')
                Key.C -> scene.onKey('C')
                Key.D -> scene.onKey('D')
                Key.G -> scene.onKey('G')
                else -> return@Window false
            }
            true
        },
    ) {
        val density = LocalDensity.current.density
        val window = this.window
        DisposableEffect(window, density) {
            window.contentPane.cursor = Toolkit.getDefaultToolkit()
                .createCustomCursor(BufferedImage(1, 1, BufferedImage.TYPE_INT_ARGB), Point(0, 0), "none")
            gaze.mapper = { nx, ny ->
                if (!window.isShowing) null else {
                    val b = screen()
                    val c = window.contentPane.locationOnScreen
                    Offset((((b.x + nx * b.width) - c.x) * density).toFloat(), (((b.y + ny * b.height) - c.y) * density).toFloat())
                }
            }
            onDispose { gaze.mapper = null }
        }
        SceneScreen(scene)
    }
}
