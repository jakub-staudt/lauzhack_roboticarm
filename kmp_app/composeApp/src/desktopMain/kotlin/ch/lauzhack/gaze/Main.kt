package ch.lauzhack.gaze

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
 * Desktop entry point: two windows on one simulator.
 * - "Gaze station": the simulation on the left, the controls on the right (click to move; Esc stops the arm).
 * - "Eye tracking": the dev branch gaze scene (gaze_scene_v2.py) driven by the MacBook camera. It goes full screen
 *   while calibrating, since the calibration dots span the whole screen.
 *
 * Station server: first argument, then $GAZE_STATION_URL, then http://127.0.0.1:8810 (`python -m simulator.web.server`).
 * Gaze service: second argument, then $GAZE_SERVICE_URL, then http://127.0.0.1:8820 (`python -m gaze_service.server`).
 *   If nothing answers there, the app starts it from the repository's .venv on camera $GAZE_CAMERA (default 0, the
 *   built-in MacBook camera) and stops it on exit. The camera permission goes to whatever launched the app
 *   (on macOS: run it from Terminal and allow Terminal to use the camera).
 */
fun main(args: Array<String>) {
    val url = args.getOrNull(0) ?: System.getenv("GAZE_STATION_URL") ?: "http://127.0.0.1:8810"
    val gazeUrl = args.getOrNull(1) ?: System.getenv("GAZE_SERVICE_URL") ?: "http://127.0.0.1:8820"
    // The eye camera sits on the main display, so gaze is measured on that screen.
    fun screen(): Rectangle = GraphicsEnvironment.getLocalGraphicsEnvironment().defaultScreenDevice.defaultConfiguration.bounds

    application {
        val scope = rememberCoroutineScope()
        val station = remember { StationController(StationApi(url.trimEnd('/')), scope).also { it.start() } }
        val gaze = remember {
            GazeController(GazeApi(gazeUrl.trimEnd('/')), scope).also {
                it.screenSize = { screen().let { b -> b.width to b.height } }
                it.launchNote = startGazeService(gazeUrl)
                it.start()
            }
        }
        val scene = remember { SceneController(station, gaze, scope).also { it.start() } }

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

        // Window 2: eye tracking (the dev branch gaze scene)
        val gazeState = rememberWindowState(width = 1100.dp, height = 720.dp, position = WindowPosition(160.dp, 120.dp))
        val calibrating = scene.stage == Stage.Calibrating
        LaunchedEffect(calibrating) {
            gazeState.placement = if (calibrating) WindowPlacement.Fullscreen else WindowPlacement.Floating
        }
        Window(
            onCloseRequest = ::exitApplication,
            title = "Eye tracking",
            state = gazeState,
            onPreviewKeyEvent = { e ->
                if (e.type != KeyEventType.KeyDown) return@Window false
                when (e.key) {
                    Key.Escape -> { station.stop(); if (gaze.calibrating) gaze.cancelCalibration() }
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
                // no mouse pointer in this window: the gaze circle is the cursor
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
}

/** Starts `python -m gaze_service.server` if [url] is local and not answering. Returns a line for the waiting screen. */
private fun startGazeService(url: String): String? {
    val uri = runCatching { java.net.URI(url) }.getOrNull() ?: return null
    if (uri.host !in setOf("127.0.0.1", "localhost")) return null
    val port = if (uri.port > 0) uri.port else 8820
    val up = runCatching { java.net.Socket().use { it.connect(java.net.InetSocketAddress(uri.host, port), 300) } }.isSuccess
    if (up) return null
    var dir: java.io.File? = java.io.File(System.getProperty("user.dir")).absoluteFile
    while (dir != null && !java.io.File(dir, "gaze_service/server.py").exists()) dir = dir.parentFile
    val root = dir ?: return "Could not find gaze_service/ to start the eye tracker. Start it by hand."
    val windows = System.getProperty("os.name").lowercase().contains("win")
    val python = java.io.File(root, if (windows) ".venv/Scripts/python.exe" else ".venv/bin/python")
    if (!python.exists()) return "No Python environment at ${python.path} (pip install -r requirements.txt)."
    val camera = System.getenv("GAZE_CAMERA") ?: "0"
    return try {
        val p = ProcessBuilder(python.path, "-m", "gaze_service.server", "--port", port.toString(), "--camera", camera)
            .directory(root).inheritIO().start()
        Runtime.getRuntime().addShutdownHook(Thread { p.destroy() })
        "Starting the eye tracker on camera $camera (MacBook camera)…"
    } catch (e: Exception) {
        "Could not start the eye tracker: ${e.message}"
    }
}
