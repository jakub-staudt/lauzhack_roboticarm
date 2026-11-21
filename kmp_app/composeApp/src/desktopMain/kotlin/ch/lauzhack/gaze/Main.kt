package ch.lauzhack.gaze

import androidx.compose.runtime.remember
import androidx.compose.runtime.rememberCoroutineScope
import androidx.compose.ui.input.key.Key
import androidx.compose.ui.input.key.KeyEventType
import androidx.compose.ui.input.key.key
import androidx.compose.ui.input.key.type
import androidx.compose.ui.unit.dp
import androidx.compose.ui.window.Window
import androidx.compose.ui.window.application
import androidx.compose.ui.window.rememberWindowState
import ch.lauzhack.gaze.api.StationApi
import ch.lauzhack.gaze.ui.App

/**
 * Desktop entry point. The station server URL comes from the first argument, then $GAZE_STATION_URL,
 * then http://127.0.0.1:8810 (`python -m simulator.web.server`).
 */
fun main(args: Array<String>) {
    val url = args.firstOrNull() ?: System.getenv("GAZE_STATION_URL") ?: "http://127.0.0.1:8810"
    application {
        val scope = rememberCoroutineScope()
        val controller = remember {
            StationController(StationApi(url.trimEnd('/')), scope).also { it.start() }
        }
        Window(
            onCloseRequest = ::exitApplication,
            title = "Gaze station",
            state = rememberWindowState(width = 1280.dp, height = 860.dp),
            onPreviewKeyEvent = { e ->
                // Esc freezes the arm, as on the web page.
                if (e.type == KeyEventType.KeyDown && e.key == Key.Escape) { controller.stop(); true } else false
            },
        ) {
            App(controller)
        }
    }
}
