package ch.lauzhack.gaze

import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.setValue
import androidx.compose.ui.geometry.Offset
import ch.lauzhack.gaze.api.GazeApi
import ch.lauzhack.gaze.api.GazeStatus
import kotlinx.coroutines.CancellationException
import kotlinx.coroutines.CoroutineScope
import kotlinx.coroutines.delay
import kotlinx.coroutines.launch

/** Polls the gaze service (`python -m gaze_service.server`) and maps the gaze onto the window. */
class GazeController(private val api: GazeApi, private val scope: CoroutineScope) {
    var status by mutableStateOf<GazeStatus?>(null); private set
    var serviceUp by mutableStateOf(false); private set
    var message by mutableStateOf<String?>(null); private set
    /** What the platform did about starting the gaze service, shown while waiting for it. */
    var launchNote by mutableStateOf<String?>(null)

    /** Gaze in window pixels (null: no face, not tracking, or off the window). */
    var point by mutableStateOf<Offset?>(null); private set

    /** Screen fractions -> window pixels; set by the platform window. */
    var mapper: ((Double, Double) -> Offset?)? = null
    /** Screen size in points, sent with calibration requests. */
    var screenSize: () -> Pair<Int, Int> = { 1440 to 900 }

    val baseUrl: String get() = api.baseUrl
    val tracking: Boolean get() = status?.phase == "tracking" && status?.calibrated == true
    val calibrating: Boolean get() = status?.phase == "calibrating" || status?.phase == "correcting"
    val moving: Boolean get() = status?.moving == true
    val face: Boolean get() = status?.face == true

    fun start() {
        scope.launch {
            while (true) {
                try {
                    status = api.gaze()
                    serviceUp = true
                } catch (e: CancellationException) {
                    throw e
                } catch (e: Exception) {
                    serviceUp = false
                    status = null
                }
                val s = status
                point = if (s != null && tracking && s.x != null && s.y != null) mapper?.invoke(s.x, s.y) else null
                delay(33)
            }
        }
    }

    fun calibrate() = call { api.calibrate(screenSize()) }
    fun correct() = call { api.correct(screenSize()) }
    fun loadSaved() = call { api.loadSaved(screenSize()) }
    fun cancelCalibration() = call { api.cancel(screenSize()) }

    private fun call(action: suspend () -> GazeStatus) {
        scope.launch {
            try {
                status = action()
                message = null
            } catch (e: CancellationException) {
                throw e
            } catch (e: Exception) {
                message = e.message ?: "Gaze service unavailable."
            }
        }
    }
}
