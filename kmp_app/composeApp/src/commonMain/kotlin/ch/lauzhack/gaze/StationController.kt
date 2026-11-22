package ch.lauzhack.gaze

import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.setValue
import androidx.compose.ui.graphics.ImageBitmap
import ch.lauzhack.gaze.api.Cube
import ch.lauzhack.gaze.api.StationApi
import ch.lauzhack.gaze.api.StationConfig
import ch.lauzhack.gaze.api.TaskState
import kotlinx.coroutines.CancellationException
import kotlinx.coroutines.CoroutineScope
import kotlinx.coroutines.delay
import kotlinx.coroutines.launch
import kotlin.math.hypot
import kotlin.time.Duration.Companion.milliseconds
import kotlin.time.TimeSource

/** What a tap on the camera does. */
enum class Mode { MoveBlock, MoveGripper }

/** The chosen cube (snapped to its centre) and destination, in world metres. */
data class Pick(val at: World, val color: String? = null)

/** A short message that overrides the status chip for a few seconds. */
data class Notice(val text: String, val kind: String, val until: TimeSource.Monotonic.ValueTimeMark)

/**
 * All UI state and actions; mirrors the behaviour of simulator/web/static/app.js.
 * Compose reads the `by mutableStateOf` properties directly.
 */
class StationController(private val api: StationApi, private val scope: CoroutineScope) {
    var config by mutableStateOf<StationConfig?>(null); private set
    var geometry by mutableStateOf<Geometry?>(null); private set
    var frame by mutableStateOf<ImageBitmap?>(null); private set
    var cubes by mutableStateOf<List<Cube>>(emptyList()); private set
    var task by mutableStateOf(TaskState(message = "Connecting…")); private set
    var connected by mutableStateOf(false); private set

    var mode by mutableStateOf(Mode.MoveBlock); private set
    var src by mutableStateOf<Pick?>(null); private set
    var dst by mutableStateOf<Pick?>(null); private set
    var askBeforeMoving by mutableStateOf(false)
    var hover by mutableStateOf<String?>(null)
    var notice by mutableStateOf<Notice?>(null); private set

    val baseUrl: String get() = api.baseUrl

    val canGo: Boolean
        get() = !task.busy && dst != null && (mode == Mode.MoveGripper || src != null)

    val hint: String
        get() = if (mode == Mode.MoveBlock) "Tap a block, then tap where it should go."
        else "Tap a spot and the gripper moves over it."

    fun start() {
        scope.launch { loadConfig() }
        scope.launch { pollState() }
        scope.launch { pollFrames() }
        scope.launch { expireNotices() }
    }

    // ------------------------------------------------------------------ server
    private suspend fun loadConfig() {
        while (config == null) {
            try {
                val cfg = api.config()
                config = cfg
                geometry = Geometry(cfg)
            } catch (e: CancellationException) {
                throw e
            } catch (_: Exception) {
                delay(1000)
            }
        }
    }

    private suspend fun pollState() {
        while (true) {
            try {
                val s = api.state()
                task = s.task
                cubes = s.blocks
                connected = true
            } catch (e: CancellationException) {
                throw e
            } catch (_: Exception) {
                connected = false
                task = TaskState(state = "error", message = "Lost connection to the simulator server.")
            }
            delay(300)
        }
    }

    private suspend fun pollFrames() {
        while (true) {
            try {
                frame = decodeImage(api.frame())
                delay(60)
            } catch (e: CancellationException) {
                throw e
            } catch (_: Exception) {
                delay(1000)
            }
        }
    }

    private suspend fun expireNotices() {
        while (true) {
            notice?.let { if (it.until.hasPassedNow()) notice = null }
            delay(200)
        }
    }

    private fun say(text: String, kind: String = "error", ms: Long = 5000) {
        notice = Notice(text, kind, TimeSource.Monotonic.markNow() + ms.milliseconds)
    }

    private fun run(action: suspend () -> TaskState) {
        scope.launch {
            try {
                task = action()
                notice = null
            } catch (e: CancellationException) {
                throw e
            } catch (e: Exception) {
                say(e.message ?: "Request failed.")
            }
        }
    }

    // ------------------------------------------------------------------ taps
    /** A tap on the camera image at pixel (px, py) of the camera frame. */
    fun onTap(px: Double, py: Double) {
        val g = geometry ?: return
        val w = g.toWorld(px, py)
        if (mode == Mode.MoveGripper) {
            dst = Pick(w)
            if (!askBeforeMoving) execute()
            return
        }
        if (src == null || dst != null) {
            // choose the cube (starting over when both were set)
            val near = cubes.filter { !it.held }.minByOrNull { hypot(it.x - w.x, it.y - w.y) }
            if (near == null || hypot(near.x - w.x, near.y - w.y) > SNAP_M) {
                say("No block there. Tap one of the coloured blocks.", ms = 3000)
                return
            }
            src = Pick(World(near.x, near.y), near.color)
            dst = null
        } else {
            if (g.blockAt(w, 1.0) == null) {
                say("Tap the destination inside the block.", ms = 3500)
                return
            }
            dst = Pick(w)
            if (!askBeforeMoving) execute()
        }
    }

    fun onHover(px: Double?, py: Double?) {
        val g = geometry
        hover = if (px == null || py == null || g == null) null else g.describe(g.toWorld(px, py))
    }

    // ------------------------------------------------------------------ actions
    fun execute() {
        val g = geometry ?: return
        if (task.busy) {
            say("The arm is busy. Press Stop first.")
            return
        }
        val d = dst ?: return
        if (mode == Mode.MoveGripper) {
            run { api.goTo(g.toPixel(d.at)) }
        } else {
            val s = src ?: return
            run { api.pickAndPlace(g.toPixel(s.at), g.toPixel(d.at)) }
        }
    }

    fun changeMode(next: Mode) {
        mode = next
        src = null
        dst = null
    }

    fun clear() {
        src = null
        dst = null
    }

    /** Gaze window: pick the cube at [srcPx] and place it at [dstPx] (camera pixels), right away. */
    fun pickPlace(srcPx: Pair<Double, Double>, dstPx: Pair<Double, Double>) = run { api.pickAndPlace(srcPx, dstPx) }

    /** Gaze window: move the gripper over [px] (camera pixels). */
    fun goToPx(px: Pair<Double, Double>) = run { api.goTo(px) }

    fun stop() = run { api.stop() }

    fun home() = run { api.home() }

    fun resetBlocks() {
        clear()
        run { api.resetBlocks() }
    }

    /** The text and kind ("idle", "running", "done", "error", ...) for the header chip. */
    fun statusChip(): Pair<String, String> {
        notice?.let { return it.text to it.kind }
        val progress = if (task.state == "running" && task.total > 0)
            " · ${task.label} ${(100.0 * task.step / task.total).toInt()}%" else ""
        return (task.message + progress) to task.state
    }

    fun describe(p: Pick?): String = p?.let { geometry?.describe(it.at) } ?: "not chosen"

    companion object {
        const val SNAP_M = 0.035 // a tap this close to a cube selects it (server uses the same value)
    }
}
