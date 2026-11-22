package ch.lauzhack.gaze

import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableStateMapOf
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.setValue
import androidx.compose.ui.geometry.Offset
import androidx.compose.ui.geometry.Rect
import androidx.compose.ui.geometry.Size
import kotlinx.coroutines.CoroutineScope
import kotlinx.coroutines.delay
import kotlinx.coroutines.launch
import kotlin.math.exp
import kotlin.math.hypot
import kotlin.math.max
import kotlin.math.min
import kotlin.math.sqrt
import kotlin.time.TimeSource

/** What is on screen: the prompts and calibration of gaze_scene_v2.py, then the scene itself. */
enum class Stage { Waiting, SavedPrompt, CalibIntro, Calibrating, WorldIntro, Running }

/** The three steps of gaze_scene_v2.py (object -> operation -> destination), then "done". */
enum class Flow { Object, Operation, Destination, Done }

/** An operation button; [needsDest] as the third field of OPERATIONS in the prototype. */
data class Operation(val key: String, val label: String, val needsDest: Boolean)

/** A gaze target in screen units (dp): an object, a button or a drop zone. */
data class Target(
    val id: String,
    val center: Offset,
    val need: Double,
    val box: Rect? = null,          // object bounding box -> distance to the box
    val poly: List<Offset>? = null, // zone -> distance to the polygon
)

/**
 * The scene loop of gaze_scene_v2.py: the overhead camera full screen, the gaze circle, dwell rings, operation buttons
 * and drop zones, on the simulator or on the real arm in Poland ([source]). Distances and timings are the prototype's, in dp.
 */
class SceneController(val source: SceneSource, val gaze: GazeController, private val scope: CoroutineScope) {
    // ---- constants from gaze_scene_v2.py
    companion object {
        const val SIGMA = 200.0
        const val ON_INPUT = 0.45
        const val DWELL_OBJECT_S = 2.0
        const val DWELL_BUTTON_S = 1.5
        const val DWELL_SPOT_S = 2.0
        const val ZONE_PENALTY_PX = 10.0
        const val DECAY = 1.5
        const val HYSTERESIS_PX = 20.0
        const val STEP_GRACE_S = 0.8
        const val SHOW_DONE_S = 3.0
        val DEST_ZONES = 2 to 3     // columns, rows
        const val DWELL_STOP_S = 0.8    // stopping must be quicker than starting
    }

    val operations: List<Operation> get() = source.operations

    /** Window size in dp, set by the screen. */
    var screen by mutableStateOf(Size(1440f, 900f))
    var density = 1f

    var stage by mutableStateOf(Stage.Waiting); private set
    var flow by mutableStateOf(Flow.Object); private set
    var chosen by mutableStateOf<Cube?>(null); private set
    /** The arm was busy with our command during the last tick (to notice when it finishes). */
    private var wasBusy = false
    var operation by mutableStateOf<Operation?>(null); private set
    var dest by mutableStateOf<Pair<String, Offset>?>(null); private set   // name, camera px
    var showGaze by mutableStateOf(true); private set
    val dwell = mutableStateMapOf<String, Double>()

    private var worldIntroDone = false
    private var corrected = false
    private var lastOnId: String? = null
    private var graceUntil = 0.0
    private var stateUntil = 0.0
    private val clock = TimeSource.Monotonic.markNow()
    private fun now() = clock.elapsedNow().inWholeMilliseconds / 1000.0

    // ------------------------------------------------------------------ view (fit_view / img2scr)
    data class View(val s: Float, val ox: Float, val oy: Float, val fw: Float, val fh: Float)

    fun view(): View? {
        val (fw, fh) = source.imageSize ?: return null
        val s = min(screen.width / fw, screen.height / fh)
        return View(s, (screen.width - fw * s) / 2, (screen.height - fh * s) / 2, fw, fh)
    }

    fun img2scr(p: Offset, v: View) = Offset(v.ox + p.x * v.s, v.oy + p.y * v.s)

    // ------------------------------------------------------------------ scene contents
    fun objects(): List<Cube> = source.objects()

    /** Bounding box of a cube in camera pixels. */
    fun cubeBox(c: Cube): Rect {
        if (c.box.isEmpty()) return Rect(c.img - Offset(15f, 15f), c.img + Offset(15f, 15f))
        return Rect(c.box.minOf { it.x }, c.box.minOf { it.y }, c.box.maxOf { it.x }, c.box.maxOf { it.y })
    }

    /** The work area as a polygon in camera pixels. */
    fun workArea(): List<Offset> = source.workArea()

    data class Zone(val name: String, val poly: List<Offset>, val img: Offset, val occupied: Boolean, val reachable: Boolean)

    /** Drop zones on the workspace as seen on screen: rows A (top) .. C (bottom), columns 1..2 left to right. */
    fun zones(): List<Zone> {
        val q = workArea()
        if (q.size != 4) return emptyList()
        // order_corners(): top-left, top-right, bottom-right, bottom-left in the camera image
        val tl = q.minBy { it.x + it.y }; val br = q.maxBy { it.x + it.y }
        val tr = q.minBy { it.y - it.x }; val bl = q.maxBy { it.y - it.x }
        fun at(u: Float, v: Float): Offset {
            val top = tl + (tr - tl) * u
            val bottom = bl + (br - bl) * u
            return top + (bottom - top) * v
        }
        val (cols, rows) = DEST_ZONES
        val objs = objects().map { cubeBox(it) }
        val out = mutableListOf<Zone>()
        for (r in 0 until rows) for (c in 0 until cols) {
            val u0 = c.toFloat() / cols; val u1 = (c + 1f) / cols
            val v0 = r.toFloat() / rows; val v1 = (r + 1f) / rows
            val center = at((u0 + u1) / 2, (v0 + v1) / 2)
            val reachable = source.reachable(center)
            out += Zone(
                "zone ${'A' + r}${c + 1}", listOf(at(u0, v0), at(u1, v0), at(u1, v1), at(u0, v1)), center,
                occupied = objs.any { it.contains(center) }, reachable = reachable,
            )
        }
        return out
    }

    /** Operation buttons in a column on the right, plus Cancel (buttons() in the prototype), in dp. */
    fun buttons(): List<Pair<String, Rect>> {
        val (W, H) = screen.width to screen.height
        val bw = W * 0.24f; val bh = H * 0.13f; val gap = H * 0.035f
        val items = operations.map { it.key } + "cancel"
        val total = items.size * bh + (items.size - 1) * gap
        val y0 = (H - total) / 2; val x0 = W - bw - 40
        return items.mapIndexed { i, k -> k to Rect(x0, y0 + i * (bh + gap), x0 + bw, y0 + i * (bh + gap) + bh) }
    }

    fun label(key: String) = operations.firstOrNull { it.key == key }?.label ?: "Cancel"

    /** The Cancel box shown while choosing a destination (CANCEL_POS), in dp. */
    fun cancelBox(): Rect = Offset(screen.width - 140, screen.height - 150).let { Rect(it.x - 100, it.y - 40, it.x + 100, it.y + 40) }

    /** The STOP box shown while the arm is moving, in dp: big, and quick to trigger. */
    fun stopBox(): Rect = Rect(screen.width - 300, screen.height - 230, screen.width - 40, screen.height - 90)

    fun targets(v: View): List<Target> = when {
        source.busy -> listOf(Target("stop", stopBox().center, DWELL_STOP_S, box = stopBox()))   // only STOP while the arm moves
        source.notReady != null -> emptyList()
        else -> selectionTargets(v)
    }

    private fun selectionTargets(v: View): List<Target> = when (flow) {
        Flow.Object -> objects().map { o ->
            val b = cubeBox(o)
            Target(o.name, img2scr(o.img, v), DWELL_OBJECT_S, Rect(img2scr(b.topLeft, v), img2scr(b.bottomRight, v)))
        }
        Flow.Operation -> buttons().map { (k, r) -> Target(k, r.center, DWELL_BUTTON_S) }
        Flow.Destination -> zones().filter { !it.occupied && it.reachable }.map { z ->
            Target(z.name, img2scr(z.img, v), DWELL_SPOT_S, poly = z.poly.map { img2scr(it, v) })
        } + Target("cancel", cancelBox().center, DWELL_BUTTON_S)
        Flow.Done -> emptyList()
    }

    // ------------------------------------------------------------------ loop
    fun start() {
        scope.launch {
            var last = now()
            while (true) {
                val t = now()
                val dt = min(t - last, 0.2)
                last = t
                updateStage()
                if (stage == Stage.Running) step(t, dt)
                delay(33)
            }
        }
    }

    private fun updateStage() {
        val s = gaze.status
        stage = when {
            !gaze.serviceUp || s == null || !s.camera -> Stage.Waiting
            gaze.calibrating -> { if (s.phase == "correcting") corrected = true; Stage.Calibrating }
            !s.calibrated -> { worldIntroDone = false; corrected = false; if (s.savedCalibration || s.mock) Stage.SavedPrompt else Stage.CalibIntro }
            !worldIntroDone -> Stage.WorldIntro
            else -> Stage.Running
        }
    }

    /** Gaze in dp, or null. */
    fun gazeDp(): Offset? = gaze.point?.let { it / density }

    private fun distance(g: Offset, t: Target): Double {
        t.poly?.let { poly ->
            val inside = pointInPolygon(g, poly)
            val edge = if (inside) 0.0 else polyDistance(g, poly)
            return edge + 0.3 * dist(g, t.center) + ZONE_PENALTY_PX
        }
        val b = t.box ?: return dist(g, t.center)
        val dx = max(max(b.left - g.x, 0f), g.x - b.right).toDouble()
        val dy = max(max(b.top - g.y, 0f), g.y - b.bottom).toDouble()
        return hypot(dx, dy) + 0.3 * dist(g, t.center)
    }

    private fun step(now: Double, dt: Double) {
        val v = view() ?: return
        val busy = source.busy
        if (flow == Flow.Done && now >= stateUntil && !busy) reset()
        if (wasBusy && !busy) stateUntil = now + SHOW_DONE_S   // the arm just finished: show the result for a moment
        wasBusy = busy
        val targets = targets(v)
        val g = gazeDp()

        var on: Int? = null
        if (g != null && !gaze.moving && targets.isNotEmpty() && (flow != Flow.Done || busy)) {
            val dists = targets.map { distance(g, it) }
            var best = dists.indices.minBy { dists[it] }               // the closest target wins...
            val li = targets.indexOfFirst { it.id == lastOnId }
            if (li >= 0 && dists[li] - dists[best] < HYSTERESIS_PX) best = li   // ...unless the current one is nearly as close
            if (exp(-dists[best] * dists[best] / (2 * SIGMA * SIGMA)) >= ON_INPUT) on = best
        }
        lastOnId = on?.let { targets[it].id }

        val ids = targets.map { it.id }.toSet()
        dwell.keys.filter { it !in ids }.forEach { dwell.remove(it) }
        if (now < graceUntil) on = null
        targets.forEachIndexed { i, t ->
            dwell[t.id] = if (i == on) (dwell[t.id] ?: 0.0) + dt else max(0.0, (dwell[t.id] ?: 0.0) - DECAY * dt)
        }

        val i = on ?: return
        val t = targets[i]
        if ((dwell[t.id] ?: 0.0) < t.need) return
        dwell.clear()
        graceUntil = now + STEP_GRACE_S
        if (t.id == "stop") return stopArm()
        when (flow) {
            Flow.Object -> {
                chosen = objects().firstOrNull { it.name == t.id } ?: return
                flow = Flow.Operation
            }
            Flow.Operation -> {
                if (t.id == "cancel") return reset()
                val op = operations.first { it.key == t.id }
                operation = op
                if (op.needsDest) flow = Flow.Destination else finish(now)
            }
            Flow.Destination -> {
                if (t.id == "cancel") return reset()
                val z = zones().first { it.name == t.id }
                dest = z.name to z.img
                finish(now)
            }
            Flow.Done -> {}
        }
    }

    /** finish_command(): send it to the arm (the dwell that got us here is the user's confirmation). */
    private fun finish(now: Double) {
        val c = chosen ?: return
        val o = objects().firstOrNull { it.name == c.name } ?: c   // the object may have moved a little
        chosen = o
        operation?.let { source.execute(it, o, dest?.second) }
        flow = Flow.Done
        stateUntil = now + SHOW_DONE_S
    }

    /** Stop the arm where it is (the STOP dwell, Esc). */
    fun stopArm() {
        source.stop()
        reset()
    }

    fun reset() {
        flow = Flow.Object
        chosen = null
        operation = null
        dest = null
        dwell.clear()
    }

    /** Keys, only as the prototype prompts for them (L / K / SPACE) plus its C, D, G, K shortcuts. */
    fun onKey(key: Char) {
        when (stage) {
            Stage.SavedPrompt -> when (key) { 'L' -> gaze.loadSaved(); 'K' -> gaze.calibrate() }
            Stage.CalibIntro -> if (key == ' ') gaze.calibrate()
            Stage.WorldIntro -> if (key == ' ') {
                worldIntroDone = true
                if (!corrected) gaze.correct()   // drift_correct(); already done right after a fresh calibration
                corrected = false
            }
            Stage.Running -> when (key) {
                'C' -> reset()
                'G' -> showGaze = !showGaze
                'D' -> { gaze.correct(); dwell.clear() }
                'K' -> { gaze.calibrate(); reset() }
            }
            else -> {}
        }
    }

    // ------------------------------------------------------------------ text
    fun banner(): Pair<String, String> {
        if (source.busy) return (source.progress ?: "The arm is moving") + "   -   look at STOP to stop it" to "orange"
        source.notice?.let { return it to "red" }
        source.notReady?.let { return it to "fg" }
        if (!gaze.face) return "No face detected (eye camera)" to "red"
        val c = chosen
        return when (flow) {
            Flow.Object -> "1) Look at an object to choose it (${DWELL_OBJECT_S.toInt()} s)" to "fg"
            Flow.Operation -> "2) What should the arm do with the ${c?.name}? Look at a button" to "orange"
            Flow.Destination -> "3) Look at the destination: a zone (A1-C2)" to "orange"
            Flow.Done -> (source.progress ?: "Sent: ${operation?.label} - ${c?.name}${dest?.let { " -> ${it.first}" } ?: ""}") to "green"
        }
    }

    fun statusLine(): String = "${objects().size} objects (${source.label})   |   ${source.status}"
}

private fun dist(a: Offset, b: Offset) = hypot((a.x - b.x).toDouble(), (a.y - b.y).toDouble())

private fun pointInPolygon(p: Offset, poly: List<Offset>): Boolean {
    var inside = false
    var j = poly.size - 1
    for (i in poly.indices) {
        val a = poly[i]; val b = poly[j]
        if ((a.y > p.y) != (b.y > p.y) && p.x < (b.x - a.x) * (p.y - a.y) / (b.y - a.y) + a.x) inside = !inside
        j = i
    }
    return inside
}

private fun polyDistance(p: Offset, poly: List<Offset>): Double = poly.indices.minOf { i ->
    val a = poly[i]; val b = poly[(i + 1) % poly.size]
    val ab = b - a
    val len2 = ab.x * ab.x + ab.y * ab.y
    val t = if (len2 == 0f) 0f else (((p - a).x * ab.x + (p - a).y * ab.y) / len2).coerceIn(0f, 1f)
    val q = a + ab * t
    sqrt(((p.x - q.x) * (p.x - q.x) + (p.y - q.y) * (p.y - q.y)).toDouble())
}
