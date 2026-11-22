package ch.lauzhack.gaze.ui

import androidx.compose.foundation.Canvas
import androidx.compose.foundation.background
import androidx.compose.foundation.layout.BoxWithConstraints
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.runtime.Composable
import androidx.compose.runtime.SideEffect
import androidx.compose.ui.Modifier
import androidx.compose.ui.geometry.CornerRadius
import androidx.compose.ui.geometry.Offset
import androidx.compose.ui.geometry.Rect
import androidx.compose.ui.geometry.Size
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.graphics.Path
import androidx.compose.ui.graphics.PathFillType
import androidx.compose.ui.graphics.drawscope.DrawScope
import androidx.compose.ui.graphics.drawscope.Stroke
import androidx.compose.ui.platform.LocalDensity
import androidx.compose.ui.text.TextMeasurer
import androidx.compose.ui.text.TextStyle
import androidx.compose.ui.text.drawText
import androidx.compose.ui.text.rememberTextMeasurer
import androidx.compose.ui.unit.IntOffset
import androidx.compose.ui.unit.IntSize
import androidx.compose.ui.unit.sp
import ch.lauzhack.gaze.Flow
import ch.lauzhack.gaze.SceneController
import ch.lauzhack.gaze.Stage
import kotlin.math.max
import kotlin.math.min

// Colours of gaze_scene_v2.py
private val BG = Color(30, 30, 30)
private val FG = Color(235, 235, 235)
private val DIM = Color(110, 110, 110)
private val BLUE = Color(80, 140, 255)
private val ORANGE = Color(255, 170, 60)
private val RED = Color(255, 90, 90)
private val GREEN = Color(90, 220, 120)
private val RING = Color(60, 60, 60)
private val BUTTON = Color(50, 50, 50)
private val BUTTON_FILL = Color(70, 110, 160)


private fun named(c: String) = when (c) { "orange" -> ORANGE; "green" -> GREEN; "red" -> RED; else -> FG }

/** The eye-tracking window: the gaze scene of the dev branch prototype (gaze_scene_v2.py). No mouse. */
@Composable
fun SceneScreen(scene: SceneController) {
    val density = LocalDensity.current.density
    val measurer = rememberTextMeasurer()
    BoxWithConstraints(Modifier.fillMaxSize().background(BG)) {
        val w = maxWidth.value
        val h = maxHeight.value
        SideEffect {
            scene.screen = Size(w, h)
            scene.density = density
        }
        // read the state the canvas depends on, so it redraws
        val stage = scene.stage
        val flow = scene.flow
        val frame = scene.source.frame
        @Suppress("UNUSED_VARIABLE") val busyNow = scene.source.busy
        val gazePt = scene.gaze.point
        val status = scene.gaze.status
        @Suppress("UNUSED_VARIABLE") val dw = scene.dwell.toMap()
        Canvas(Modifier.fillMaxSize()) {
            val d = density
            val text = Text(measurer, this, d)
            when (stage) {
                Stage.Waiting -> text.message(
                    if (!scene.gaze.serviceUp) listOf(
                        "Waiting for the eye tracker",
                        scene.gaze.launchNote ?: "Start it with:  .venv/bin/python -m gaze_service.server",
                        "(${scene.gaze.baseUrl})",
                    ) else if (status?.error?.isNotBlank() == true) listOf("Eye camera not available") + status.error.chunked(90)
                    else listOf("Waiting for the eye camera", "Opening the camera…"),
                )
                Stage.SavedPrompt -> text.message(listOf(
                    "Calibration found",
                    "L = load the saved calibration (same seat, same camera position)",
                    "K = calibrate again",
                ))
                Stage.CalibIntro -> text.message(listOf(
                    "Calibration",
                    "25 dots across the whole screen, about 2 seconds each.",
                    "Keep your head still. Look at the center of each dot until it moves on.",
                    "Press SPACE to start",
                ))
                Stage.WorldIntro -> text.message(listOf(
                    "Look at the world",
                    "The table camera is shown full screen. Look at an object for 2 s to choose it,",
                    "then look at an operation button, then (if asked) at a destination.",
                    "C = cancel   D = correction   ESC = stop the arm",
                    "Press SPACE to start",
                ))
                Stage.Calibrating -> drawCalibration(scene, text)
                Stage.Running -> drawScene(scene, text, flow, frame, gazePt)
            }
        }
    }
}

/** Text helpers in screen units (dp), like text_center() in the prototype. */
private class Text(val m: TextMeasurer, val ds: DrawScope, val d: Float) {
    fun layout(s: String, size: Int, color: Color) = m.measure(s, TextStyle(fontSize = size.sp, color = color))
    fun center(s: String, x: Float, y: Float, size: Int = 24, color: Color = FG) {
        val l = layout(s, size, color)
        val at = Offset(x * d - l.size.width / 2f, y * d - l.size.height / 2f)
        ds.drawText(layout(s, size, Color.Black.copy(alpha = 0.75f)), topLeft = at + Offset(1.5f * d, 1.5f * d)) // readable on the light table
        ds.drawText(l, topLeft = at)
    }
    fun at(s: String, x: Float, y: Float, size: Int = 24, color: Color = FG) = ds.drawText(layout(s, size, color), topLeft = Offset(x * d, y * d))
    fun message(lines: List<String>) {
        val w = ds.size.width / d
        val h = ds.size.height / d
        lines.forEachIndexed { i, line ->
            center(line, w / 2, h * 0.28f + i * 50, if (i == 0) 44 else 24, if (i == 0) BLUE else FG)
        }
    }
}

private fun DrawScope.drawCalibration(scene: SceneController, text: Text) {
    val s = scene.gaze.status ?: return
    val t = s.target ?: return
    val d = text.d
    val w = size.width / d
    // where that screen point is in this window (the window goes full screen while calibrating)
    val c = scene.gaze.mapper?.invoke(t.x, t.y) ?: Offset((t.x * size.width).toFloat(), (t.y * size.height).toFloat())
    if (t.kind == "cross") {
        drawLine(FG, c - Offset(25 * d, 0f), c + Offset(25 * d, 0f), 3 * d)
        drawLine(FG, c - Offset(0f, 25 * d), c + Offset(0f, 25 * d), 3 * d)
        text.center("Gaze correction: look at the cross", w / 2, 30f, 24, DIM)
    } else {
        drawCircle(BLUE, ((40 * (1 - t.progress) + 8) * d).toFloat(), c, style = Stroke(4 * d))
        drawCircle(if (t.recording) GREEN else FG, 6 * d, c)
        text.center("Calibration ${t.index + 1}/${t.total}", w / 2, 30f, 24, DIM)
    }
    if (!s.face) text.center("No face detected - check camera, lighting and position", w / 2, size.height / d / 2, 24, RED)
}

/** draw_ring(): grey track plus a blue (or given colour) arc that fills clockwise from the top. */
private fun DrawScope.ring(center: Offset, frac: Double, radius: Float, d: Float, color: Color = BLUE) {
    drawCircle(RING, radius * d, center * d, style = Stroke(5 * d))
    if (frac > 0.01) {
        val r = radius * d
        drawArc(color, -90f, (360 * min(frac, 1.0)).toFloat(), false, center * d - Offset(r, r), Size(2 * r, 2 * r), style = Stroke(8 * d))
    }
}

private fun DrawScope.drawScene(
    scene: SceneController, text: Text, flow: Flow,
    frame: androidx.compose.ui.graphics.ImageBitmap?, gazePt: Offset?,
) {
    val d = text.d
    val W = size.width / d
    val H = size.height / d
    val v = scene.view()
    if (frame != null && v != null) {
        // camera full screen, outside the work area darkened (frame_surface)
        drawImage(frame, dstOffset = IntOffset((v.ox * d).toInt(), (v.oy * d).toInt()),
            dstSize = IntSize((v.fw * v.s * d).toInt(), (v.fh * v.s * d).toInt()))
        val work = scene.workArea().map { scene.img2scr(it, v) * d }
        if (work.size == 4) {
            val p = Path().apply {
                fillType = PathFillType.EvenOdd
                addRect(Rect(v.ox * d, v.oy * d, (v.ox + v.fw * v.s) * d, (v.oy + v.fh * v.s) * d))
                moveTo(work[0].x, work[0].y); work.drop(1).forEach { lineTo(it.x, it.y) }; close()
            }
            drawPath(p, Color.Black.copy(alpha = 0.7f))
        }
        if (flow == Flow.Operation) {
            drawRect(Color.Black.copy(alpha = 0.55f), Offset(v.ox * d, v.oy * d), Size(v.fw * v.s * d, v.fh * v.s * d))
        }
    } else {
        text.center(scene.source.notReady ?: "No camera picture yet (${scene.source.baseUrl})", W / 2, H / 2, 24, RED)
    }

    if (v != null) {
        val targets = scene.targets(v)
        // objects: dwell ring and name (no coloured outline: the camera shows the block itself)
        if (flow == Flow.Object || flow == Flow.Destination) {
            for (o in scene.objects()) {
                if (flow == Flow.Destination) continue
                val b = scene.cubeBox(o)
                val r = Rect(scene.img2scr(b.topLeft, v), scene.img2scr(b.bottomRight, v))
                val t = targets.firstOrNull { it.id == o.name } ?: continue
                val radius = min(max(40f, min(r.width, r.height) * 0.45f), 80f)
                ring(t.center, (scene.dwell[t.id] ?: 0.0) / t.need, radius, d)
                text.center(o.name, t.center.x, r.bottom + 22 + radius - min(r.width, r.height) / 2, 24, FG)
            }
        }
        val chosen = scene.chosen
        if (chosen != null && flow != Flow.Object) {
            val p = scene.img2scr(chosen.img, v)
            drawCircle(ORANGE, 60 * d, p * d, style = Stroke(5 * d))
            text.center("1  ${chosen.name}", p.x, p.y - 85, 24, ORANGE)
        }
        if (flow == Flow.Operation) {
            for ((k, r) in scene.buttons()) {
                val frac = (scene.dwell[k] ?: 0.0) / SceneController.DWELL_BUTTON_S
                drawRoundRect(BUTTON, r.topLeft * d, r.size * d, CornerRadius(14 * d))
                if (frac > 0) drawRoundRect(BUTTON_FILL, r.topLeft * d, Size(r.width * min(frac, 1.0).toFloat(), r.height) * d, CornerRadius(14 * d))
                drawRoundRect(if (k == "cancel") RED else FG, r.topLeft * d, r.size * d, CornerRadius(14 * d), style = Stroke(3 * d))
                text.center(scene.label(k), r.center.x, r.center.y, 40, FG)
            }
        }
        if (flow == Flow.Destination) {
            for (z in scene.zones()) {
                val poly = z.poly.map { scene.img2scr(it, v) }
                val t = targets.firstOrNull { it.id == z.name }
                val frac = t?.let { (scene.dwell[it.id] ?: 0.0) / it.need } ?: 0.0
                val color = when { t == null -> Color(90, 90, 90); frac > 0.02 -> GREEN; else -> Color(200, 200, 200) }
                val path = Path().apply { moveTo(poly[0].x * d, poly[0].y * d); poly.drop(1).forEach { lineTo(it.x * d, it.y * d) }; close() }
                drawPath(path, color, style = Stroke((if (t == null) 1 else 2) * d))
                if (t != null && frac > 0.02) ring(t.center, frac, 35f, d, GREEN)
                text.center(z.name.removePrefix("zone "), poly[0].x + 22, poly[0].y + 18, 24, Color(220, 220, 220))
            }
            val cb = scene.cancelBox()
            val cf = (scene.dwell["cancel"] ?: 0.0) / SceneController.DWELL_BUTTON_S
            if (cf > 0) drawRoundRect(RED.copy(alpha = 0.35f), cb.topLeft * d, Size(cb.width * min(cf, 1.0).toFloat(), cb.height) * d, CornerRadius(12 * d))
            drawRoundRect(RED, cb.topLeft * d, cb.size * d, CornerRadius(12 * d), style = Stroke(3 * d))
            text.center("Cancel", cb.center.x, cb.center.y, 24, RED)
        }
        if (flow == Flow.Done && chosen != null) {
            val a = scene.img2scr(chosen.img, v)
            scene.dest?.let { (_, img) ->
                val b = scene.img2scr(img, v)
                drawLine(GREEN, a * d, b * d, 6 * d)
                drawCircle(GREEN, 40 * d, b * d, style = Stroke(5 * d))
            }
            drawCircle(GREEN, 60 * d, a * d, style = Stroke(6 * d))
        }
    }

    // STOP while the arm is moving: big, red, triggered by a short dwell (or Esc)
    if (scene.source.busy) {
        val sb = scene.stopBox()
        val sf = (scene.dwell["stop"] ?: 0.0) / SceneController.DWELL_STOP_S
        drawRoundRect(RED.copy(alpha = 0.25f), sb.topLeft * d, sb.size * d, CornerRadius(16 * d))
        if (sf > 0) drawRoundRect(RED.copy(alpha = 0.6f), sb.topLeft * d, Size(sb.width * min(sf, 1.0).toFloat(), sb.height) * d, CornerRadius(16 * d))
        drawRoundRect(RED, sb.topLeft * d, sb.size * d, CornerRadius(16 * d), style = Stroke(5 * d))
        text.center("STOP", sb.center.x, sb.center.y, 56, FG)
    }

    // the gaze circle: orange, grey while the eyes are moving
    if (scene.showGaze && gazePt != null) {
        drawCircle(if (scene.gaze.moving) DIM else ORANGE, 12 * d, gazePt, style = Stroke(3 * d))
    }

    val (banner, col) = scene.banner()
    text.center(banner, W / 2, H - 40, if (flow == Flow.Done && !scene.source.busy) 44 else 24, named(col))
    text.at(scene.statusLine(), 20f, 15f, 24, GREEN)
}
