package ch.lauzhack.gaze.ui

import androidx.compose.foundation.Canvas
import androidx.compose.foundation.Image
import androidx.compose.foundation.background
import androidx.compose.foundation.gestures.detectTapGestures
import androidx.compose.foundation.layout.Box
import androidx.compose.foundation.layout.aspectRatio
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.material3.Text
import androidx.compose.runtime.Composable
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.draw.clip
import androidx.compose.ui.geometry.Offset
import androidx.compose.ui.geometry.Size
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.graphics.Path
import androidx.compose.ui.graphics.PathEffect
import androidx.compose.ui.graphics.drawscope.DrawScope
import androidx.compose.ui.graphics.drawscope.Stroke
import androidx.compose.ui.input.pointer.PointerEventType
import androidx.compose.ui.input.pointer.PointerIcon
import androidx.compose.ui.input.pointer.pointerHoverIcon
import androidx.compose.ui.input.pointer.pointerInput
import androidx.compose.ui.layout.ContentScale
import androidx.compose.ui.text.TextMeasurer
import androidx.compose.ui.text.TextStyle
import androidx.compose.ui.text.drawText
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.text.rememberTextMeasurer
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import ch.lauzhack.gaze.Mode
import ch.lauzhack.gaze.StationController
import ch.lauzhack.gaze.World

/**
 * The overhead camera with the overlay of app.js: the working block's dashed outline and corner labels,
 * unreachable spots, the chosen cube (green ring), the destination (orange ring) and an arrow between them.
 */
@Composable
fun CameraView(ctl: StationController, modifier: Modifier = Modifier) {
    val g = ctl.geometry
    val ratio = if (g != null) g.imageWidth.toFloat() / g.imageHeight else 4f / 3f
    val measurer = rememberTextMeasurer()

    Box(
        modifier
            .fillMaxWidth()
            .aspectRatio(ratio)
            .clip(Radii.card)
            .background(Color.Black),
    ) {
        ctl.frame?.let {
            Image(it, contentDescription = "Overhead camera of the table and the robot",
                contentScale = ContentScale.FillBounds, modifier = Modifier.fillMaxSize())
        } ?: Text(if (ctl.connected) "Loading camera…" else "Waiting for the simulator at ${ctl.baseUrl}",
            color = Color(0xFF98A2B3), modifier = Modifier.align(Alignment.Center))

        Canvas(
            Modifier
                .fillMaxSize()
                .pointerHoverIcon(PointerIcon.Crosshair)
                .pointerInput(g) {
                    if (g == null) return@pointerInput
                    detectTapGestures { p ->
                        ctl.onTap(p.x / size.width * g.imageWidth.toDouble(), p.y / size.height * g.imageHeight.toDouble())
                    }
                }
                .pointerInput(g) {
                    if (g == null) return@pointerInput
                    awaitPointerEventScope {
                        while (true) {
                            val e = awaitPointerEvent()
                            val p = e.changes.firstOrNull()?.position
                            if (e.type == PointerEventType.Exit || p == null) ctl.onHover(null, null)
                            else ctl.onHover(p.x / size.width * g.imageWidth.toDouble(), p.y / size.height * g.imageHeight.toDouble())
                        }
                    }
                },
        ) {
            if (g == null) return@Canvas
            val k = size.width / g.imageWidth // image pixels -> canvas pixels
            fun px(w: World): Offset = g.toPixel(w).let { (x, y) -> Offset((x * k).toFloat(), (y * k).toFloat()) }

            // working block: dashed outline, corner labels, unreachable spots
            for (name in g.workBlocks) {
                val side = g.sideCm
                val corners = listOf(0.0 to 0.0, side to 0.0, side to side, 0.0 to side)
                    .map { (cx, cy) -> px(g.blockToWorld(name, cx, cy)) }
                val path = Path().apply {
                    moveTo(corners[0].x, corners[0].y)
                    corners.drop(1).forEach { lineTo(it.x, it.y) }
                    close()
                }
                drawPath(path, Overlay.blockOutline,
                    style = Stroke(width = 2.dp.toPx(), pathEffect = PathEffect.dashPathEffect(floatArrayOf(8.dp.toPx(), 5.dp.toPx()))))
                label(measurer, "(0, 0)", corners[0] + Offset(2.dp.toPx(), 22.dp.toPx()), Overlay.cornerLabel)
                label(measurer, "(${side.toInt()}, ${side.toInt()})", corners[2] + Offset(-66.dp.toPx(), -2.dp.toPx()), Overlay.cornerLabel)
                for ((cx, cy) in g.unreachable(name)) {
                    drawCircle(Overlay.unreachable, radius = 2.6.dp.toPx(), center = px(g.blockToWorld(name, cx.toDouble(), cy.toDouble())))
                }
            }

            val src = ctl.src
            val dst = ctl.dst
            if (src != null && dst != null && ctl.mode == Mode.MoveBlock) {
                drawLine(Overlay.arrow, px(src.at), px(dst.at), strokeWidth = 2.dp.toPx(),
                    pathEffect = PathEffect.dashPathEffect(floatArrayOf(6.dp.toPx(), 6.dp.toPx())))
            }
            if (src != null && ctl.mode == Mode.MoveBlock) ring(measurer, px(src.at), Overlay.srcRing, g.describe(src.at))
            if (dst != null) ring(measurer, px(dst.at), Overlay.dstRing, g.describe(dst.at))
        }
    }
}

private fun DrawScope.ring(measurer: TextMeasurer, c: Offset, color: Color, text: String) {
    drawCircle(color, radius = 18.dp.toPx(), center = c, style = Stroke(width = 3.dp.toPx()))
    drawCircle(color, radius = 3.5.dp.toPx(), center = c)
    label(measurer, text, c + Offset(22.dp.toPx(), -8.dp.toPx()), Color.White)
}

/** A small dark label with its bottom-left corner at [at] (as in app.js). */
private fun DrawScope.label(measurer: TextMeasurer, text: String, at: Offset, color: Color) {
    val layout = measurer.measure(text, TextStyle(fontSize = 12.sp, fontWeight = FontWeight.SemiBold, color = color))
    val padX = 5.dp.toPx()
    val h = 18.dp.toPx()
    drawRect(Overlay.labelBg, topLeft = Offset(at.x, at.y - h), size = Size(layout.size.width + 2 * padX, h))
    drawText(layout, topLeft = Offset(at.x + padX, at.y - h + (h - layout.size.height) / 2))
}
