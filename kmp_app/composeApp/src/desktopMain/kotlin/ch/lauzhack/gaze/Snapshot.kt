package ch.lauzhack.gaze

import androidx.compose.ui.ImageComposeScene
import androidx.compose.ui.unit.Density
import ch.lauzhack.gaze.api.StationApi
import ch.lauzhack.gaze.ui.App
import kotlinx.coroutines.CoroutineScope
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.SupervisorJob
import kotlinx.coroutines.cancel
import kotlinx.coroutines.delay
import kotlinx.coroutines.runBlocking
import org.jetbrains.skia.EncodedImageFormat
import java.io.File

/**
 * Dev tool: renders the real UI off-screen (no window) against a running station server and writes PNGs.
 * With `--select` it taps the first cube and a destination with "Ask before moving" on, so nothing moves.
 *
 *     ./gradlew :composeApp:snapshot --args="out-dir [server-url] [--select]"
 */
fun main(args: Array<String>) = runBlocking {
    val out = File(args.getOrElse(0) { "build/snapshots" }).apply { mkdirs() }
    val url = args.getOrNull(1)?.takeIf { it.startsWith("http") } ?: "http://127.0.0.1:8810"
    val select = "--select" in args

    val scope = CoroutineScope(SupervisorJob() + Dispatchers.Default)
    val ctl = StationController(StationApi(url), scope).also { it.start() }
    repeat(150) { if (ctl.geometry == null || ctl.frame == null || ctl.cubes.isEmpty()) delay(100) }
    check(ctl.geometry != null && ctl.frame != null) { "No data from $url" }

    if (select) {
        val g = ctl.geometry!!
        ctl.askBeforeMoving = true
        val cube = ctl.cubes.first { !it.held }
        val (cx, cy) = g.toPixel(World(cube.x, cube.y))
        ctl.onTap(cx, cy)
        val (dx, dy) = g.toPixel(g.blockToWorld("left", 10.0, 4.0))
        ctl.onTap(dx, dy)
        ctl.onHover(dx, dy)
    }

    for ((name, dark) in listOf("light" to false, "dark" to true)) {
        val scene = ImageComposeScene(1280, 860, Density(1f)) { App(ctl, dark) }
        var t = 0L
        repeat(5) { scene.render(t); t += 16_000_000; delay(50) }
        val png = scene.render(t).encodeToData(EncodedImageFormat.PNG)!!.bytes
        File(out, "ui-$name${if (select) "-selected" else ""}.png").writeBytes(png)
        scene.close()
    }
    println("Wrote snapshots to ${out.absolutePath}")
    scope.cancel()
}
