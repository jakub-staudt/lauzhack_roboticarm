package ch.lauzhack.gaze

import androidx.compose.ui.ImageComposeScene
import androidx.compose.ui.geometry.Offset
import androidx.compose.ui.unit.Density
import ch.lauzhack.gaze.api.GazeApi
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
 * Dev tool: drives the real UI off-screen against the simulator and a mock gaze service, writing a PNG per step:
 * prompts, calibration, object dwell, operation buttons, destination zones and "Sent". Moves the simulated arm.
 *
 *     .venv/bin/python -m gaze_service.server --mock --port 8821
 *     ./gradlew :composeApp:snapshot --args="out-dir http://127.0.0.1:8810 http://127.0.0.1:8821"
 */
fun main(args: Array<String>): Unit = runBlocking<Unit> {
    val out = File(args.getOrElse(0) { "build/snapshots" }).apply { mkdirs() }
    val url = args.getOrElse(1) { "http://127.0.0.1:8810" }
    val gazeUrl = args.getOrElse(2) { "http://127.0.0.1:8821" }
    val (W, H) = 1440 to 900

    val scope = CoroutineScope(SupervisorJob() + Dispatchers.Default)
    // GAZE_ARM=real: drive the real-arm path (the camera page server at GAZE_ARM_URL) instead of the simulator
    val realUrl = if (System.getenv("GAZE_ARM") == "real") System.getenv("GAZE_ARM_URL") ?: "http://127.0.0.1:8820" else null
    val station = StationController(StationApi(url), scope).also { if (realUrl == null) it.start() }
    val source: SceneSource = if (realUrl != null) RealArmSource(realUrl, scope).also { it.start() } else SimSource(station)
    val gaze = GazeController(GazeApi(gazeUrl), scope)
    gaze.mapper = { nx, ny -> Offset((nx * W).toFloat(), (ny * H).toFloat()) }
    gaze.screenSize = { W to H }
    gaze.start()
    val scene = SceneController(source, gaze, scope).also { it.start() }
    repeat(150) { if (source.notReady != null || source.frame == null || !gaze.serviceUp) delay(100) }
    check(source.notReady == null && source.frame != null && gaze.serviceUp) { "Not ready: ${source.notReady} / gaze ${gaze.serviceUp}" }

    val http = java.net.http.HttpClient.newHttpClient()
    fun look(p: Offset) {
        val req = java.net.http.HttpRequest.newBuilder(java.net.URI("$gazeUrl/api/mock"))
            .POST(java.net.http.HttpRequest.BodyPublishers.ofString("""{"x": ${p.x / W}, "y": ${p.y / H}}""")).build()
        http.send(req, java.net.http.HttpResponse.BodyHandlers.discarding())
    }
    val ui = ImageComposeScene(W, H, Density(1f)) { ch.lauzhack.gaze.ui.SceneScreen(scene) }
    var t = 0L
    suspend fun run(ms: Long) { repeat((ms / 40).toInt()) { ui.render(t); t += 40_000_000; delay(40) } }
    fun shot(name: String) = File(out, "scene-$name.png").writeBytes(ui.render(t).encodeToData(EncodedImageFormat.PNG)!!.bytes)
    suspend fun waitFor(what: String, ms: Long = 8000, cond: () -> Boolean) {
        var left = ms
        while (!cond() && left > 0) { run(80); left -= 80 }
        check(cond()) { "timed out waiting for $what (stage=${scene.stage}, flow=${scene.flow})" }
    }

    gaze.cancelCalibration(); run(300)
    waitFor("prompt") { scene.stage == Stage.SavedPrompt || scene.stage == Stage.CalibIntro }
    shot("1-prompt")
    scene.onKey('K')
    waitFor("calibration") { scene.stage == Stage.Calibrating }
    run(2600); shot("2-calibration")
    gaze.cancelCalibration(); run(300)
    waitFor("prompt again") { scene.stage == Stage.SavedPrompt }
    scene.onKey('L')
    waitFor("world intro") { scene.stage == Stage.WorldIntro }
    shot("3-world-intro")
    scene.onKey(' ')
    waitFor("running", 12000) { scene.stage == Stage.Running && gaze.tracking }

    val v = scene.view()!!
    val cube = scene.objects().first()
    look(scene.img2scr(cube.img, v)); run(1200); shot("4-object-dwell")
    waitFor("operation") { scene.flow == Flow.Operation }
    run(200)
    val pick = scene.buttons().first { it.first == "pick_place" }.second.center
    look(pick); run(1000); shot("5-operation")
    waitFor("destination") { scene.flow == Flow.Destination }
    val zone = scene.zones().first { !it.occupied && it.reachable }
    look(scene.img2scr(zone.img, v)); run(1300); shot("6-destination")
    waitFor("done") { scene.flow == Flow.Done }
    run(300); shot("7-sent")
    if (realUrl != null) {
        waitFor("arm busy", 5000) { source.busy }
        run(1500); shot("8-arm-moving")
        println("while moving: ${source.progress}")
        waitFor("arm finished", 30000) { !source.busy }
        run(500); shot("9-arm-done")
        println("after: progress=${source.progress} notice=${source.notice}")
    }
    println("Scene run OK: ${scene.chosen?.name} -> ${zone.name}, arm: ${station.task.state}. PNGs in ${out.absolutePath}")
    ui.close()
    // the other window: simulation + controls
    val control = ImageComposeScene(1280, 860, Density(1f)) { App(station) }
    var tc = 0L
    repeat(5) { control.render(tc); tc += 16_000_000; delay(50) }
    File(out, "control-window.png").writeBytes(control.render(tc).encodeToData(EncodedImageFormat.PNG)!!.bytes)
    control.close()
    scope.cancel()
    kotlin.system.exitProcess(0)
}
