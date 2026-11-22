package ch.lauzhack.gaze

import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.setValue
import androidx.compose.ui.geometry.Offset
import androidx.compose.ui.graphics.ImageBitmap
import io.ktor.client.HttpClient
import io.ktor.client.call.body
import io.ktor.client.plugins.HttpTimeout
import io.ktor.client.request.get
import io.ktor.client.request.header
import io.ktor.client.request.post
import io.ktor.client.request.setBody
import io.ktor.client.statement.bodyAsText
import io.ktor.http.ContentType
import io.ktor.http.contentType
import io.ktor.http.isSuccess
import kotlinx.coroutines.CancellationException
import kotlinx.coroutines.CoroutineScope
import kotlinx.coroutines.delay
import kotlinx.coroutines.launch
import kotlinx.serialization.SerialName
import kotlinx.serialization.Serializable
import kotlinx.serialization.json.Json
import kotlinx.serialization.json.JsonObject
import kotlinx.serialization.json.add
import kotlinx.serialization.json.buildJsonObject
import kotlinx.serialization.json.put
import kotlinx.serialization.json.putJsonArray
import kotlinx.serialization.json.putJsonObject
import kotlin.math.hypot
import kotlin.time.TimeSource
import kotlin.uuid.ExperimentalUuidApi
import kotlin.uuid.Uuid

// ---- what lauzkach_eye_robot_control.server answers (only the fields used here)

@Serializable
data class DetectSource(val status: String = "", val error: String? = null)

@Serializable
data class DetectCalibration(val state: String = "", val remaining: Double? = null, val reason: String? = null)

@Serializable
data class DetectedBlock(
    val colour: String = "",
    val px: List<Double> = emptyList(),
    @SerialName("box_px") val boxPx: List<List<Double>> = emptyList(),
)

@Serializable
data class DetectSnapshot(
    val source: DetectSource = DetectSource(),
    val calibration: DetectCalibration = DetectCalibration(),
    val seq: Long? = null,
    val locked: Boolean = false,
    @SerialName("polygon_shown") val polygon: List<List<Double>>? = null,
    val blocks: List<DetectedBlock> = emptyList(),
)

@Serializable
data class PickPlaceStatus(
    val state: String = "idle",
    val step: Int = 0,
    val total: Int = 0,
    val name: String = "",
    val message: String = "",
)

@Serializable
data class RealOverview(
    val available: Boolean = false,
    val connected: Boolean? = null,
    val error: String? = null,
    val sequence: PickPlaceStatus? = null,
)

@Serializable
data class DriveOverview(
    val enabled: Boolean = false,
    @SerialName("server_ms") val serverMs: Double = 0.0,   // the server sends milliseconds with a fraction
    val generation: Long = 0,
    val real: RealOverview = RealOverview(),
)

@Serializable
private data class ApiErrorBody(val error: String = "", val code: String? = null)

/**
 * The real SO-101 in Poland, through the camera page server (`lauzkach_eye_robot_control.server --real-agent ...`)
 * running on this laptop: it reads the overhead camera over Tailscale, finds the table rectangle and the blocks, and
 * runs a whole pick and place through the arm agent next to the arm.
 *
 * Commands carry the same envelope as the team's web page (a command id so a retry never moves the arm twice,
 * the server's time so a lagging command is refused, and the stop generation). Polling /api/drive also keeps the
 * server's dead-man alive while the arm moves: if this app goes away, the arm holds.
 */
class RealArmSource(override val baseUrl: String, private val scope: CoroutineScope) : SceneSource {
    override val label = "real arm"

    var snapshot by mutableStateOf<DetectSnapshot?>(null); private set
    var drive by mutableStateOf<DriveOverview?>(null); private set
    override var frame by mutableStateOf<ImageBitmap?>(null); private set
    override var imageSize by mutableStateOf<Pair<Float, Float>?>(null); private set
    private var lastErrorText by mutableStateOf<String?>(null)
    private var lastErrorAt = TimeSource.Monotonic.markNow()
    /** A command's problem, shown for 8 s (the server's own error stays visible through the sequence status). */
    private var lastError: String?
        get() = lastErrorText?.takeIf { lastErrorAt.elapsedNow().inWholeSeconds < 8 }
        set(v) { lastErrorText = v; lastErrorAt = TimeSource.Monotonic.markNow() }
    private var serverUp by mutableStateOf(false)
    private var loadedSeq = -1L
    private var serverMsAt = 0.0
    private var serverMark = TimeSource.Monotonic.markNow()

    private val json = Json { ignoreUnknownKeys = true }
    private val http = HttpClient {
        install(HttpTimeout) { requestTimeoutMillis = 6_000; connectTimeoutMillis = 2_000 }
        expectSuccess = false
    }

    fun start() {
        scope.launch { while (true) { pollDetect(); delay(200) } }
        scope.launch { while (true) { pollDrive(); delay(400) } }
    }

    private suspend fun pollDetect() {
        try {
            val s = json.decodeFromString<DetectSnapshot>(http.get("$baseUrl/api/detect").bodyAsText())
            snapshot = s
            serverUp = true
            val seq = s.seq
            if (seq != null && seq != loadedSeq) {
                val r = http.get("$baseUrl/frame/$seq.jpg") { header("Cache-Control", "no-store") }
                if (r.status.isSuccess()) {
                    val img = decodeImage(r.body<ByteArray>())
                    frame = img
                    imageSize = img.width.toFloat() to img.height.toFloat()
                    loadedSeq = seq
                }
            }
        } catch (e: CancellationException) {
            throw e
        } catch (e: Exception) {
            serverUp = false
        }
    }

    private suspend fun pollDrive() {
        try {
            val d = json.decodeFromString<DriveOverview>(http.get("$baseUrl/api/drive").bodyAsText())
            drive = d
            serverMsAt = d.serverMs
            serverMark = TimeSource.Monotonic.markNow()
        } catch (e: CancellationException) {
            throw e
        } catch (e: Exception) {
            drive = null
            println("arm server /api/drive: ${e.message}")
        }
    }

    /** The server's clock now, from its last answer (same machine, so no skew beyond the poll interval). */
    private fun serverNow(): Long = (serverMsAt + serverMark.elapsedNow().inWholeMilliseconds).toLong()

    @OptIn(ExperimentalUuidApi::class)
    private fun send(path: String, body: JsonObject.() -> JsonObject = { this }, what: String) {
        if (drive == null && path != "/api/drive/stop") {
            lastError = "$what: no answer from the arm server yet, nothing was sent."
            return
        }
        val generation = drive?.generation ?: 0
        val cid = Uuid.random().toString()
        scope.launch {
            val payload = buildJsonObject {
                put("cid", cid)
                put("sent_ms", serverNow())
                put("gen", generation)
            }.let { base -> JsonObject(base + body(JsonObject(emptyMap()))) }
            repeat(2) { attempt ->        // a lost reply is retried with the SAME id: the server never moves the arm twice
                try {
                    val r = http.post("$baseUrl$path") {
                        contentType(ContentType.Application.Json)
                        setBody(payload.toString())
                    }
                    if (r.status.isSuccess()) {
                        lastError = null
                    } else {
                        val err = runCatching { json.decodeFromString<ApiErrorBody>(r.bodyAsText()) }.getOrNull()
                        val hint = when (err?.code) {
                            "stale" -> " The link is lagging: nothing was moved. Try again."
                            "old_generation" -> " (Stop was pressed after it was sent.)"
                            else -> ""
                        }
                        lastError = "$what: ${err?.error?.ifBlank { null } ?: "refused (${r.status.value})"}$hint"
                    }
                    return@launch
                } catch (e: CancellationException) {
                    throw e
                } catch (e: Exception) {
                    if (attempt == 1) lastError = "$what: no answer from $baseUrl (${e::class.simpleName})"
                    else delay(300)
                }
            }
        }
    }

    // ------------------------------------------------------------------ SceneSource

    override fun objects(): List<Cube> {
        val s = snapshot ?: return emptyList()
        if (!s.locked) return emptyList()
        val seen = mutableMapOf<String, Int>()
        return s.blocks.filter { it.px.size == 2 }.map { b ->
            val n = (seen[b.colour] ?: 0) + 1
            seen[b.colour] = n
            val name = if (n == 1) "${b.colour} block" else "${b.colour} block $n"
            Cube(name, b.colour, Offset(b.px[0].toFloat(), b.px[1].toFloat()),
                b.boxPx.filter { it.size == 2 }.map { Offset(it[0].toFloat(), it[1].toFloat()) })
        }
    }

    override fun workArea(): List<Offset> {
        val s = snapshot ?: return emptyList()
        if (!s.locked) return emptyList()
        return s.polygon.orEmpty().filter { it.size == 2 }.map { Offset(it[0].toFloat(), it[1].toFloat()) }
    }

    override val operations = listOf(Operation("pick_place", "Pick & place", true))

    /** The arm server was started without the real arm: choosing works, but nothing is sent. */
    val viewOnly: Boolean get() = drive?.real?.available == false

    override fun execute(op: Operation, cube: Cube, dest: Offset?) {
        if (op.key != "pick_place" || dest == null) return
        if (viewOnly) {
            lastError = "View-only: the real arm is not connected, nothing was moved (${cube.name} -> chosen spot)."
            return
        }
        val others = objects().filter { hypot((it.img.x - cube.img.x).toDouble(), (it.img.y - cube.img.y).toDouble()) > 40 }
        send("/api/drive/pickplace", {
            buildJsonObject {
                putJsonObject("pick") {
                    put("px", cube.img.x.toDouble()); put("py", cube.img.y.toDouble())
                    putJsonArray("box") { cube.box.forEach { p -> add(kotlinx.serialization.json.buildJsonArray { add(p.x.toDouble()); add(p.y.toDouble()) }) } }
                    put("colour", cube.color)
                }
                putJsonObject("dest") { put("px", dest.x.toDouble()); put("py", dest.y.toDouble()) }
                putJsonArray("others") { others.forEach { o -> add(kotlinx.serialization.json.buildJsonArray { add(o.img.x.toDouble()); add(o.img.y.toDouble()) }) } }
                put("park_after", true)
                put("confirm_real", true)   // the gaze dwell on the destination is the user's confirmation
            }
        }, "Move the ${cube.name}")
    }

    /** Soft stop: always accepted by the server, whatever the lag or generation. */
    override fun stop() = send("/api/drive/stop", what = "Stop")

    private val sequence: PickPlaceStatus? get() = drive?.real?.sequence

    override val busy: Boolean get() = sequence?.state == "running"

    override val progress: String?
        get() {
            val q = sequence ?: return null
            return when (q.state) {
                "running" -> "Step ${q.step} of ${q.total}: ${q.name.ifBlank { "working" }}${if (q.message.isNotBlank()) " - ${q.message}" else ""}"
                "done" -> q.message.ifBlank { "Done." }
                else -> null
            }
        }

    override val notice: String?
        get() {
            lastError?.let { return it }
            val q = sequence
            if (q != null && q.state in setOf("error", "stopped")) return q.message.ifBlank { "Stopped." }
            val r = drive?.real
            if (r != null && r.available && r.connected == false) return "Arm agent not reachable: ${r.error ?: "no answer"}"
            return null
        }

    override val notReady: String?
        get() {
            if (!serverUp) return "Waiting for the arm server at $baseUrl (run_real.command starts it)"
            val s = snapshot ?: return "Connecting to the arm server…"
            if (s.source.status != "live" && frame == null) return "Waiting for the camera in Poland (${s.source.error ?: s.source.status})"
            return when (s.calibration.state) {
                "locked" -> null
                "countdown" -> "Measuring the table rectangle… ${s.calibration.remaining?.toInt() ?: ""} s"
                "failed" -> "Could not find the table rectangle: ${s.calibration.reason ?: "check the camera"}"
                else -> "Looking for the table rectangle…"
            }
        }

    override val status: String
        get() {
            val r = drive?.real ?: return "arm server: not connected"
            return when {
                !r.available -> "arm: view-only (not connected)"
                r.connected == false -> "arm: agent not reachable"
                else -> "arm: " + (sequence?.let { if (it.state == "running") "moving" else it.state } ?: "ready")
            }
        }
}
