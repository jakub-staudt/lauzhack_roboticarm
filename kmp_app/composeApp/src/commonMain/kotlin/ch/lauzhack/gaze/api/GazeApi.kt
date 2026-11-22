package ch.lauzhack.gaze.api

import io.ktor.client.HttpClient
import io.ktor.client.call.body
import io.ktor.client.plugins.HttpTimeout
import io.ktor.client.plugins.contentnegotiation.ContentNegotiation
import io.ktor.client.request.get
import io.ktor.client.request.post
import io.ktor.client.request.setBody
import io.ktor.client.statement.HttpResponse
import io.ktor.client.statement.bodyAsText
import io.ktor.http.ContentType
import io.ktor.http.contentType
import io.ktor.http.isSuccess
import io.ktor.serialization.kotlinx.json.json
import kotlinx.serialization.Serializable
import kotlinx.serialization.json.Json

/** A calibration dot or correction cross, in screen fractions. */
@Serializable
data class GazeTargetPoint(
    val kind: String = "dot",
    val x: Double = 0.5,
    val y: Double = 0.5,
    val index: Int = 0,
    val total: Int = 1,
    val progress: Double = 0.0,
    val recording: Boolean = false,
)

/** `GET /api/gaze` of the gaze service (`python -m gaze_service.server`). x / y are screen fractions. */
@Serializable
data class GazeStatus(
    val phase: String = "idle",          // idle | calibrating | correcting | tracking
    val camera: Boolean = false,
    val error: String = "",
    val mock: Boolean = false,
    val calibrated: Boolean = false,
    val face: Boolean = false,
    val x: Double? = null,
    val y: Double? = null,
    val moving: Boolean = false,
    val target: GazeTargetPoint? = null,
    val savedCalibration: Boolean = false,
)

@Serializable
private data class ScreenBody(val screen: List<Int>)

/** Client for the gaze service (default http://127.0.0.1:8820), the eye tracker from the dev branch prototype. */
class GazeApi(val baseUrl: String) {
    private val json = Json { ignoreUnknownKeys = true }

    private val http = HttpClient {
        install(ContentNegotiation) { json(json) }
        install(HttpTimeout) {
            requestTimeoutMillis = 2_000
            connectTimeoutMillis = 1_000
        }
        expectSuccess = false
    }

    suspend fun gaze(): GazeStatus = ok(http.get("$baseUrl/api/gaze")).body()

    /** Full calibration (25 dots) followed by the gaze correction (5 crosses). [screen] is in screen points. */
    suspend fun calibrate(screen: Pair<Int, Int>): GazeStatus = post("/api/calibrate", screen)

    suspend fun correct(screen: Pair<Int, Int>): GazeStatus = post("/api/correct", screen)

    suspend fun loadSaved(screen: Pair<Int, Int>): GazeStatus = post("/api/load", screen)

    suspend fun cancel(screen: Pair<Int, Int>): GazeStatus = post("/api/cancel", screen)

    private suspend fun post(path: String, screen: Pair<Int, Int>): GazeStatus {
        val r = http.post("$baseUrl$path") {
            contentType(ContentType.Application.Json)
            setBody(ScreenBody(listOf(screen.first, screen.second)))
        }
        return ok(r).body()
    }

    private suspend fun ok(response: HttpResponse): HttpResponse {
        if (response.status.isSuccess()) return response
        val text = response.bodyAsText()
        val message = runCatching { json.decodeFromString<ErrorBody>(text).error }.getOrNull()
            ?.takeIf { it.isNotBlank() } ?: "Gaze service error (${response.status.value})"
        throw StationException(message)
    }
}
