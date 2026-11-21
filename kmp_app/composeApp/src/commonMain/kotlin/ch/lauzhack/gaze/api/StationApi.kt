package ch.lauzhack.gaze.api

import io.ktor.client.HttpClient
import io.ktor.client.call.body
import io.ktor.client.plugins.HttpTimeout
import io.ktor.client.plugins.contentnegotiation.ContentNegotiation
import io.ktor.client.request.get
import io.ktor.client.request.header
import io.ktor.client.request.post
import io.ktor.client.request.setBody
import io.ktor.client.statement.HttpResponse
import io.ktor.client.statement.bodyAsText
import io.ktor.http.ContentType
import io.ktor.http.contentType
import io.ktor.http.isSuccess
import io.ktor.serialization.kotlinx.json.json
import kotlinx.serialization.json.Json

/** A refused request; [message] is the server's own explanation, ready to show to the user. */
class StationException(message: String) : Exception(message)

/**
 * Client for the gaze station server (`python -m simulator.web.server`, default http://127.0.0.1:8810).
 * Same endpoints as the web page: frame, state, config, pick_place, go_to, home, stop, reset_blocks.
 */
class StationApi(val baseUrl: String) {
    private val json = Json { ignoreUnknownKeys = true }

    private val http = HttpClient {
        install(ContentNegotiation) { json(json) }
        install(HttpTimeout) {
            requestTimeoutMillis = 10_000
            connectTimeoutMillis = 3_000
        }
        expectSuccess = false
    }

    suspend fun config(): StationConfig = ok(http.get("$baseUrl/api/config")).body()

    suspend fun state(): StationState = ok(http.get("$baseUrl/api/state")).body()

    /** The overhead camera as PNG bytes. */
    suspend fun frame(): ByteArray = ok(http.get("$baseUrl/frame.png") { header("Cache-Control", "no-store") }).body()

    /** Pick the cube at [src] and put it at [dst]; both are overhead-camera pixels. */
    suspend fun pickAndPlace(src: Pair<Double, Double>, dst: Pair<Double, Double>): TaskState =
        post("/api/pick_place", PickPlaceBody(src.toList(), dst.toList()))

    /** Move the gripper tip over [at] (camera pixels). */
    suspend fun goTo(at: Pair<Double, Double>): TaskState = post("/api/go_to", GoToBody(at.toList()))

    suspend fun home(): TaskState = post("/api/home", emptyMap<String, String>())

    suspend fun stop(): TaskState = post("/api/stop", emptyMap<String, String>())

    suspend fun resetBlocks(): TaskState = post("/api/reset_blocks", emptyMap<String, String>())

    private suspend inline fun <reified T> post(path: String, body: T): TaskState {
        val response = http.post("$baseUrl$path") {
            contentType(ContentType.Application.Json)
            setBody(body)
        }
        return ok(response).body()
    }

    private suspend fun ok(response: HttpResponse): HttpResponse {
        if (response.status.isSuccess()) return response
        val text = response.bodyAsText()
        val message = runCatching { json.decodeFromString<ErrorBody>(text).error }.getOrNull()
            ?.takeIf { it.isNotBlank() } ?: "Request failed (${response.status.value})"
        throw StationException(message)
    }

    fun close() = http.close()
}
