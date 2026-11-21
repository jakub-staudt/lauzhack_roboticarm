package ch.lauzhack.gaze.api

import kotlinx.serialization.SerialName
import kotlinx.serialization.Serializable

/** `GET /api/config`: the overhead camera and the work blocks (see simulator/frame.py). */
@Serializable
data class StationConfig(
    @SerialName("cam_pos") val camPos: List<Double>,
    @SerialName("cam_focal") val camFocal: Double,
    @SerialName("cam_size") val camSize: List<Int>,
    @SerialName("table_z") val tableZ: Double,
    @SerialName("near_x") val nearX: Double,
    @SerialName("origin_y") val originY: Map<String, Double>,
    @SerialName("side_cm") val sideCm: Double,
    val reach: ReachMap? = null,
)

/** Grid points (cm, per block) the arm can reach with a top-down grasp; the rest are drawn as red dots. */
@Serializable
data class ReachMap(
    @SerialName("step_cm") val stepCm: Int,
    val blocks: Map<String, List<List<Int>>>,
)

/** The single running task. `state`: idle, planning, running, done, stopped, error. */
@Serializable
data class TaskState(
    val state: String = "idle",
    val label: String = "",
    val step: Int = 0,
    val total: Int = 0,
    val message: String = "",
) {
    val busy: Boolean get() = state == "running" || state == "planning"
}

/** A cube on the table, in world metres. */
@Serializable
data class Cube(
    val color: String,
    val x: Double,
    val y: Double,
    val z: Double,
    val label: String,
    val held: Boolean,
)

/** `GET /api/state`. */
@Serializable
data class StationState(
    val task: TaskState,
    val blocks: List<Cube>,
    val joints: Map<String, Double> = emptyMap(),
)

@Serializable
internal data class ErrorBody(val error: String = "")

@Serializable
internal data class PickPlaceBody(val src: List<Double>, val dst: List<Double>)

@Serializable
internal data class GoToBody(val at: List<Double>)
