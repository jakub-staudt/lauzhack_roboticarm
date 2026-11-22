package ch.lauzhack.gaze

import androidx.compose.ui.geometry.Offset
import androidx.compose.ui.graphics.ImageBitmap

/** A block the user can look at: [img] is its centre and [box] its outline, both in camera pixels. */
data class Cube(val name: String, val color: String, val img: Offset, val box: List<Offset>)

/**
 * What the gaze scene shows and moves: the simulator ([SimSource]) or the real arm in Poland ([RealArmSource]).
 * All positions are in the camera picture's pixels.
 */
interface SceneSource {
    /** "simulator" or "real arm", for the status line. */
    val label: String
    val baseUrl: String
    val frame: ImageBitmap?
    /** Size of the camera picture (width, height), or null before the first frame. */
    val imageSize: Pair<Float, Float>?
    fun objects(): List<Cube>
    /** The work area (where blocks can be put down) as a 4-corner polygon, or empty when unknown. */
    fun workArea(): List<Offset>
    fun reachable(img: Offset): Boolean = true
    val operations: List<Operation>
    /** Run [op] on [cube] (and [dest] for operations that need one). Returns at once; see [busy] / [progress]. */
    fun execute(op: Operation, cube: Cube, dest: Offset?)
    /** Stop the arm where it is (soft stop). */
    fun stop()
    /** The arm is carrying out a command (the scene then only offers STOP). */
    val busy: Boolean
    /** One line about the running command, e.g. "Step 3 of 7: Lowering onto the block". */
    val progress: String?
    /** The last problem to show in red, or null. */
    val notice: String?
    /** Why the scene cannot take a selection yet (e.g. the table rectangle is still being measured), or null. */
    val notReady: String?
    /** The arm part of the status line. */
    val status: String
}

/** The MuJoCo simulator behind the simulator web server (StationController). */
class SimSource(val station: StationController) : SceneSource {
    override val label = "simulator"
    override val baseUrl: String get() = station.baseUrl
    override val frame get() = station.frame
    override val imageSize: Pair<Float, Float>?
        get() = station.geometry?.let { it.imageWidth.toFloat() to it.imageHeight.toFloat() }

    private fun px(w: World): Offset = station.geometry!!.toPixel(w).let { (x, y) -> Offset(x.toFloat(), y.toFloat()) }

    override fun objects(): List<Cube> {
        if (station.geometry == null) return emptyList()
        return station.cubes.filter { !it.held }.map { c ->
            val h = CUBE_HALF_M
            val box = listOf(-h to -h, h to -h, h to h, -h to h).map { (dx, dy) -> px(World(c.x + dx, c.y + dy)) }
            Cube("${c.color} block", c.color, px(World(c.x, c.y)), box)
        }
    }

    override fun workArea(): List<Offset> {
        val g = station.geometry ?: return emptyList()
        val side = g.sideCm
        return listOf(0.0 to side, side to side, side to 0.0, 0.0 to 0.0).map { (cx, cy) -> px(g.blockToWorld("left", cx, cy)) }
    }

    override fun reachable(img: Offset): Boolean {
        val g = station.geometry ?: return false
        val reach = station.config?.reach?.blocks?.get("left")?.map { it[0] to it[1] }?.toSet() ?: return true
        val bp = g.blockAt(g.toWorld(img.x.toDouble(), img.y.toDouble())) ?: return false
        fun snap(v: Double) = kotlin.math.round(v / 2).toInt() * 2
        return reach.contains(snap(bp.cx) to snap(bp.cy))
    }

    override val operations = listOf(
        Operation("pick_place", "Pick & place", true),
        Operation("point", "Point at it", false),
    )

    override fun execute(op: Operation, cube: Cube, dest: Offset?) {
        val src = cube.img.x.toDouble() to cube.img.y.toDouble()
        when (op.key) {
            "pick_place" -> dest?.let { station.pickPlace(src, it.x.toDouble() to it.y.toDouble()) }
            "point" -> station.goToPx(src)
        }
    }

    override fun stop() = station.stop()
    override val busy get() = false   // the simulator queues nothing: a new command replaces the old one
    override val progress: String? get() = null
    override val notice: String? get() = station.notice?.text
    override val notReady: String? get() = if (station.geometry == null) "Waiting for the simulator at ${station.baseUrl}" else null
    override val status: String get() = "arm: " + station.task.message.ifBlank { station.task.state }

    companion object {
        const val CUBE_HALF_M = 0.02
    }
}
