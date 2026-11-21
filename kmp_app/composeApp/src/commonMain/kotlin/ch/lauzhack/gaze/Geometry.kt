package ch.lauzhack.gaze

import ch.lauzhack.gaze.api.StationConfig
import kotlin.math.round

/** A point on the table in world metres (x forward from the robot, y to its left). */
data class World(val x: Double, val y: Double)

/** A point in a block's own frame, in cm: (0, 0) near-left corner, (20, 20) far-right corner. */
data class BlockPoint(val block: String, val cx: Double, val cy: Double)

/**
 * Camera and block coordinates, mirroring simulator/frame.py (and the web page's app.js).
 * Only the left block is the working area.
 */
class Geometry(private val cfg: StationConfig) {
    val imageWidth: Int get() = cfg.camSize[0]
    val imageHeight: Int get() = cfg.camSize[1]
    val sideCm: Double get() = cfg.sideCm
    val workBlocks = listOf("left")

    private val depth get() = cfg.camPos[2] - cfg.tableZ

    /** Overhead-camera pixel -> world point on the table. */
    fun toWorld(px: Double, py: Double): World = World(
        cfg.camPos[0] + (px - (imageWidth - 1) / 2.0) / cfg.camFocal * depth,
        cfg.camPos[1] - (py - (imageHeight - 1) / 2.0) / cfg.camFocal * depth,
    )

    /** World point on the table -> overhead-camera pixel. */
    fun toPixel(w: World): Pair<Double, Double> = Pair(
        (w.x - cfg.camPos[0]) / depth * cfg.camFocal + (imageWidth - 1) / 2.0,
        -(w.y - cfg.camPos[1]) / depth * cfg.camFocal + (imageHeight - 1) / 2.0,
    )

    /** (cx, cy) cm in [block]'s frame -> world. */
    fun blockToWorld(block: String, cx: Double, cy: Double): World =
        World(cfg.nearX + cy / 100.0, cfg.originY.getValue(block) - cx / 100.0)

    /** The working block containing [w] (± [marginCm]), or null. */
    fun blockAt(w: World, marginCm: Double = 0.0): BlockPoint? {
        for (name in workBlocks) {
            val cx = (cfg.originY.getValue(name) - w.y) * 100.0
            val cy = (w.x - cfg.nearX) * 100.0
            if (cx >= -marginCm && cx <= sideCm + marginCm && cy >= -marginCm && cy <= sideCm + marginCm) {
                return BlockPoint(name, cx, cy)
            }
        }
        return null
    }

    /** "(12.0, 8.5)" inside the working block, otherwise "outside the block". */
    fun describe(w: World): String {
        val hit = blockAt(w, 0.5) ?: return "outside the block"
        return "(${hit.cx.fmt1()}, ${hit.cy.fmt1()})"
    }

    /** Grid points of [block] the arm cannot reach (cm), from the server's reach map. */
    fun unreachable(block: String): List<Pair<Int, Int>> {
        val reach = cfg.reach ?: return emptyList()
        val ok = reach.blocks[block].orEmpty().map { it[0] to it[1] }.toSet()
        val side = sideCm.toInt()
        val out = mutableListOf<Pair<Int, Int>>()
        for (cy in 0..side step reach.stepCm) for (cx in 0..side step reach.stepCm) {
            if ((cx to cy) !in ok) out += cx to cy
        }
        return out
    }
}

internal fun Double.fmt1(): String {
    val r = round(this * 10) / 10
    val s = r.toString()
    return if ('.' in s) s else "$s.0"
}
