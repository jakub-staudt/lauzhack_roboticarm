package ch.lauzhack.gaze

import java.io.File
import java.net.InetSocketAddress
import java.net.Socket
import java.net.URI

/**
 * Starts the Python parts of the project when they are not already running, so the app works both from a checkout
 * (`./gradlew run`) and installed (.dmg / .msi):
 * - the eye tracker: `.venv` + `python -m gaze_service.server --camera $GAZE_CAMERA` (default 0, the built-in camera);
 * - the simulator:   `.venv-sim` + `python -m simulator.web.server`.
 *
 * The repository is found from $GAZE_REPO, then the working directory and its parents, then a few usual places.
 * Output goes to <tmp>/GazeStation/<name>.log. Processes started here are stopped when the app quits.
 *
 * macOS camera permission belongs to the app that started the tracker: the installed app asks for it itself
 * (NSCameraUsageDescription); `./gradlew run` from Terminal uses Terminal's (run.command starts the tracker first).
 */
object LocalServices {
    private val windows = System.getProperty("os.name").lowercase().contains("win")
    private val mac = System.getProperty("os.name").lowercase().contains("mac")
    val logDir = File(System.getProperty("java.io.tmpdir"), "GazeStation").apply { mkdirs() }

    fun findRepo(): File? {
        val home = File(System.getProperty("user.home"))
        val candidates = buildList {
            System.getenv("GAZE_REPO")?.let { add(File(it)) }
            var d: File? = File(System.getProperty("user.dir")).absoluteFile
            while (d != null) { add(d); d = d.parentFile }
            for (rel in listOf("Documents/LauzHack2026/lauzhack_roboticarm", "Documents/lauzhack_roboticarm", "lauzhack_roboticarm",
                "source/repos/lauzhack_roboticarm", "Desktop/lauzhack_roboticarm")) add(File(home, rel))
        }
        return candidates.firstOrNull { File(it, "gaze_service/server.py").exists() }
    }

    private fun python(root: File, venv: String) = File(root, if (windows) "$venv/Scripts/python.exe" else "$venv/bin/python")

    private fun localPort(url: String, default: Int): Int? {
        val uri = runCatching { URI(url) }.getOrNull() ?: return null
        if (uri.host !in setOf("127.0.0.1", "localhost")) return null
        return if (uri.port > 0) uri.port else default
    }

    private fun answering(port: Int) =
        runCatching { Socket().use { it.connect(InetSocketAddress("127.0.0.1", port), 300) } }.isSuccess

    private fun launch(name: String, root: File, cmd: List<String>): Process {
        val log = File(logDir, "$name.log")
        val p = ProcessBuilder(cmd).directory(root).redirectErrorStream(true).redirectOutput(log).start()
        Runtime.getRuntime().addShutdownHook(Thread { p.destroy() })
        println("Started $name (log: ${log.path})")
        return p
    }

    /** Returns a line for the eye tracking window's waiting screen, or null when nothing needed doing. */
    fun startGazeService(url: String): String? {
        val port = localPort(url, 8820) ?: return null
        if (answering(port)) return null
        val root = findRepo() ?: return "Could not find the project folder. Set GAZE_REPO to it, or start the eye tracker by hand."
        val py = python(root, ".venv")
        if (!py.exists()) return "No Python environment at ${py.path} (see gaze_service/README.md)."
        val camera = System.getenv("GAZE_CAMERA") ?: "0"
        return try {
            launch("eye-tracker", root, listOf(py.path, "-m", "gaze_service.server", "--port", "$port", "--camera", camera))
            "Starting the eye tracker on camera $camera (${if (mac) "MacBook camera" else "built-in camera"})…"
        } catch (e: Exception) {
            "Could not start the eye tracker: ${e.message}"
        }
    }

    /** Starts the simulator if [url] is local and not answering (it takes a few seconds to load MuJoCo). */
    fun startSimulator(url: String) {
        val port = localPort(url, 8810) ?: return
        if (answering(port)) return
        val root = findRepo() ?: return println("Simulator not running and project folder not found (set GAZE_REPO).")
        val py = python(root, ".venv-sim")
        if (!py.exists()) return println("Simulator not running and no ${py.path} (see simulator/README.md).")
        runCatching { launch("simulator", root, listOf(py.path, "-m", "simulator.web.server", "--port", "$port")) }
            .onFailure { println("Could not start the simulator: ${it.message}") }
    }
}
