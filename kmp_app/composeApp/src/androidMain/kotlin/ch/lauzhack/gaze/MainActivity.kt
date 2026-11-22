package ch.lauzhack.gaze

import android.Manifest
import android.content.pm.PackageManager
import android.os.Bundle
import android.view.WindowManager
import androidx.activity.ComponentActivity
import androidx.activity.compose.setContent
import androidx.activity.result.contract.ActivityResultContracts
import androidx.compose.foundation.background
import androidx.compose.foundation.border
import androidx.compose.foundation.clickable
import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Box
import androidx.compose.foundation.layout.BoxWithConstraints
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.Spacer
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.height
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.layout.width
import androidx.compose.foundation.shape.RoundedCornerShape
import androidx.compose.foundation.text.KeyboardOptions
import androidx.compose.material3.OutlinedTextField
import androidx.compose.material3.Text
import androidx.compose.runtime.Composable
import androidx.compose.runtime.DisposableEffect
import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.remember
import androidx.compose.runtime.rememberCoroutineScope
import androidx.compose.runtime.setValue
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.geometry.Offset
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.platform.LocalDensity
import androidx.compose.ui.text.input.KeyboardType
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import androidx.core.view.WindowCompat
import androidx.core.view.WindowInsetsCompat
import androidx.core.view.WindowInsetsControllerCompat
import ch.lauzhack.gaze.api.GazeApi
import ch.lauzhack.gaze.api.StationApi
import ch.lauzhack.gaze.ui.App
import ch.lauzhack.gaze.ui.SceneScreen

/**
 * Android: a thin client. The laptop runs the simulator and the eye tracker (kmp_app/serve_phone.command); the phone
 * shows the same two screens as the desktop app and streams its front camera to the laptop for the eye tracking.
 * Connect over Tailscale (or the same Wi-Fi): type the laptop's address once, it is remembered.
 */
class MainActivity : ComponentActivity() {
    private var hasCamera by mutableStateOf(false)

    private val askCamera = registerForActivityResult(ActivityResultContracts.RequestPermission()) { hasCamera = it }

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        window.addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON)
        WindowCompat.setDecorFitsSystemWindows(window, false)
        WindowInsetsControllerCompat(window, window.decorView).apply {
            hide(WindowInsetsCompat.Type.systemBars())
            systemBarsBehavior = WindowInsetsControllerCompat.BEHAVIOR_SHOW_TRANSIENT_BARS_BY_SWIPE
        }
        hasCamera = checkSelfPermission(Manifest.permission.CAMERA) == PackageManager.PERMISSION_GRANTED
        if (!hasCamera) askCamera.launch(Manifest.permission.CAMERA)

        val prefs = getSharedPreferences("gaze", MODE_PRIVATE)
        setContent { androidx.compose.material3.MaterialTheme(colorScheme = androidx.compose.material3.darkColorScheme()) {
            var laptop by remember { mutableStateOf(prefs.getString("laptop", null)) }
            val address = laptop
            if (address == null) {
                Setup(prefs.getString("laptop_last", "") ?: "") { ip ->
                    prefs.edit().putString("laptop", ip).putString("laptop_last", ip).apply()
                    laptop = ip
                }
            } else {
                Connected(address, hasCamera) {
                    prefs.edit().remove("laptop").apply()
                    laptop = null
                }
            }
        } }
    }

    @Composable
    private fun Connected(address: String, camera: Boolean, onChangeLaptop: () -> Unit) {
        val scope = rememberCoroutineScope()
        val host = address.trim().removePrefix("http://").trimEnd('/').substringBefore(':')
        val station = remember(host) { StationController(StationApi("http://$host:8810"), scope).also { it.start() } }
        val gaze = remember(host) {
            GazeController(GazeApi("http://$host:8820"), scope).also {
                it.launchNote = "On the laptop run kmp_app/serve_phone.command, then check the address: $host"
                it.start()
            }
        }
        val scene = remember(host) { SceneController(SimSource(station), gaze, scope).also { it.start() } }
        val streamer = remember(host) { FrameStreamer(this, "http://$host:8820") }
        DisposableEffect(host, camera) {
            if (camera) streamer.start(this@MainActivity, display?.rotation ?: android.view.Surface.ROTATION_90)
            onDispose { streamer.stop() }
        }
        var controls by remember { mutableStateOf(false) }

        BoxWithConstraints(Modifier.fillMaxSize().background(Color(30, 30, 30))) {
            val d = LocalDensity.current.density
            val wPx = constraints.maxWidth
            val hPx = constraints.maxHeight
            // The app fills the phone screen, so screen fractions map straight to the window.
            gaze.mapper = { nx, ny -> Offset((nx * wPx).toFloat(), (ny * hPx).toFloat()) }
            gaze.screenSize = { (wPx / d).toInt() to (hPx / d).toInt() }

            if (controls) App(station) else SceneScreen(scene)

            // Touch stands in for the keys the desktop prompts ask for.
            if (!controls) Prompts(scene, Modifier.align(Alignment.BottomCenter).padding(bottom = 80.dp))
            Row(Modifier.align(Alignment.TopEnd).padding(10.dp), horizontalArrangement = Arrangement.spacedBy(8.dp)) {
                Chip(if (controls) "Eye tracking" else "Controls") { controls = !controls }
                if (!controls && scene.stage == Stage.Running) {
                    Chip("Cancel") { scene.onKey('C') }
                    Chip("Recalibrate") { scene.onKey('K') }
                }
                Chip("Laptop") { onChangeLaptop() }
            }
            val problem = when {
                !camera -> "Allow the camera for Gaze Station (Android Settings > Apps > Gaze Station > Permissions)."
                streamer.error != null -> streamer.error
                else -> null
            }
            if (problem != null) {
                Text(problem, color = Color(255, 90, 90), fontSize = 13.sp,
                    modifier = Modifier.align(Alignment.BottomStart).padding(12.dp))
            }
        }
    }
}

@Composable
private fun Prompts(scene: SceneController, modifier: Modifier) {
    Row(modifier, horizontalArrangement = Arrangement.spacedBy(16.dp)) {
        when (scene.stage) {
            Stage.SavedPrompt -> {
                Big("Load saved calibration") { scene.onKey('L') }
                Big("Calibrate again") { scene.onKey('K') }
            }
            Stage.CalibIntro -> Big("Start calibration") { scene.onKey(' ') }
            Stage.WorldIntro -> Big("Start") { scene.onKey(' ') }
            else -> {}
        }
    }
}

@Composable
private fun Big(text: String, onClick: () -> Unit) {
    Box(
        Modifier.border(3.dp, Color(235, 235, 235), RoundedCornerShape(14.dp)).background(Color(50, 50, 50), RoundedCornerShape(14.dp))
            .clickable(onClick = onClick).padding(horizontal = 28.dp, vertical = 16.dp),
    ) { Text(text, color = Color(235, 235, 235), fontSize = 20.sp) }
}

@Composable
private fun Chip(text: String, onClick: () -> Unit) {
    Box(
        Modifier.background(Color(0, 0, 0, 150), RoundedCornerShape(8.dp)).clickable(onClick = onClick)
            .padding(horizontal = 12.dp, vertical = 7.dp),
    ) { Text(text, color = Color(220, 220, 220), fontSize = 13.sp) }
}

/** First start: the laptop's Tailscale address (shown by serve_phone.command). */
@Composable
private fun Setup(last: String, onConnect: (String) -> Unit) {
    var ip by remember { mutableStateOf(last) }
    Box(Modifier.fillMaxSize().background(Color(30, 30, 30)), contentAlignment = Alignment.Center) {
        Column(horizontalAlignment = Alignment.CenterHorizontally) {
            Text("Connect to the laptop", color = Color(80, 140, 255), fontSize = 30.sp)
            Spacer(Modifier.height(8.dp))
            Text("On the laptop run kmp_app/serve_phone.command and type the address it shows.",
                color = Color(235, 235, 235), fontSize = 15.sp)
            Text("Phone and laptop must be on the same Tailscale (or Wi-Fi).", color = Color(150, 150, 150), fontSize = 13.sp)
            Spacer(Modifier.height(16.dp))
            Row(verticalAlignment = Alignment.CenterVertically) {
                OutlinedTextField(
                    ip, { ip = it.trim() }, singleLine = true, label = { Text("Laptop address, e.g. 100.x.y.z") },
                    keyboardOptions = KeyboardOptions(keyboardType = KeyboardType.Uri),
                    modifier = Modifier.width(360.dp),
                )
                Spacer(Modifier.width(12.dp))
                Big("Connect") { if (ip.isNotBlank()) onConnect(ip) }
            }
        }
    }
}
