package ch.lauzhack.gaze

import android.content.Context
import android.graphics.Bitmap
import android.graphics.Matrix
import android.util.Size
import android.view.Surface
import androidx.camera.core.CameraSelector
import androidx.camera.core.ImageAnalysis
import androidx.camera.core.ImageProxy
import androidx.camera.core.resolutionselector.ResolutionSelector
import androidx.camera.core.resolutionselector.ResolutionStrategy
import androidx.camera.lifecycle.ProcessCameraProvider
import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.setValue
import androidx.core.content.ContextCompat
import androidx.lifecycle.LifecycleOwner
import java.io.ByteArrayOutputStream
import java.net.HttpURLConnection
import java.net.URL
import java.util.concurrent.Executors
import java.util.concurrent.atomic.AtomicBoolean

/**
 * Sends the front camera to the laptop's eye tracker (`gaze_service.server --phone`): small upright JPEGs,
 * about 15 per second, never more than one on the way (so a slow link drops frames instead of piling them up).
 * The laptop runs EyeGestures on them; the app reads the gaze from /api/gaze as on the desktop.
 */
class FrameStreamer(private val context: Context, private val gazeUrl: String) {
    var sent by mutableStateOf(0); private set
    var error by mutableStateOf<String?>(null); private set

    private val executor = Executors.newSingleThreadExecutor()
    private val busy = AtomicBoolean(false)
    private var last = 0L
    private var provider: ProcessCameraProvider? = null

    fun start(owner: LifecycleOwner, displayRotation: Int = Surface.ROTATION_90) {
        val future = ProcessCameraProvider.getInstance(context)
        future.addListener({
            val p = future.get()
            provider = p
            val analysis = ImageAnalysis.Builder()
                .setResolutionSelector(
                    ResolutionSelector.Builder()
                        .setResolutionStrategy(ResolutionStrategy(Size(640, 480), ResolutionStrategy.FALLBACK_RULE_CLOSEST_HIGHER_THEN_LOWER))
                        .build(),
                )
                .setTargetRotation(displayRotation)
                .setBackpressureStrategy(ImageAnalysis.STRATEGY_KEEP_ONLY_LATEST)
                .setOutputImageFormat(ImageAnalysis.OUTPUT_IMAGE_FORMAT_RGBA_8888)
                .build()
            analysis.setAnalyzer(executor) { image -> handle(image) }
            try {
                p.unbindAll()
                p.bindToLifecycle(owner, CameraSelector.DEFAULT_FRONT_CAMERA, analysis)
                error = null
            } catch (e: Exception) {
                error = "Front camera unavailable: ${e.message}"
            }
        }, ContextCompat.getMainExecutor(context))
    }

    fun stop() {
        provider?.unbindAll()
    }

    private fun handle(image: ImageProxy) {
        try {
            val now = System.currentTimeMillis()
            if (now - last < 66 || busy.get()) return
            last = now
            val jpeg = toUprightJpeg(image)
            busy.set(true)
            Thread {
                try {
                    post(jpeg)
                    sent++
                    error = null
                } catch (e: Exception) {
                    error = "Cannot reach the eye tracker at $gazeUrl (${e.javaClass.simpleName})"
                } finally {
                    busy.set(false)
                }
            }.start()
        } finally {
            image.close()
        }
    }

    private fun toUprightJpeg(image: ImageProxy): ByteArray {
        var bmp = image.toBitmap()
        val rotation = image.imageInfo.rotationDegrees
        if (rotation != 0) {
            val m = Matrix().apply { postRotate(rotation.toFloat()) }
            bmp = Bitmap.createBitmap(bmp, 0, 0, bmp.width, bmp.height, m, true)
        }
        return ByteArrayOutputStream().use { out ->
            bmp.compress(Bitmap.CompressFormat.JPEG, 70, out)
            out.toByteArray()
        }
    }

    private fun post(jpeg: ByteArray) {
        val c = URL("$gazeUrl/api/frame").openConnection() as HttpURLConnection
        try {
            c.requestMethod = "POST"
            c.connectTimeout = 2000
            c.readTimeout = 3000
            c.doOutput = true
            c.setRequestProperty("Content-Type", "image/jpeg")
            c.setFixedLengthStreamingMode(jpeg.size)
            c.outputStream.use { it.write(jpeg) }
            val code = c.responseCode
            c.inputStream.use { it.readBytes() }
            if (code != 200) error("HTTP $code")
        } finally {
            c.disconnect()
        }
    }
}
