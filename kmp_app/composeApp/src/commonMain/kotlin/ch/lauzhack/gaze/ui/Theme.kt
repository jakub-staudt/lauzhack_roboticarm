package ch.lauzhack.gaze.ui

import androidx.compose.foundation.isSystemInDarkTheme
import androidx.compose.foundation.shape.RoundedCornerShape
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.darkColorScheme
import androidx.compose.material3.lightColorScheme
import androidx.compose.runtime.Composable
import androidx.compose.runtime.CompositionLocalProvider
import androidx.compose.runtime.Immutable
import androidx.compose.runtime.staticCompositionLocalOf
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.unit.dp

/**
 * The palette of simulator/web/static/style.css (`:root` and its `prefers-color-scheme: dark` override).
 * ok / warn / bad are not overridden for dark mode in the CSS, so they are shared.
 */
@Immutable
data class StationColors(
    val bg: Color,
    val surface: Color,
    val ink: Color,
    val muted: Color,
    val line: Color,
    val accent: Color,
    val accentInk: Color,
    val danger: Color,
    val src: Color,
    val dst: Color,
    val ok: Color = Color(0xFF1A9A55),
    val warn: Color = Color(0xFFB26A00),
    val bad: Color = Color(0xFFC0262D),
)

val LightStation = StationColors(
    bg = Color(0xFFF4F5F7), surface = Color(0xFFFFFFFF), ink = Color(0xFF1C2230), muted = Color(0xFF5D6676),
    line = Color(0xFFD9DDE5), accent = Color(0xFF1D5FD6), accentInk = Color(0xFFFFFFFF), danger = Color(0xFFC0262D),
    src = Color(0xFF1A9A55), dst = Color(0xFFE07B00),
)

val DarkStation = StationColors(
    bg = Color(0xFF12151C), surface = Color(0xFF1B2029), ink = Color(0xFFE8EBF2), muted = Color(0xFF98A2B3),
    line = Color(0xFF2C3340), accent = Color(0xFF5B8EF0), accentInk = Color(0xFF0B1220), danger = Color(0xFFEF5A60),
    src = Color(0xFF3FC47C), dst = Color(0xFFF2A03D),
)

/** Fixed colours drawn over the camera image (app.js), the same in light and dark mode. */
object Overlay {
    val blockOutline = Color(25, 70, 190, (0.9 * 255).toInt())
    val labelBg = Color(15, 20, 30, (0.78 * 255).toInt())
    val cornerLabel = Color(0xFF9FC0FF)
    val unreachable = Color(200, 40, 50, (0.55 * 255).toInt())
    val arrow = Color(255, 255, 255, (0.85 * 255).toInt())
    val srcRing = Color(0xFF2FD27A)
    val dstRing = Color(0xFFFFA733)
}

/** Cube swatches (app.js SWATCH). */
fun swatch(color: String): Color = when (color) {
    "red" -> Color(0xFFC8081C)
    "blue" -> Color(0xFF0A40C8)
    "yellow" -> Color(0xFFE09C09)
    else -> Color.Gray
}

/** Corner radii from the CSS: .view / .panel 12px, buttons / .pick / .modes 10px, inner tabs 8px. */
object Radii {
    val card = RoundedCornerShape(12.dp)
    val control = RoundedCornerShape(10.dp)
    val tab = RoundedCornerShape(8.dp)
    val pill = RoundedCornerShape(999.dp)
    val swatch = RoundedCornerShape(3.dp)
}

val LocalStationColors = staticCompositionLocalOf { LightStation }

@Composable
fun StationTheme(dark: Boolean = isSystemInDarkTheme(), content: @Composable () -> Unit) {
    val c = if (dark) DarkStation else LightStation
    val material = if (dark) {
        darkColorScheme(primary = c.accent, onPrimary = c.accentInk, background = c.bg, surface = c.surface,
            onBackground = c.ink, onSurface = c.ink, outline = c.line, error = c.danger)
    } else {
        lightColorScheme(primary = c.accent, onPrimary = c.accentInk, background = c.bg, surface = c.surface,
            onBackground = c.ink, onSurface = c.ink, outline = c.line, error = c.danger)
    }
    CompositionLocalProvider(LocalStationColors provides c) {
        MaterialTheme(colorScheme = material, content = content)
    }
}

object StationTheme {
    val colors: StationColors @Composable get() = LocalStationColors.current
}
