package ch.lauzhack.gaze.ui

import androidx.compose.foundation.background
import androidx.compose.foundation.border
import androidx.compose.foundation.isSystemInDarkTheme
import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Box
import androidx.compose.foundation.layout.BoxWithConstraints
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.layout.widthIn
import androidx.compose.foundation.layout.width
import androidx.compose.foundation.rememberScrollState
import androidx.compose.foundation.verticalScroll
import androidx.compose.material3.Text
import androidx.compose.runtime.Composable
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.text.style.TextOverflow
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import ch.lauzhack.gaze.StationController

/** The page: header with the status chip, then camera + control panel (two columns from 900 dp wide). */
@Composable
fun App(ctl: StationController, dark: Boolean = isSystemInDarkTheme()) {
    StationTheme(dark) {
        val c = StationTheme.colors
        Column(Modifier.fillMaxSize().background(c.bg)) {
            Header(ctl)
            BoxWithConstraints(Modifier.fillMaxSize()) {
                val wide = maxWidth >= 900.dp
                Box(
                    Modifier.fillMaxSize().verticalScroll(rememberScrollState()),
                    contentAlignment = Alignment.TopCenter,
                ) {
                    val content = Modifier.widthIn(max = 1200.dp).fillMaxWidth()
                        .padding(start = 16.dp, end = 16.dp, bottom = 24.dp)
                    if (wide) {
                        Row(content, horizontalArrangement = Arrangement.spacedBy(16.dp)) {
                            Stage(ctl, Modifier.weight(1f))
                            ControlPanel(ctl, Modifier.width(340.dp))
                        }
                    } else {
                        Column(content, verticalArrangement = Arrangement.spacedBy(16.dp)) {
                            Stage(ctl, Modifier.fillMaxWidth())
                            ControlPanel(ctl, Modifier.fillMaxWidth())
                        }
                    }
                }
            }
        }
    }
}

@Composable
private fun Header(ctl: StationController) {
    val c = StationTheme.colors
    Row(
        Modifier.fillMaxWidth().padding(horizontal = 16.dp, vertical = 12.dp),
        verticalAlignment = Alignment.CenterVertically,
        horizontalArrangement = Arrangement.SpaceBetween,
    ) {
        Text("Gaze station", fontSize = 18.sp, fontWeight = FontWeight.Bold, color = c.ink)
        StatusChip(ctl)
    }
}

/** `.chip` with its `data-state` colours. */
@Composable
private fun StatusChip(ctl: StationController) {
    val c = StationTheme.colors
    val (text, state) = ctl.statusChip()
    val tint = when (state) {
        "running", "planning" -> c.accent
        "done" -> c.ok
        "error" -> c.bad
        "stopped" -> c.warn
        else -> null
    }
    Box(
        Modifier
            .widthIn(max = 560.dp)
            .background(c.surface, Radii.pill)
            .border(1.dp, tint ?: c.line, Radii.pill)
            .padding(horizontal = 12.dp, vertical = 4.dp),
    ) {
        Text(text, color = tint ?: c.muted, fontSize = 14.sp, maxLines = 1, overflow = TextOverflow.Ellipsis)
    }
}

/** `.stage`: the camera and the hover readout below it. */
@Composable
private fun Stage(ctl: StationController, modifier: Modifier) {
    Column(modifier) {
        CameraView(ctl)
        Text(
            ctl.hover ?: "Tap the picture",
            color = StationTheme.colors.muted, fontSize = 15.sp,
            modifier = Modifier.padding(top = 8.dp),
        )
    }
}
