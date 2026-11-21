package ch.lauzhack.gaze.ui

import androidx.compose.foundation.BorderStroke
import androidx.compose.foundation.background
import androidx.compose.foundation.border
import androidx.compose.foundation.clickable
import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Box
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.ExperimentalLayoutApi
import androidx.compose.foundation.layout.FlowRow
import androidx.compose.foundation.layout.PaddingValues
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.RowScope
import androidx.compose.foundation.layout.Spacer
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.height
import androidx.compose.foundation.layout.heightIn
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.layout.size
import androidx.compose.foundation.shape.CircleShape
import androidx.compose.material3.Checkbox
import androidx.compose.material3.CheckboxDefaults
import androidx.compose.material3.OutlinedButton
import androidx.compose.material3.Button
import androidx.compose.material3.ButtonDefaults
import androidx.compose.material3.Text
import androidx.compose.runtime.Composable
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.draw.clip
import androidx.compose.ui.draw.shadow
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.unit.Dp
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import ch.lauzhack.gaze.Mode
import ch.lauzhack.gaze.StationController

/** The right-hand `.panel` of the web page. */
@OptIn(ExperimentalLayoutApi::class)
@Composable
fun ControlPanel(ctl: StationController, modifier: Modifier = Modifier) {
    val c = StationTheme.colors
    Column(
        modifier
            .background(c.surface, Radii.card)
            .border(1.dp, c.line, Radii.card)
            .padding(16.dp),
    ) {
        ModeTabs(ctl)
        Text(ctl.hint, color = c.muted, fontSize = 15.sp, modifier = Modifier.padding(vertical = 12.dp))

        Column(verticalArrangement = Arrangement.spacedBy(8.dp), modifier = Modifier.padding(bottom = 14.dp)) {
            if (ctl.mode == Mode.MoveBlock) {
                PickCard(c.src, "Block", ctl.src?.let { "${it.color} at ${ctl.describe(it)}" } ?: "not chosen")
            }
            PickCard(c.dst, "Destination", ctl.describe(ctl.dst))
        }

        Row(horizontalArrangement = Arrangement.spacedBy(8.dp), verticalAlignment = Alignment.CenterVertically) {
            PrimaryButton("Go", enabled = ctl.canGo, onClick = ctl::execute)
            PlainButton("Clear", onClick = ctl::clear)
            PlainButton("Stop", onClick = ctl::stop, danger = true)
        }
        // `.actions.small` wraps (flex-wrap) when the checkbox doesn't fit beside the buttons.
        FlowRow(
            horizontalArrangement = Arrangement.spacedBy(8.dp),
            verticalArrangement = Arrangement.spacedBy(4.dp),
            itemVerticalAlignment = Alignment.CenterVertically,
            modifier = Modifier.padding(top = 8.dp),
        ) {
            PlainButton("Home", onClick = ctl::home, small = true)
            PlainButton("Reset blocks", onClick = ctl::resetBlocks, small = true)
            Row(
                verticalAlignment = Alignment.CenterVertically,
                modifier = Modifier.clip(Radii.tab).clickable { ctl.askBeforeMoving = !ctl.askBeforeMoving },
            ) {
                Checkbox(
                    checked = ctl.askBeforeMoving,
                    onCheckedChange = { ctl.askBeforeMoving = it },
                    colors = CheckboxDefaults.colors(checkedColor = c.accent, uncheckedColor = c.muted),
                )
                Text("Ask before moving", color = c.muted, fontSize = 14.sp, softWrap = false,
                    modifier = Modifier.padding(end = 8.dp))
            }
        }

        SectionTitle("Blocks on the table")
        Column(verticalArrangement = Arrangement.spacedBy(6.dp)) {
            for (cube in ctl.cubes) {
                Row(verticalAlignment = Alignment.CenterVertically) {
                    Box(Modifier.size(14.dp).background(swatch(cube.color), Radii.swatch))
                    Spacer(Modifier.size(10.dp))
                    Text(
                        "${cube.color.replaceFirstChar { it.uppercase() }}: " +
                            if (cube.held) "in the gripper" else cube.label,
                        color = c.ink, fontSize = 16.sp,
                    )
                }
            }
            if (ctl.cubes.isEmpty()) Text("—", color = c.muted)
        }
        Text(
            "Coordinates are in centimetres from the robot's point of view: (0, 0) is the block's near-left " +
                "corner and (20, 20) its far-right corner.",
            color = c.muted, fontSize = 13.sp, modifier = Modifier.padding(top = 16.dp),
        )
    }
}

/** The `.modes` segmented control. */
@Composable
private fun ModeTabs(ctl: StationController) {
    val c = StationTheme.colors
    Row(
        Modifier.fillMaxWidth().background(c.bg, Radii.control).padding(4.dp),
        horizontalArrangement = Arrangement.spacedBy(4.dp),
    ) {
        Tab("Move a block", ctl.mode == Mode.MoveBlock) { ctl.changeMode(Mode.MoveBlock) }
        Tab("Move the gripper", ctl.mode == Mode.MoveGripper) { ctl.changeMode(Mode.MoveGripper) }
    }
}

@Composable
private fun RowScope.Tab(text: String, selected: Boolean, onClick: () -> Unit) {
    val c = StationTheme.colors
    Box(
        Modifier
            .weight(1f)
            .then(if (selected) Modifier.shadow(1.dp, Radii.tab) else Modifier)
            .clip(Radii.tab)
            .background(if (selected) c.surface else Color.Transparent)
            .clickable(onClick = onClick)
            .padding(vertical = 10.dp, horizontal = 8.dp),
        contentAlignment = Alignment.Center,
    ) {
        Text(text, color = if (selected) c.ink else c.muted,
            fontWeight = if (selected) FontWeight.SemiBold else FontWeight.Normal, fontSize = 15.sp)
    }
}

/** A `.pick` card: coloured dot, small bold caption, value. */
@Composable
private fun PickCard(dot: Color, caption: String, value: String) {
    val c = StationTheme.colors
    Row(
        Modifier.fillMaxWidth().border(1.dp, c.line, Radii.control).padding(horizontal = 12.dp, vertical = 10.dp),
        verticalAlignment = Alignment.CenterVertically,
    ) {
        Box(Modifier.size(14.dp).background(dot, CircleShape))
        Spacer(Modifier.size(10.dp))
        Column {
            Text(caption, color = c.muted, fontSize = 13.sp, fontWeight = FontWeight.SemiBold)
            Text(value, color = c.ink, fontSize = 16.sp)
        }
    }
}

@Composable
private fun SectionTitle(text: String) {
    Text(
        text.uppercase(), color = StationTheme.colors.muted, fontSize = 13.sp, fontWeight = FontWeight.SemiBold,
        letterSpacing = 0.6.sp, modifier = Modifier.padding(top = 20.dp, bottom = 8.dp),
    )
}

/** `button.primary`: accent fill, grows to fill the row. */
@Composable
private fun RowScope.PrimaryButton(text: String, enabled: Boolean, onClick: () -> Unit) {
    val c = StationTheme.colors
    Button(
        onClick = onClick, enabled = enabled, shape = Radii.control,
        colors = ButtonDefaults.buttonColors(
            containerColor = c.accent, contentColor = c.accentInk,
            disabledContainerColor = c.accent.copy(alpha = 0.45f), disabledContentColor = c.accentInk.copy(alpha = 0.8f),
        ),
        contentPadding = PaddingValues(horizontal = 16.dp, vertical = 10.dp),
        modifier = Modifier.weight(1f).heightIn(min = 44.dp),
    ) { Text(text, fontWeight = FontWeight.SemiBold, fontSize = 16.sp) }
}

/** Default `button` (surface + line border); `danger` = red text and border; `small` = `.actions.small`. */
@Composable
private fun PlainButton(text: String, onClick: () -> Unit, danger: Boolean = false, small: Boolean = false) {
    val c = StationTheme.colors
    val minHeight: Dp = if (small) 38.dp else 44.dp
    OutlinedButton(
        onClick = onClick, shape = Radii.control,
        border = BorderStroke(1.dp, if (danger) c.danger else c.line),
        colors = ButtonDefaults.outlinedButtonColors(containerColor = c.surface, contentColor = if (danger) c.danger else c.ink),
        contentPadding = if (small) PaddingValues(horizontal = 12.dp, vertical = 6.dp) else PaddingValues(horizontal = 16.dp, vertical = 10.dp),
        modifier = Modifier.heightIn(min = minHeight),
    ) {
        Text(text, fontSize = if (small) 14.sp else 16.sp, fontWeight = if (danger) FontWeight.SemiBold else FontWeight.Normal)
    }
}
