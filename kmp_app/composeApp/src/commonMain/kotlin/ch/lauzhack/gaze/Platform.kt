package ch.lauzhack.gaze

import androidx.compose.ui.graphics.ImageBitmap

/** Decode PNG/JPEG bytes into an image. Each platform uses its own decoder (Skia on desktop). */
expect fun decodeImage(bytes: ByteArray): ImageBitmap
