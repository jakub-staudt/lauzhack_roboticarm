import org.jetbrains.compose.desktop.application.dsl.TargetFormat

plugins {
    alias(libs.plugins.kotlin.multiplatform)
    alias(libs.plugins.kotlin.compose)
    alias(libs.plugins.kotlin.serialization)
    alias(libs.plugins.compose.multiplatform)
}

kotlin {
    jvmToolchain(17)

    // Desktop (macOS / Windows / Linux) for now. Android, iOS and web targets can be added later:
    // all UI and logic live in commonMain; only image decoding and the HTTP engine are per platform.
    jvm("desktop")

    sourceSets {
        commonMain.dependencies {
            implementation(compose.runtime)
            implementation(compose.foundation)
            implementation(compose.material3)
            implementation(compose.ui)
            implementation(libs.ktor.client.core)
            implementation(libs.ktor.client.content.negotiation)
            implementation(libs.ktor.serialization.kotlinx.json)
            implementation(libs.kotlinx.coroutines.core)
            implementation(libs.kotlinx.serialization.json)
        }
        val desktopMain by getting {
            dependencies {
                implementation(compose.desktop.currentOs)
                implementation(libs.ktor.client.cio)
                implementation(libs.kotlinx.coroutines.swing)
            }
        }
    }
}

compose.desktop {
    application {
        mainClass = "ch.lauzhack.gaze.MainKt"
        nativeDistributions {
            targetFormats(TargetFormat.Dmg, TargetFormat.Msi, TargetFormat.Deb)
            packageName = "GazeStation"
            packageVersion = "1.0.0"
        }
    }
}

// Dev tool: render the UI off-screen to PNGs (see Snapshot.kt). Needs the station server running.
tasks.register<JavaExec>("snapshot") {
    group = "verification"
    description = "Render the UI off-screen against a running station server"
    val desktop = kotlin.targets.getByName("desktop").compilations.getByName("main")
    dependsOn(desktop.compileTaskProvider)
    classpath = files(desktop.output.allOutputs, desktop.runtimeDependencyFiles)
    mainClass.set("ch.lauzhack.gaze.SnapshotKt")
    jvmArgs("-Djava.awt.headless=true")
}
