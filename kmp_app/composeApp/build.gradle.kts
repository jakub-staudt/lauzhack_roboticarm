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

// Packaging uses an Eclipse Temurin 17 that Gradle downloads into its own cache (Homebrew's JDK is refused by
// jpackage, and this way macOS, Windows and CI all package with the same JDK).
/** Installer version (macOS, Windows and Linux file names use it). */
val appVersion = "1.1.0"

val packagingJdk = javaToolchains.launcherFor {
    languageVersion.set(JavaLanguageVersion.of(17))
    vendor.set(JvmVendorSpec.ADOPTIUM)
}

compose.desktop {
    application {
        mainClass = "ch.lauzhack.gaze.MainKt"
        javaHome = packagingJdk.get().metadata.installationPath.asFile.absolutePath

        nativeDistributions {
            // .dmg must be built on macOS, .msi on Windows, .deb on Linux (each OS packages only for itself; CI builds all).
            targetFormats(TargetFormat.Dmg, TargetFormat.Msi, TargetFormat.Deb)
            packageName = "GazeStation"
            packageVersion = appVersion
            description = "Eye-tracking pick and place for the SO-101 arm (LauzHack 2026)"
            vendor = "LauzHack 2026 team"
            // java.net.http: Snapshot tool; jdk.unsupported: Compose/Skia; java.instrument & co: from suggestRuntimeModules.
            modules("java.instrument", "java.management", "java.net.http", "jdk.unsupported")

            macOS {
                bundleID = "ch.lauzhack.gazestation"
                entitlementsFile.set(project.file("entitlements.plist"))
                runtimeEntitlementsFile.set(project.file("entitlements.plist"))
                iconFile.set(project.file("icons/icon.icns"))
                dockName = "Gaze Station"
                // The app starts the eye tracker, so macOS asks *this app* for the camera.
                infoPlist {
                    extraKeysRawXml = """
                        <key>NSCameraUsageDescription</key>
                        <string>Gaze Station tracks your eyes with the camera so you can choose blocks by looking at them.</string>
                    """.trimIndent()
                }
            }
            windows {
                iconFile.set(project.file("icons/icon.ico"))
                menuGroup = "Gaze Station"
                shortcut = true
                dirChooser = true
                perUserInstall = true   // installs without admin rights
                upgradeUuid = "8B3A6E2C-4F1D-4C7A-9E55-2A7D1C0F6B31"
            }
            linux {
                iconFile.set(project.file("icons/icon.png"))
                packageName = "gazestation"          // Debian package names are lower case
                debMaintainer = "lauzhack2026@users.noreply.github.com"
                menuGroup = "Education"
                appCategory = "Education"
                shortcut = true                      // adds it to the app menu (Activities)
            }
        }
    }
}

// macOS installer for unsigned (ad-hoc) builds. jpackage's own ad-hoc signature turns on the hardened runtime but
// leaves out the entitlements, and then macOS silently refuses the camera to the eye tracker the app starts. This
// signs the app ad-hoc *with* entitlements.plist (camera + what the JVM needs) and wraps it in a .dmg.
// Use this instead of packageDmg until there is a Developer ID to sign with.
val macApp = layout.buildDirectory.dir("compose/binaries/main/app/GazeStation.app").get().asFile
val macDmg = layout.buildDirectory.file("compose/binaries/main/dmg/GazeStation-$appVersion.dmg").get().asFile
val isMac = System.getProperty("os.name").lowercase().contains("mac")

tasks.register<Exec>("signMacApp") {
    group = "compose desktop"
    description = "Ad-hoc sign GazeStation.app with entitlements.plist (camera)"
    dependsOn("createDistributable")
    enabled = isMac
    commandLine("codesign", "--force", "--deep", "--options", "runtime",
        "--entitlements", project.file("entitlements.plist").absolutePath, "-s", "-", macApp.absolutePath)
}

tasks.register<Exec>("packageMacDmg") {
    group = "compose desktop"
    description = "Build GazeStation.dmg (ad-hoc signed with the camera entitlement)"
    dependsOn("signMacApp")
    enabled = isMac
    macDmg.parentFile.mkdirs()
    commandLine("hdiutil", "create", "-volname", "GazeStation", "-srcfolder", macApp.absolutePath, "-ov", "-format", "UDZO", macDmg.absolutePath)
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
