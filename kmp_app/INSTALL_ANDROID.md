# Gaze Station on Android

The phone app is a **thin client**: it shows the two screens (eye tracking and controls) and sends its front camera
to a laptop, which runs the simulator and the eye tracker. Phone and laptop connect over **Tailscale** (or the same
Wi-Fi).

## Laptop (once per session)

The laptop needs the project and its two Python environments (see [README](README.md) / the install guides).
Then, in Terminal:

```bash
~/Documents/LauzHack2026/lauzhack_roboticarm/kmp_app/serve_phone.command     # Windows: serve_phone.bat
```

Keep that window open. It prints the laptop's **Tailscale address** (e.g. `100.x.y.z`). If macOS asks whether Python
may accept incoming connections, click **Allow**.

## Phone (once)

1. Install **Tailscale** from the Play Store and sign in to the same account as the laptop.
2. On the phone, open the latest release
   (<https://github.com/jakub-staudt/lauzhack_roboticarm/releases>) and download **GazeStation-1.2.0.apk**.
   Opening it asks to allow installing apps from your browser: allow, then **Install**.
   (Play Protect may warn about an unknown developer: *More details > Install anyway*.)
3. Open **Gaze Station**, allow the camera, type the laptop address, **Connect**.

## Use

- Put the phone on a stand in landscape, at about arm's length, with your face well lit.
- **Calibration:** tap *Start calibration* (or *Load saved calibration*), follow the 25 dots with your eyes keeping your
  head still, then *Start* and the 5 crosses.
- Look at a block for 2 s, then at **Pick & place** for 1.5 s, then at a zone for 2 s: the simulated arm moves it.
- Buttons at the top right: **Controls** (the simulation with the click controls), **Cancel**, **Recalibrate**,
  **Laptop** (change the address).

## If it does not work

| Message | Fix |
|---|---|
| Cannot reach the eye tracker at … | Laptop: is `serve_phone.command` running? Phone: is Tailscale on? Address right? (tap **Laptop**) |
| Waiting for the eye tracker / for the simulator | Same as above; the simulator takes up to 20 s to start |
| Allow the camera for Gaze Station | Android Settings > Apps > Gaze Station > Permissions > Camera |
| Waiting for the phone's camera | The app is not sending frames: reopen it, check the camera permission |
| No face detected | More light on your face, centre your face, phone at arm's length |

Only the **simulated** arm moves. Calibrations made on a phone are for that phone; recalibrate when switching between
the desktop app and the phone.
