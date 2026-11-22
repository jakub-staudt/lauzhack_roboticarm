# Gaze Station on Ubuntu: install and setup

For Ubuntu 22.04 and 24.04 (64-bit Intel/AMD). The `.deb` from the
[release page](https://github.com/jakub-staudt/lauzhack_roboticarm/releases) installs the app only. The app also needs
two Python programs from this project, which it starts by itself once they are set up:

- the **simulator** (the 3D arm in the "Gaze station" window), and
- the **eye tracker** (uses the laptop's camera for the "Eye tracking" window).

Until then you will see *"Waiting for the simulator"*, *"Lost connection to the simulator server"* and
*"Waiting for the eye tracker"*. Do the steps below once per PC, in a terminal (Ctrl+Alt+T).

## 1. Install the app

```bash
sudo apt install ./gazestation_1.1.0-1_amd64.deb      # in the folder you downloaded it to
```

It appears as **Gaze Station** in the app menu (it installs to `/opt/gazestation`; remove with
`sudo apt remove gazestation`).

## 2. Install Python 3.11, Git and system libraries

Python 3.11 exactly (newer versions miss some packages). Ubuntu does not ship it, so use the deadsnakes PPA:

```bash
sudo apt update
sudo apt install -y software-properties-common git curl
sudo add-apt-repository -y ppa:deadsnakes/ppa
sudo apt install -y python3.11 python3.11-venv python3.11-dev \
    libgl1 libegl1 libglib2.0-0 libgles2 v4l-utils
python3.11 --version
```

`libgl1`/`libegl1`/`libgles2` are for the simulator's 3D rendering and OpenCV, `v4l-utils` lists cameras.

## 3. Download the project into your home folder

The app looks for the project in `~/lauzhack_roboticarm` (and `~/Documents/LauzHack2026/lauzhack_roboticarm`).

```bash
cd ~
git clone -b kmp-setup https://github.com/jakub-staudt/lauzhack_roboticarm.git
cd lauzhack_roboticarm
```

## 4. Create the two Python environments

About 1–2 GB of downloads, 10–20 minutes:

```bash
python3.11 -m venv .venv     && .venv/bin/pip install --upgrade pip && .venv/bin/pip install -r requirements.txt
python3.11 -m venv .venv-sim && .venv-sim/bin/pip install --upgrade pip && .venv-sim/bin/pip install -r simulator/requirements.txt
```

## 5. Camera

```bash
v4l2-ctl --list-devices        # the built-in camera is usually /dev/video0
groups | grep -w video || sudo usermod -aG video "$USER"   # then log out and in again
```

Close other programs that use the camera (browser video calls, Cheese).

## 6. Start Gaze Station

Quit it if it is open, then start it from the app menu (or run `/opt/gazestation/bin/GazeStation` in a terminal to see
its messages). Within a few seconds:

- **Gaze station** window: the 3D arm and blocks appear (the simulator takes 5–20 s the first time);
- **Eye tracking** window: *"Press SPACE to start"*. Click that window, press SPACE and follow the 25 dots with your
  eyes, keeping your head still; then SPACE again and the 5 crosses. The orange circle then follows your gaze.

Look at a block for 2 s, then at **Pick & place** for 1.5 s, then at a zone for 2 s: the simulated arm moves it.
ESC stops the arm.

## If it does not work

**Logs:** `/tmp/GazeStation/simulator.log` and `/tmp/GazeStation/eye-tracker.log`. Send these when asking for help.

**Check the parts by hand** (app closed), in `~/lauzhack_roboticarm`:

```bash
.venv-sim/bin/python -m simulator.web.server     # then open http://127.0.0.1:8810 in a browser
.venv/bin/python -m gaze_service.server          # then open http://127.0.0.1:8820/api/gaze
```

Start the app while they run and it uses them instead of starting its own.

| Problem | Fix |
|---|---|
| Could not find the project folder | Step 3, or start the app with the folder: `GAZE_REPO=/path/to/lauzhack_roboticarm /opt/gazestation/bin/GazeStation` |
| No Python environment at … | Step 4 not done (or done in another folder) |
| Simulator log mentions EGL / OpenGL | `sudo apt install libegl1 libgles2`, or force software rendering: start the simulator by hand with `MUJOCO_GL=osmesa` (needs `sudo apt install libosmesa6`) |
| No images from camera 0 yet | Step 5; another program has the camera; or another camera: `GAZE_CAMERA=1 /opt/gazestation/bin/GazeStation` |
| The gaze circle is offset from where you look | Calibrate again (K). On Wayland, Java cannot always tell where its window is: log out, click the cog on the login screen, choose **Ubuntu on Xorg**, log in and try again |
| Something else uses port 8820 / 8810 | `ss -ltnp | grep -E ':8820|:8810'` and stop it (e.g. the team's rectangle detector also uses 8820) |

This app moves only the **simulated** arm.
