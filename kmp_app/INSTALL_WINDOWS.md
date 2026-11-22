# Gaze Station on Windows: setup after installing the .msi

The `.msi` from the [release page](https://github.com/jakub-staudt/lauzhack_roboticarm/releases/tag/kmp-v1.0.0)
installs the app only. The app also needs two Python programs from this project:

- the **simulator** (the 3D arm in the "Gaze station" window), and
- the **eye tracker** (uses the laptop's camera for the "Eye tracking" window).

The app starts both by itself once they are set up. Until then you will see *"Waiting for the simulator"*,
*"Lost connection to the simulator server"* and *"Could not find the project folder"*. Do the steps below once per PC.

## 1. Install Python 3.11 and Git

1. **Python 3.11** (exactly 3.11; newer versions miss some packages): download the *Windows installer (64-bit)* from
   <https://www.python.org/downloads/release/python-3119/>. In the first installer screen tick
   **"Add python.exe to PATH"**, then *Install Now*.
2. **Git**: <https://git-scm.com/download/win>, default options.

Check in a new PowerShell window (Start menu > type *PowerShell*):

```powershell
py -3.11 --version     # Python 3.11.x
git --version
```

## 2. Download the project into your user folder

The app looks for the project in your user folder, `C:\Users\<you>\lauzhack_roboticarm`. Use that place:
on many Windows 10 PCs *Documents* is moved into OneDrive, and the app does not look there.

```powershell
cd $HOME
git clone -b kmp-setup https://github.com/jakub-staudt/lauzhack_roboticarm.git
cd lauzhack_roboticarm
```

## 3. Create the two Python environments

Still in `C:\Users\<you>\lauzhack_roboticarm`. This downloads about 1–2 GB and can take 10–20 minutes.

```powershell
py -3.11 -m venv .venv
.venv\Scripts\python -m pip install --upgrade pip
.venv\Scripts\pip install -r requirements.txt                 # eye tracker (EyeGestures, mediapipe, OpenCV)

py -3.11 -m venv .venv-sim
.venv-sim\Scripts\python -m pip install --upgrade pip
.venv-sim\Scripts\pip install -r simulator\requirements.txt   # simulator (MuJoCo)
```

If PowerShell refuses to run scripts, use these commands in *Command Prompt* (cmd) instead; they are the same.

## 4. Allow the camera

Settings > Privacy > Camera:

- **Allow apps to access your camera**: On
- **Allow desktop apps to access your camera**: On (at the bottom of that page)

Close other programs that use the camera (Teams, Zoom, the Camera app).

## 5. Start Gaze Station

Close Gaze Station completely if it is open, then start it from the Start menu (*Gaze Station*). Within a few seconds:

- **Gaze station** window: the 3D arm and blocks appear (the simulator takes 5–20 s to load the first time);
- **Eye tracking** window: *"Press SPACE to start"*. Click that window, press SPACE and follow the 25 dots with your
  eyes, keeping your head still; then SPACE again and the 5 crosses. The orange circle then follows your gaze.

Look at a block for 2 s, then at **Pick & place** for 1.5 s, then at a zone for 2 s: the simulated arm moves it.
ESC stops the arm.

## If it does not work

**The project is somewhere else.** Tell the app where it is (then restart the app):

```powershell
setx GAZE_REPO "C:\path\to\lauzhack_roboticarm"
```

**Wrong camera** (e.g. an external webcam is used instead of the built-in one): choose another camera, then restart:

```powershell
setx GAZE_CAMERA 1
```

**Logs.** The app writes what the simulator and the eye tracker print to `%TEMP%\GazeStation\`
(paste `%TEMP%\GazeStation` into the Explorer address bar):

- `simulator.log`
- `eye-tracker.log`

Send these to the team when asking for help.

**Check the parts by hand.** With the app closed, in `C:\Users\<you>\lauzhack_roboticarm`:

```powershell
.venv-sim\Scripts\python -m simulator.web.server     # then open http://127.0.0.1:8810 in a browser
.venv\Scripts\python -m gaze_service.server          # then open http://127.0.0.1:8820/api/gaze
```

If either prints an error, that is the part to fix. Start the app while they run and it uses them instead of starting
its own.

| Message | Meaning |
|---|---|
| Could not find the project folder | Step 2 not done, or the folder is elsewhere: set `GAZE_REPO` |
| No Python environment at … | Step 3 not done (or done in another folder) |
| Waiting for the simulator / Lost connection | Simulator not started yet or crashed: wait 20 s, then see `simulator.log` |
| No images from camera 0 yet | Step 4, another program has the camera, or try `GAZE_CAMERA=1` |
| No face detected | Light on your face, sit centred about an arm's length from the screen |

This app moves only the **simulated** arm.
