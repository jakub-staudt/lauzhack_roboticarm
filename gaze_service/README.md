# Gaze service

The eye tracker from the `dev` branch prototype (`gaze_pick_place.py`: EyeGestures v3 on the laptop's front camera,
25-dot calibration, 5-cross gaze correction) behind a small HTTP API, so the Kotlin app (`kmp_app`) can run the same
full-screen gaze scene.

## Run

```sh
python3 -m venv .venv && .venv/bin/pip install -r requirements.txt   # the dev branch requirements (eyeGestures, mediapipe, opencv)
.venv/bin/python -m gaze_service.server              # http://127.0.0.1:8820, camera 0
.venv/bin/python -m gaze_service.server --camera 1   # another webcam
.venv/bin/python -m gaze_service.server --mock       # no camera: set the gaze with POST /api/mock {"x":0.5,"y":0.5}
```

On macOS, start it from Terminal (or iTerm) and allow camera access for that app.

Then start the app (`kmp_app`): it asks for the calibration (L = load saved, K / SPACE = calibrate) full screen,
exactly like `gaze_scene_v2.py`, and then shows the scene. Gaze timings and distances are the prototype's
(2 s on an object or zone, 1.5 s on a button, decay when you look away, 0.8 s grace after each step).

## API

`GET /api/gaze` returns phase (`idle`, `calibrating`, `correcting`, `tracking`), camera, face, calibrated, the gaze
point `x`/`y` as screen fractions, `moving` (saccade), and the current calibration `target`. `POST /api/calibrate`,
`/api/correct`, `/api/load`, `/api/cancel` take `{"screen": [w, h]}` in screen points. The calibration is saved to
`gaze_service/eg_calibration.pkl` and `gaze_correction.json` (both ignored by git).
