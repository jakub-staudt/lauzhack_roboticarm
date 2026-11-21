# web_ui

A standalone control page for the arm: overhead camera, keyboard hold-to-move, take/release control, STOP.
It replaces the original dashboard for day-to-day use and does not modify anything on the server; it only calls the
controller's public API (the same calls the original dashboard makes).

```bash
python3 server.py                                  # standard library only
python3 server.py --controller http://127.0.0.1:8765 --port 8800   # e.g. through an SSH tunnel
```
Open http://127.0.0.1:8800, click **Take control**, then hold W/A/S/D, R/F, I/K, J/L, Q/E. **Esc** = STOP.

- Only one owner at a time: release control in the original dashboard first.
- `server.py` is a same-origin proxy (the controller rejects cross-origin browser POSTs). It forwards only
  state, the overhead camera, control claim/heartbeat/release, cartesian jog and the stop/reset routes. Torque, joint
  jog/targets, speed and model routes are not reachable from this page.
- Next: overlay the rectangle, selected pick/place points and gaze pointer on the camera image (the eye-tracking
  contract is in `../eye_tracking/README.md`).
