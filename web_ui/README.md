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

## Running it smoothly on a laggy link (what worked, 2026-11-22)

The arm is reached over a relayed Tailscale path (60-600 ms, spikes, and a two-minute outage was seen). Manual control felt good again with:

* **A fresh proxy.** An old proxy (4.5 hours old) kept port 8800 and answered every request with `502` after the link had dropped.
  Check with `ss -ltnp | grep :8800`, stop the old process by its pid (not `pkill -f`), then start the proxy again:
  `cd web_ui && ../robotic_arm/.venv/bin/python server.py --port 8800`.
* **Light video.** Video shares the connection with the controls. Open `http://127.0.0.1:8800/?name=YourName&fps=10&wrist=0`
  (frame rate 2/10/15/20, wrist camera off), or change both with the switches under the overhead picture. The default is 20 fps.
* **Hold-to-move keys** (the controller's hold-jog). Velocity control was tried and reverted: the controller refuses it when
  the round trip is over about 200 ms.
* **Dashboard connection** (`:8780`, kept alive) rather than the controller port directly: roughly twice as fast.

**Stop.** *Stop motion* (**Esc**) halts the arm where it is, keeps torque on and latches nothing; control is released, so click
Take control to carry on remotely. *EMERGENCY STOP* is the controller's latching emergency stop: use it only if someone can
get to the arm.

**Watch what the arm really did** with the read-only monitor: `python robotic_arm/scripts/monitor_state.py --log /tmp/arm.log`.

**Known catch:** a wrist roll near its lower stop (about -114 degrees) blocks keyboard moves ("outside its EEPROM limit"); use the
roll nudge buttons to back off.

