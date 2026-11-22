"""Read-only monitor of the real arm: polls GET /api/state once a second and logs every change.

Sends nothing but GET requests (no claim, no command), so it is safe to leave running while someone else drives the arm.
Use it to see what the arm really did: who held control, torque, emergency stop, the joints, and the controller's events.

    python scripts/monitor_state.py                      # logs to arm_monitor.log in the current folder
    python scripts/monitor_state.py --log /tmp/arm.log --url http://100.85.56.14:8780/api/state
"""

from __future__ import annotations

import argparse
import json
import time
import urllib.request


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--url", default="http://100.85.56.14:8780/api/state")
    ap.add_argument("--log", default="arm_monitor.log")
    ap.add_argument("--every", type=float, default=1.0, help="seconds between polls")
    args = ap.parse_args()
    last, t_last = None, 0.0
    while True:
        try:
            s = json.load(urllib.request.urlopen(args.url, timeout=6))
            m = s["measured"]
            row = (s["control"].get("label"), s["torque"], s["emergency_stop"]["latched"], round(m["gripper"]), *(round(m[j], 1) for j in (
                "shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll")))
            if row != last or time.time() - t_last > 8:
                recent = " | ".join(e["message"] for e in s.get("events", [])[-3:] if e["timestamp"] > time.time() - 12)
                line = (f"{time.strftime('%H:%M:%S')} owner={row[0]} torque={row[1]} ESTOP={row[2]} grip={row[3]} pan={row[4]} lift={row[5]} "
                        f"elbow={row[6]} wflex={row[7]} roll={row[8]} age={s.get('age_s', 0):.2f}" + (f"  events: {recent}" if recent else ""))
                with open(args.log, "a") as f:
                    f.write(line + "\n")
                last, t_last = row, time.time()
        except Exception as exc:  # noqa: BLE001 - an outage is data too: log it and keep going
            with open(args.log, "a") as f:
                f.write(f"{time.strftime('%H:%M:%S')} (no answer: {type(exc).__name__})\n")
        time.sleep(args.every)


if __name__ == "__main__":
    main()
