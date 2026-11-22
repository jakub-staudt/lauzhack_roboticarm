"""Record (picture, joint angles) pairs while the arm is moved around by hand over the board. READ-ONLY.

Sends only GET requests (the arm's state and the overhead camera picture). It saves a pair whenever the arm has come to rest in
a pose that differs enough from the last saved one, so just jog the arm over the whole board: different spots, heights, tilts
and rolls make a good dataset (30 to 60 poses). Stop it with Ctrl+C.

    python tools/record_pose_samples.py --out data/poses
"""

from __future__ import annotations

import argparse
import json
import time
import urllib.request
from pathlib import Path


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default="data/poses")
    ap.add_argument("--dashboard", default="http://100.85.56.14:8780")
    ap.add_argument("--min-change", type=float, default=6.0, help="total joint change [deg] from the last saved pose")
    ap.add_argument("--still", type=float, default=0.4, help="joints must move less than this [deg] between two polls to count as at rest")
    args = ap.parse_args()
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    count = len(list(out.glob("pose_*.json")))
    last_saved, previous = None, None

    def state() -> dict:
        return json.load(urllib.request.urlopen(f"{args.dashboard}/api/state", timeout=8))["measured"]

    print(f"recording into {out} (read-only). Move the arm over the board; Ctrl+C to stop.")
    while True:
        try:
            joints = state()
            still = previous is not None and max(abs(joints[k] - previous[k]) for k in joints) < args.still
            far = last_saved is None or sum(abs(joints[k] - last_saved[k]) for k in joints) >= args.min_change
            if still and far:
                picture = urllib.request.urlopen(f"{args.dashboard}/cameras/overhead.jpg", timeout=10).read()
                after = state()                                      # the arm must not have moved while the picture was taken
                if max(abs(after[k] - joints[k]) for k in joints) < args.still:
                    (out / f"pose_{count:03d}.jpg").write_bytes(picture)
                    (out / f"pose_{count:03d}.json").write_text(json.dumps({"joints": after, "time": time.time()}, indent=1))
                    print(f"saved pose_{count:03d}  ({count + 1} so far)")
                    count, last_saved = count + 1, after
            previous = joints
        except KeyboardInterrupt:
            print(f"stopped; {count} poses in {out}")
            return
        except Exception as exc:  # noqa: BLE001 - an outage is not the end of the recording
            print(f"no answer ({type(exc).__name__}); retrying")
        time.sleep(0.5)


if __name__ == "__main__":
    main()
