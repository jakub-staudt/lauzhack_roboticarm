"""Fit the feature model from a recorded dataset and save it.

Needs the arm's kinematics, so run it with the robotic_arm environment:

    PYTHONPATH=lauzkach_eye_robot_control robotic_arm/.venv/bin/python lauzkach_eye_robot_control/tools/fit_feature_model.py \
        data/poses --reference lauzkach_eye_robot_control/tests/fixtures/overhead_arm_away.jpg --marker dark --out data/feature_model.json

--marker: `colour` (a magenta sticker), `tag:7` (an AprilTag with that id) or `dark` (the black wrist camera board). With a sticker
at the tip use --no-offset.
"""

from __future__ import annotations

import argparse
import json

import cv2
import numpy as np

from lauzkach_eye_robot_control.dataset import build_samples, load_dataset
from lauzkach_eye_robot_control.detect import detect
from lauzkach_eye_robot_control.feature_model import FeatureModel
from lauzkach_eye_robot_control.robot_map import RobotMap
from lauzkach_eye_robot_control.tip import ColorMarkerObserver, DarkBlobObserver, TagMarkerObserver


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("dataset")
    ap.add_argument("--reference", required=True, help="a picture with the rectangle fully visible (the camera does not move)")
    ap.add_argument("--robot-points", default="robotic_arm/calibration_poses.json")
    ap.add_argument("--faces", default="up")
    ap.add_argument("--marker", default="dark")
    ap.add_argument("--radius", type=float, default=90, help="how far from the arm model's guess to look [px]")
    ap.add_argument("--no-offset", action="store_true", help="the feature is at the tip (a sticker on it)")
    ap.add_argument("--out", default="feature_model.json")
    args = ap.parse_args()

    from soarm_lab.kinematics import RobotKinematics
    from soarm_lab.robot import load_robot
    kin = RobotKinematics(load_robot())

    def fk(joints):
        pose = kin.tcp(joints)
        return tuple(pose[:3, 3]), pose[:3, :3]

    if args.marker == "dark":
        print("WARNING: the dark-blob feature is experimental; on the real camera it picked a black cable. Check the picked pixels by eye.")
    reference = detect(cv2.imread(args.reference))
    if not reference["grid"] or reference["grid"].get("px_to_cm") is None:
        raise SystemExit("the rectangle was not found in the reference picture")
    px_to_cm, robot_map = reference["grid"]["px_to_cm"], RobotMap.load(args.robot_points)
    if args.marker == "colour":
        observers = [ColorMarkerObserver(radius=args.radius)]
    elif args.marker.startswith("tag:"):
        observers = [TagMarkerObserver(int(args.marker[4:]))]
    else:
        observers = [DarkBlobObserver(radius=args.radius)]

    template = FeatureModel(px_to_cm, args.faces, robot_map)
    samples, skipped = build_samples(load_dataset(args.dataset), fk, template, observers)
    print(f"{len(samples)} usable samples, {len(skipped)} skipped")
    for line in skipped[:10]:
        print("  skipped:", line)
    model = FeatureModel.fit(samples, px_to_cm, args.faces, robot_map, fit_offset=not args.no_offset)
    print("fit:", model.summary())
    json.dump({"px_to_cm": px_to_cm, "faces": args.faces, "offset": model.offset.tolist(), "lift": model.lift.tolist(),
               "bias": model.bias.tolist(), "rms_px": model.rms_px, "samples": model.n_samples}, open(args.out, "w"), indent=1)
    print("saved", args.out)


if __name__ == "__main__":
    main()
