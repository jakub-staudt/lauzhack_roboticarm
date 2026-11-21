# Whole-board XY mapping

Automatic collection is now at <http://localhost:8780/map>; see [automatic collection](automatic-collection.md). The optional manual reference tool described below is at <http://localhost:8780/map/manual>. This is a read-only mapping tool; it never claims control or sends robot commands.

The requested output is a flat board map with XY coordinates. The current implementation maps the **entire visible image plane**, not just the reference rectangle. Occluded and out-of-frame areas remain unknown.

## Use the map

1. Keep the overhead camera fixed and capture a snapshot.
2. Identify a flat rectangle of known size on the board. Use measured board corners if all four are visible, or place a measured reference on the board. A large reference spanning the workspace is preferable to a small one. The A4 shortcut only enters 210 × 297 mm; it does not imply a sheet exists or detect its corners.
3. Select top-left, top-right, bottom-right, bottom-left in perimeter order. Enter the rectangle's physical width and height in millimetres.
4. Build the map. The whole visible plane is rectified and a grid is drawn. Move the pointer over it to read XY. The reference's first corner is (0,0); X follows its width and Y follows its height.
5. Download the map PNG and export the calibration JSON. Capture images remain in browser memory unless explicitly downloaded; the application does not publish or persist camera imagery.

No dimensions were assumed from camera images. The live board has not yet been metrically calibrated because reference dimensions/corners have not been supplied.

## Accuracy and coverage

The four-point homography assumes a flat surface and neglects lens distortion. It can fit four selected corners exactly while still being wrong elsewhere. Before describing coordinates as accurate, independently measure distances and check points across the whole board, particularly outside the reference rectangle. Report the errors and required tolerance. A moved camera, changed focus or changed capture resolution invalidates the calibration.

For better metric accuracy, calibrate camera intrinsics and distortion from a physical ChArUco target and rectify the image before fitting the board plane. OpenCV documents [ChArUco calibration](https://docs.opencv.org/4.13.0/da/d13/tutorial_aruco_calibration.html) and [camera geometry, homographies and reconstruction](https://docs.opencv.org/4.13.0/d9/d0c/group__calib3d.html). The current tool has not implemented that intrinsic-calibration step; exports explicitly mark distortion correction, independent accuracy validation and robot-base registration as false.

The arm and objects above the plane are warped visually by the homography; their projected pixels are not board surface geometry. The map is not a collision model and cannot drive the robot.

To capture the **entire physical board**, all of it must be observed. Safely move obstructions out of the view under operator control, or collect additional views with surveyed fiducials. Repositioning the overhead camera requires recalibration. Stitching additional wrist views needs a reliable transform into the same board frame; a moving wrist does not form a fixed stereo rig with the overhead camera.

## Current two-camera feasibility check

Two 640 × 480 cached JPEGs were inspected on 2026-11-03 without moving the robot. The overhead view covers most of the OSB board; the wrist view is a close-up and includes the gripper. A SIFT preflight found 652 overhead features, 814 wrist features, but only **2 matches** passing a 0.7 nearest-neighbour ratio threshold. That is insufficient even to fit a four-correspondence planar transform, and much less to validate a robust stitch. This is one image pair at one pose, not proof that future views cannot be aligned. No unverified automatic mosaic is presented as a scan.

## Tests

From the repository root:

```bash
node --test tests/test_board_map.cjs tests/test_dashboard_ui.cjs
python3 -m unittest discover -s tests -v
```

Geometry tests use known synthetic projective transforms, validate points both inside and outside the reference, check inverse projection, and reject degenerate/crossed references or a horizon crossing the frame. These validate the math, not real-world calibration accuracy.
