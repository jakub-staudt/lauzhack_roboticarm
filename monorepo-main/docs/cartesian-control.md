# Cartesian manual control

The shared workbench panel provides a Cartesian/Joint switch in Manual mode.
Open the operator UI on the dashboard on port 8780. The workbench on port 8765 serves the controller API.
Take control and select Cartesian to use the movement, orientation and gripper
hold-to-move buttons.

| Button | Keyboard | Movement |
| --- | --- | --- |
| Forward / Back | W / S | Move the gripper tip away from / toward the base in the arm's current radial plane. |
| Left / Right | A / D | Rotate shoulder pan; this follows an arc rather than a straight sideways path. |
| Up / Down | R / F | Move the gripper tip vertically. |
| Tilt up / Tilt down | I / K | Move wrist flex directly; the tip follows an arc around the wrist. |
| Roll left / Roll right | J / L | Turn wrist roll around its axis; left is counterclockwise looking from the wrist toward the tip. |
| Open / Close | Q / E | Gradually open / close the gripper. |
| STOP ROBOT | Escape | Latch the existing software stop and revoke control. |

Enable keyboard control explicitly. Hold one direction at a time and release it
to stop. Shortcuts do not operate while editing a form field. Leaving the page,
losing window focus, switching control modes, or losing ownership cancels jogging.
Keyboard auto-repeat does not create additional movement requests. Checking
Enable keyboard movement takes effect immediately, including while that checkbox
still has focus. Delayed replies from an old ownership session cannot revoke a
newly claimed session.

Normal forward/back movement holds tip height; up/down holds reach. Shoulder
lift, elbow flex and wrist flex move together to preserve the gripper tilt at
the start of the hold. Pan and wrist roll remain unchanged during translation.
The path is anchored to that starting pose instead of accumulating geometric
drift from each rounded servo command. New holds continue from the last accepted
joint setpoints, rather than replacing them with measured feedback. A small
steady servo tracking error therefore cannot become cumulative motion on each
press or release, and gripper-only holds preserve every arm-joint setpoint.
Releasing a hold or its movement watchdog expiring stops trajectory generation
at the last accepted setpoint; servos may finish settling to that setpoint.
Faults, lost control and STOP retain the separate measured-pose hold behavior
and the 5-degree arm tracking-error stop. Joint control remains available for
deliberate wrist adjustments.

After approaching a grasp, use I/K to tilt the wrist and J/L to align the jaws.
Tilt moves only wrist flex; roll moves only wrist roll. Neither compensates at
the tip, so the tip can follow an arc. Other joints and jaw opening remain
unchanged. Both orientation controls use the selected profile's angular
rate (5/10/20 degrees per second), subject to joint-rate and workspace limits.
They share the same one-direction hold, release, focus-loss and STOP handling as
translation. Recovery applies only to translation.

If the measured tip starts just below the configured height margin, direct tilt
may raise it back into the allowed area. Such motion must not lower the requested
tip position further; worsening measured feedback stops it. Normal floor checks
resume once the measured and commanded tip are above the floor. The configured
tabletop and margin are unchanged.

Motion errors remain visible until dismissed or superseded by a deliberate
action. Routine heartbeats do not clear them. The message panel overlays the
page below the header, so showing an error does not move the controls or cameras.
When a hold ends, the UI reports the controller's specific stop reason when
available.

## Recovery

Enable Recovery explicitly when a normal straight path is blocked. It uses the
selected speed profile and permits at most **3 degrees of total tilt change**
and **2 mm of total tip deviation** from the hold's reference path. These are
whole-hold budgets, not allowances that reset every tick. Changing Recovery
mode ends the active hold; press a direction again deliberately.

A measured joint can sit slightly beyond a software limit. For an overshoot of
at most 1 degree, Recovery can first ease it back inside while compensating with
the other pitch joints. The complete ingress must be feasible within the tip
and tilt budgets and the known EEPROM limits. Unknown hard limits, excessive
overshoot, another worsening violation or an infeasible ingress reject recovery.
No calibration, EEPROM limit or global software limit is widened.

## Estimated travel

The Reach and Height bars show sampled straight-line distances from the measured
pose at its current tilt. Each direction uses the same normal-mode solver and
effective joint limits, with a 400 mm search horizon and 0.5 mm boundary search
resolution. A value marked with a lower bound means the search horizon was
reached. Resolution is not a claim of physical positioning accuracy.

The display identifies a limiting joint only when the model supports that
attribution. Otherwise it reports a local solver/position-orientation boundary.
The floor is identified separately. These are local sampled estimates, not a
global reachable volume or collision-free clearance, and they never authorize
motion or disable a direction on their own.

Calculations use a separate solver worker. While the pose changes, a completed
prior estimate can remain dimmed for at most three seconds with its age and an
explicit previous-pose label. Stale robot feedback hides the values. Estimates
continue to describe normal fixed-tilt motion even when Recovery is selected.

## Speed and transport

Cartesian jogging starts at Slow. Speed profiles bound tip translation, shoulder
pan, joint motion and gripper opening separately. The controller also respects
the robot's software and calibrated joint limits.

| Profile | Tip translation | Shoulder pan | Arm joint cap | Gripper |
| --- | --- | --- | --- | --- |
| Slow | 10 mm/s | 5°/s | 8°/s | 10 units/s |
| Medium | 25 mm/s | 10°/s | 15°/s | 20 units/s |
| Fast | 50 mm/s | 20°/s | 25°/s | 35 units/s |

The gripper uses the existing calibrated 0–100 opening scale. Local reach or
joint-speed limits can stop a translation before the nominal tip speed is reached.
Fractional path progress is retained across servo rounding, and each hold starts
with a short acceleration ramp. The control loop remains at 20 Hz; physical
smoothness still needs supervised verification on the actual servos.

Jogging uses a short movement lease independent of the browser's control lease.
The browser renews only the currently held movement. Stopped or expired movements
cannot be revived by late renewals, and a release cancels a start that is still
in flight. Reconnection requires a new deliberate press.

High-latency connections can interrupt hold-to-move control. Do not extend the
movement watchdog to mask network delays. Prefer the direct private-network
route for responsive control.

## Camera preview

In Session, select a per-browser Video preview rate: **Low bandwidth (2 fps)**,
**Smooth (10 fps)** or **High (15 fps)**. Smooth is selected initially at the
server's advertised direct RAMION IP/hostname. Low bandwidth remains the default
for localhost SSH and unrecognized routes. An explicit selection stays in effect
for the current page. Reduce the rate if the connection cannot sustain the video
traffic.

The proxy fetches the latest cached JPEG after the prior frame has been written;
it does not queue scheduled frames. Its send buffer is bounded. Hidden camera
views pause their browser streams and resume when shown again. Preview settings
do not change camera capture, assignment, or model-observation images.
The UI distinguishes the preview target rate from camera capture rate. Neither
is a measured sensor-to-screen latency.

## Workspace configuration

Spatial jogging requires an explicit tabletop height in robot-base coordinates:

```text
RAMION_CARTESIAN_TABLE_Z_M=<tabletop Z in metres relative to the robot model>
RAMION_CARTESIAN_TCP_MARGIN_M=0.02
```

Set these in the host's existing `~/.config/ramion/workbench.env`. An absent or
invalid table setting disables spatial jogging. The margin is the minimum
gripper-tip height above the configured plane.

For the standard interlocked SO-ARM100 overhead camera mount with no added
spacers, the mesh-derived tabletop height is `-0.0074` m in the existing SO-101
model frame. The arm sits 5 mm above the table on the recessed seat; the model's
own base bottom is at Z = -2.4 mm. This geometric registration does not replace
physical calibration.

Enable the matching visual assembly with:

```text
RAMION_WORKSPACE_LAYOUT=so101_overhead_32x32_right
RAMION_CARTESIAN_TABLE_Z_M=-0.0074
RAMION_CARTESIAN_TCP_MARGIN_M=0.02
```

Here **right** means the robot's right looking outward along its reach, not the
observer's right when facing it. The viewer includes the arm base and all three
camera-stand sections. The nominal assembly is static and remains visible in
disconnected preview; no robot pose is invented when measurements are absent.
The original robot coordinates, joint markers and TCP are unchanged.

The four STL files come from [TheRobotStudio's overhead mount](https://github.com/TheRobotStudio/SO-ARM100/tree/a758567c3978dfeefe282ede0500085a48fe8f78/Optional/Overhead_Cam_Mount_32x32_UVC_Module).
Their source commit, checksums, transforms and registration evidence are recorded
in `apps/arm-workbench/packages/soarm/soarm/environment_assets/overhead_32x32/provenance.json`.

This is a **tip-height bound**, not collision detection for the entire arm,
camera stand, objects or people. The displayed mount is a visual reference and
does not establish calibrated clearance. Check the real workspace and supervise
movement. Existing STOP remains a software stop with the limitations described
in [control and notebooks](control-and-notebooks.md).

## Verification and rollout

Controller tests use fake hardware and numerical kinematics. Browser tests mock
HTTP requests and exercise release, focus, ownership and delayed-response paths.
These checks do not establish physical movement accuracy or physical STOP
performance.

Deploy through the existing [release procedure](deployment.md), which returns
the service to disconnected preview. Physical acceptance requires an operator
at the robot: verify direction signs, straight translations and held tilt,
partial gripper motion, release-to-hold, keyboard cancellation and STOP. Verify
Recovery separately with workspace clearance, and compare Slow/Medium motion
for smoothness before increasing speed further.
