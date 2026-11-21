# Camera observation and notebook control

The private monorepo is the shared source for dashboard, controller, model notebooks and deployment scripts.

~~~text
Browser dashboard :8780 ----+
Browser workbench :8765 ----+--> workbench controller --> serial adapter --> SO-101
Model notebooks -----------+           |
                                       +--> overhead, wrist, observer cameras
~~~

The controller captures each camera once. Notebooks obtain overhead/wrist observations and
submit bounded model targets through its HTTP API; the observer camera is for people watching
the arm and is not added to existing model inputs.

STOP ROBOT is a latched software stop. It invalidates control ownership and model sessions,
discards pending targets and attempts to hold a fresh measured pose with torque retained.
Further commands are rejected until explicit reset. Reset never resumes an old run:
claim control and deliberately start a new episode. A blocked cloud/model call may finish
computing after STOP, but its revoked session cannot send targets. Restart notebook kernels
after updating the client package.

The latch uses a shared state file outside source releases. Managed arm startup records a
hardware-session marker before opening serial; a restarted process requires preview reset.
Both service modes must use the same state path. Deployment always returns to preview.

A software stop cannot guarantee physical stopping through a failed network, USB link,
driver or computer. A sent hold has no servo acknowledgement. Inspect the status and actual
arm; use the local physical stop/power control when necessary. Cutting torque can let an
unsupported arm fall. This is not a safety-rated emergency-stop circuit.

Only the supported HTTP notebook path participates in this control contract. A user with
serial-device access can bypass it through legacy direct-hardware tools such as dream_run.
Do not run those tools alongside the controller. Enforcing isolation against arbitrary SSH
code requires separate accounts/device permissions and is outside this trusted-team setup.

## Notebooks

Sources and pinned environments: [notebooks](../apps/arm-workbench/notebooks/README.md).

Run setup from the consolidated workspace:

~~~bash
cd /home/kwiscion/src/ramion-monorepo/apps/arm-workbench
bash notebooks/setup.sh all
RAMION_JUPYTER_PORT=8889 bash notebooks/launch.sh
~~~

Port 8889 allows the old Jupyter instance on 8888 to remain available during migration.
Tunnel it from your laptop with:

~~~bash
ssh -L 8889:127.0.0.1:8889 kwiscion@ramion-server-01
~~~

Use the token URL printed by Jupyter. Keep the dashboard open alongside it.
Committed notebooks have cleared outputs and motion gates off. Credentials, model weights,
local notebook experiments, captures and runtime environments remain outside Git.

Dream's earlier inward-folding incident is unresolved. Its command-feedback correction is
included, but physical execution remains disabled pending trajectory/frame validation.
Do not relax tracking or freshness watchdogs to work around that incident.
