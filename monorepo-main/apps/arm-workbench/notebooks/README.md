# Robot model notebooks

Run SmolVLA on this server's RTX 3060, use your own dedicated Codex login as a visual
planner, or run Dream-1 through the existing single-arm adapter. All robot state,
camera images and motion go through the workbench HTTP API. Notebook kernels never
open a serial port or a camera device.

The demonstration task is to move the stuffed toy mouse onto the **left gray
area**, **right gray area**, or **between the gray areas**, as seen in the overhead
camera. These are image-relative names. Start with the mouse at the taught pickup
position for the Codex workflow.

| Notebook | Kernel | What it does |
| --- | --- | --- |
| `00_setup.ipynb` | Ramion Â· setup / Codex | Check server state, both camera views, GPU and local tool availability. |
| `01_local_vla.ipynb` | Ramion Â· SmolVLA GPU | Load SmolVLA and benchmark two-camera inference locally. |
| `02_codex.ipynb` | Ramion Â· setup / Codex | Teach named poses; ask Codex to select constrained manipulation skills; preview and explicitly execute. |
| `03_dream.ipynb` | Ramion Â· Dream-1 | Preview Dream's action chunks and optionally run a bounded episode. |

## Install on the server

Use your own Linux account and checkout. Setup creates `.venv-notebooks`, `.venv-vla` and
`.venv-dream` in the development checkout; it does not modify the running app's
environment, restart services, claim robot control or move the arm.

```bash
cd "$HOME/src/ramion-monorepo/apps/arm-workbench"
bash notebooks/setup.sh all
```

Install fewer kernels with `core`, `vla` or `dream` instead of `all`. Both model
options also install the core Jupyter server. The model environments are separate
because Dream and LeRobot have different dependency constraints. Setup requires
network access for packages; model weights are downloaded only when the local
VLA notebook explicitly loads its checkpoint.

Setup uses the checked-in `requirements-*-lock.txt` files containing the exact
versions tested on this server with Python 3.12 and Linux x86_64. These include the
pinned twin API Git revision; the two local workspace packages are installed
editable afterward. The shorter `requirements-*.txt` files document the direct
dependency choices and serve as a fallback only if a lock file is absent. The
notebook environments do not reuse or resolve into the running app's environment.

## Open with JupyterLab over SSH

From your laptop, open an SSH tunnel with your own server username and keep it running:

```bash
ssh -L 8888:127.0.0.1:8888 YOUR_USER@100.85.56.14
```

In that server shell:

```bash
cd "$HOME/src/ramion-monorepo/apps/arm-workbench"
bash notebooks/launch.sh
```

Open the `http://127.0.0.1:8888/lab?token=...` URL printed by Jupyter on your
laptop. Jupyter binds to server loopback and retains its token authentication.
Use `RAMION_JUPYTER_PORT=8889 bash notebooks/launch.sh` and a matching SSH tunnel
if port 8888 is busy.

## Open with VS Code Remote SSH

Connect to `YOUR_USER@100.85.56.14` with Remote SSH, open
`~/src/ramion-monorepo/apps/arm-workbench`, and install the Python and Jupyter extensions
in the remote session. Open a notebook and select its kernel from the table above.
If VS Code does not list the registered name, select the corresponding interpreter:

| Workflow | Server interpreter |
| --- | --- |
| Setup / Codex | `~/src/ramion-monorepo/apps/arm-workbench/.venv-notebooks/bin/python` |
| Local VLA | `~/src/ramion-monorepo/apps/arm-workbench/.venv-vla/bin/python` |
| Dream | `~/src/ramion-monorepo/apps/arm-workbench/.venv-dream/bin/python` |

## First session

1. Run `00_setup.ipynb`; verify both views and current measured joints.
2. Open one model notebook. Each starts with observation or an action preview.
3. For physical execution, have an operator beside the arm, release browser
   control, inspect the intended path, and enter the notebook's exact arming
   phrase. Physical motion tests are reserved for that supervised session.
4. Keep the workbench open for its STOP control. Interrupting an executing cell
   attempts cleanup; the workbench also enforces its own command timeout and
   episode deadline. STOP holds the current measured pose and keeps torque on.

The model API extension must be present in the **running** workbench before any
notebook can arm motion. Editing this development checkout does not update the
snapshot under `/home/kwiscion/apps/ramion-workbench/src`. Use the existing
[deployment procedure](../ops/linux/README.md) during a supervised handoff;
notebooks do not restart or replace the active hardware service. Camera viewing
and model inference can use the current read-only endpoints before that handoff.

## Deploying controller changes

Use the [team release workflow](../../../docs/TEAM_HANDOFF.md#8-deploy-a-reviewed-release)
from a clean committed monorepo checkout. Existing service owners deploy the
checksummed app archives through `scripts/deploy.sh`; deployment returns to preview.

The retained `notebooks/package-workbench.sh` is a legacy source-bundle helper.
Do not use it to overwrite the current release in place: that bypasses the release
and rollback procedure. Notebook-only edits take effect in their personal checkout
after reopening the file/restarting the kernel; they do not require a service deployment.

## Model expectations

**SmolVLA:** `lerobot/smolvla_base` is a pretrained base policy, and this setup has
no task dataset or fine-tuned checkpoint yet. The default notebook uses real
camera frames, a zero-state benchmark input, and a masked third camera slot. It
reports raw action values and latency, and does not turn those values into robot
commands. Task execution needs a suitable fitted checkpoint, its camera mapping,
and an explicit conversion between the checkpoint's joint units and this robot's
degree/percent units. It is not a trained mouse-moving demo.

**Codex:** The CLI uses a dedicated file-based profile selected by the operator.
Create a dedicated profile from your own Linux account in an SSH terminal:

```bash
install -d -m 700 "$HOME/.local/state/ramion/codex"
CODEX_HOME="$HOME/.local/state/ramion/codex" codex -c 'cli_auth_credentials_store="file"' login --device-auth
chmod 600 "$HOME/.local/state/ramion/codex/auth.json"
```

Enable device-code sign-in in your ChatGPT security settings if required. Complete
the browser sign-in with your own Codex account. See the
[OpenAI device login instructions](https://learn.chatgpt.com/docs/auth#login-on-headless-devices). Use
the profile path `~/.local/state/ramion/codex` in the notebook's `CODEX_PROFILE`.

Run Jupyter under your own Linux login. Operators sharing one Linux account can
read that account's credential files, so a shared account cannot isolate their
personal credentials. The CLI receives camera
images and a catalog of taught skills; it does not get robot credentials or
unrestricted low-level control. Start with eight taught poses: four for pickup and
four for one chosen destination. Sixteen poses cover pickup and all three destinations.
Teach collision-clear approach, grasp, lift, lower, release and retreat positions
with the workbench. At grasp/release, only
the gripper changes; during transport, the gripper stays fixed. The planner selects
from those skills.
Changing the mouse's starting position, the gray areas, or the camera arrangement
requires checking and, where necessary, reteaching the poses.

**Dream-1:** The notebook deliberately reuses the repository's experimental
single-arm adaptation: the physical SO101 is the right arm, a dummy left-arm
state is supplied, and the missing left-wrist view is filled from the scene view.
This differs from Dream's documented bimanual setup and does not guarantee task
success. Enter your Dream API key in the hidden prompt when starting an explicitly
enabled preview or run. The SDK receives that key for the current inference session.
The pinned client supports disabling additional
recording; by default the adapter verifies that it is off before sending observations.
Set `ALLOW_ADDITIONAL_RECORDING = True` only to explicitly opt into the SDK recorder.
Inference still sends the camera images and robot state to Dream Labs.
The adapter rejects action chunks more than three seconds old relative to their
originating observation. Cloud latency can therefore stop a preview or episode;
do not treat a timeout as a reason to bypass the age check.

Cloud previews and episodes have explicit request/time limits. A request limit is
not a monetary billing cap. Start with the short preview before requesting a
longer session. Notebook outputs should be cleared before sharing, especially
when they contain camera images or private task descriptions; never paste API
keys or Codex authentication files into cells.

## Historical verification before bring-your-own credentials

These model calls used the earlier credential setup. The new explicit credential paths have offline regression coverage; no live model calls were made for that change.

| Check | Observed result |
| --- | --- |
| Notebook defaults | All four notebooks executed successfully with cloud/model-loading/motion gates off. Both workbench cameras were read. Files retain cleared outputs. |
| Local SmolVLA | Real GPU inference returned a `50 Ă— 6` action chunk: 0.728 seconds cold, 0.370 seconds warm, about 1,215 MiB peak PyTorch allocation. Outputs remain non-executable base-policy units. |
| Codex vision | A bounded call completed in 5.76 seconds, recognized the mouse near the gripper, and returned `needs_teaching` with no skills because no poses had been taught. |
| Dream Flash | Inference verified: the selected `dreamlabs/dream-1-beta-flash` returned a `48 Ă— 6` chunk in 1.851 seconds (source-observation age 1.891 seconds), accepted for preview with additional recording off. An initial `NO_CAPACITY / pool_warming` response recovered after an explicit retry. A subsequent 3.489-second reply (source age 3.569 seconds) was rejected by the unchanged 3-second age limit. No targets were sent to the arm. |
| Dream beta comparison | Replies at 3.210 and 3.063 seconds were rejected by the unchanged 3-second source-observation age limit. They were not sent to the arm. |
| Software checks | 236 hardware-free workbench/helper/model tests passed; the touched Python passed Ruff, and the panel passed Pyright. All three notebook environments passed dependency checks. |
| Jupyter | Loopback launch succeeded and all three named kernels were discovered. The smoke-test server was stopped afterward. |

Dream Flash remains the selected default. Latency varies: a warm kernel did not
guarantee a fresh reply, so inspect each preview and retain the 3-second age limit.
If capacity is unavailable again, wait the provider's reported retry interval and
explicitly rerun the bounded preview. Its sanitized `last_error` metadata includes
provider code/reason and retry interval. It does not retry forever or silently
switch models.

A provider retry delay can exceed the preview window (25 seconds) or the complete
inference budget (30 seconds). In that case the preview ends with the provider's
error and retry guidance; it does not extend the budget or raise an uninformative
timeout. After updating the notebook helpers, restart the Dream kernel and reload
the notebook from disk before rerunning the preview.

No physical robot motion or deployed-service update was performed in this work.
Moving the mouse, taught-pose clearance and STOP during actual motion still require
the supervised handoff and acceptance session.

## Troubleshooting

| Symptom | Action |
| --- | --- |
| Model API unavailable | Complete the supervised deployment handoff; read-only observation still works. |
| Another operator owns control | Release control in the existing browser/notebook; do not bypass the lease. |
| Stale camera or feedback | Inspect the workbench; repair the connection before retrying. |
| A kernel cannot import its model | Select its matching interpreter and rerun `setup.sh vla` or `setup.sh dream`. |
| CUDA unavailable or out of memory | Inspect GPU usage with `nvidia-smi`; close the other model kernel before loading SmolVLA. |
| Codex authentication failure | Run `CODEX_HOME="$HOME/.local/state/ramion/codex" codex -c 'cli_auth_credentials_store="file"' login status` from your own SSH account. |
| Dream authentication/model error | Re-enter your key in the hidden prompt and check checkpoint access. |
| Dream `NO_CAPACITY` / `pool_warming` | Wait the reported retry interval, then explicitly rerun the bounded preview. No usable action exists until serving capacity is available. |

References: [SmolVLA](https://huggingface.co/docs/lerobot/smolvla),
[Codex non-interactive mode](https://learn.chatgpt.com/docs/non-interactive-mode),
[Dream client](https://github.com/dreamlabs-co/dreamlabs/blob/main/dream_library/dream_client/README.md).

## Slower Dream configuration

The Dream notebook now selects `dreamlabs/dream-1-beta`, with a 120-second preview window, at most three requests, and a 15-second model-output age limit. Provider backoff is respected. A no-motion test returned a valid chunk in 5.64 seconds. Historical Flash results above describe the previous configuration. Local camera/robot freshness and the workbench watchdogs remain unchanged; the live episode remains five seconds. Reload the notebook from disk and restart its kernel to use these settings. Execution was reset to off so a new preview can be inspected before rearming.

Dream execution now exposes `RUN_SECONDS = 60` and `MAX_RUN_REQUESTS = 45`, including warm-up requests. The 120-second startup preview, 15-second model-output age limit and speed of 8 units/s remain unchanged. Live status reports chunks and stop reasons. The episode does not detect successful placement automatically: observe the cameras and press workbench STOP when the task is done. A time-limit stop is not task completion.

## Dream command-feedback correction

Following an inward-folding run stopped by the wrist tracking watchdog, the notebook adapter now anchors chunk planning to the workbench command actually sent to the servos, rather than accumulating its own faster internal command. No-motion previews anchor to measured joints. Its default speed cap is 8 units/s to match the notebook runner. Offline regressions verify both anchors. This corrects a feedback mismatch, but does not validate single-arm Dream predictions or prove the cause of the observed collision. Execution remains disabled pending trajectory and physical mapping validation. Restart the Dream kernel and reload the notebook to load this correction.

## Shared latched stop

Use the persistent STOP ROBOT button in the dashboard while a notebook cell is running. It revokes the session at the controller even while model inference is blocked. Reset never resumes that run; restart the cell deliberately. Notebook kernels must be restarted after updating the shared client. See [control architecture](../../../docs/control-and-notebooks.md).
