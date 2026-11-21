# Controller import

Imported on 2026-11-03 from `/home/kwiscion/apps/ramion-workbench/src` on `ramion-server-01`, after passwordless sudo became available to `kwikiel`.

All 137 included source files were compared byte-for-byte by SHA-256 with the server. See `source-snapshot.json`. The source directory has no `.git` history; this is a filesystem import rather than a history migration. The adjacent server deployment metadata reports source commit `24abbb6f7f16a2fc8ab8a868f99a79d12e1e5c30`; that is recorded provenance, not independently reconstructed ancestry.

## Contents

- `packages/soarm`: hardware backends, serial bridge, built-in browser panel, two-camera streaming, measured-pose viewer, and robot model/assets.
- `packages/soarm_lab`: simulation, policies, training/evaluation, and twin integrations.
- Tests, docs, existing `pyproject.toml` and `uv.lock`, Linux service units and launch scripts.
- Original LICENSE, NOTICE and third-party notices.

The original source files are unchanged. `IMPORT.md`, `source-snapshot.json`, and `deployment-snapshot/DEPLOYMENT.json` are import metadata. Exclusions: `.venv`, Python/test/linter caches, and `uv.lock.before-linux`. Existing SSH keys, runtime environment files and controller state outside the source directory are not copied.

`deployment-snapshot/DEPLOYMENT.json` is a historical server record. Its test counts describe the earlier deployment, not checks performed during this import.

## Monorepo paths

The original README's upstream clone instructions describe `machinekind/so-arm101`. For this checkout, use:

```bash
git clone https://github.com/ramion-cloud/monorepo.git
cd monorepo/apps/arm-workbench
uv sync --locked
```

The Linux unit files still target the current deployed directory under `/home/kwiscion/apps/ramion-workbench/src`. They are not automatically changed to point at a monorepo checkout. Deploying from a future monorepo checkout requires deliberately updating service working directories and executable paths. This import did not redeploy, restart, enable or otherwise change either hardware service.

The pinned Machinekind twin API is fetched from GitHub by uv. The twin application, GPU training stacks and other optional dependencies remain optional as declared in the original workspace.

## Verification

Run the hardware-free suite and static checks from this directory:

```bash
uv run --locked pytest -q -m 'not twin and not render'
uv run --locked ruff check .
uv run --locked pyright
```

Twin/render integrations, GPU training and physical hardware acceptance are outside these checks. No physical movement commands were issued for this import.

### Results of this import

On the local macOS Python 3.12 environment, `uv sync --locked` succeeded. Ruff passed and Pyright reported zero errors or warnings. The hardware-free suite produced **406 passed, 1 failed, 17 skipped, 18 deselected**. The failure is the existing bundled lift-policy simulation assertion in `test_shipped_lift_lifts_ends_and_keeps_the_arm_calm`: 16 joint-limit ticks after grasp, versus the expected maximum of 12. This source was imported unchanged; the failure is not repaired or hidden. Runtime numerical warnings also appeared in policy matrix multiplication. Optional twin/render integrations were excluded explicitly.

## Consolidation on 2026-11-04

Notebook development from /home/kwiscion/src/ramion-workbench, observer camera support and a shared latched software stop are now maintained here. The original source-snapshot and deployment-snapshot files above remain historical import records; they do not describe the evolved source or a new deployment. Releases identify their Git commit in .release-commit and retain archive checksums. See the repository deployment guide.
