#!/usr/bin/env bash
# Workbench checks, as CI runs them. The twin and render suites are deselected (nightly).
# Usage: bash scripts/ci/workbench.sh [sync|lint|types|test ...]   (no argument: all, in that order)
set -euo pipefail
# shellcheck source=scripts/ci/require-uv.sh
source "$(dirname -- "${BASH_SOURCE[0]}")/require-uv.sh"
# Production builds with Python 3.12 (docs/deployment.md); override with UV_PYTHON.
export UV_PYTHON="${UV_PYTHON:-3.12}"
cd -- "$(git -C "$(dirname -- "${BASH_SOURCE[0]}")" rev-parse --show-toplevel)/apps/arm-workbench"

# The workbench group, as production builds it: cameras, kinematics, viser. Without it ~25 tests fail.
run=(uv run --locked --group workbench)
step() { printf '\n==> %s\n' "$*"; }

stage_sync() { step sync; uv sync --locked --group workbench; }
stage_lint() { step 'ruff check'; "${run[@]}" ruff check .; }
stage_types() { step pyright; "${run[@]}" pyright; }
stage_test() { step 'pytest (not twin, not render)'; "${run[@]}" pytest -q -rs -m 'not twin and not render'; }

for stage in "${@:-sync lint types test}"; do
    for s in $stage; do
        case "$s" in sync | lint | types | test) "stage_$s" ;; *) echo "Unknown stage: $s" >&2; exit 2 ;; esac
    done
done
