#!/usr/bin/env bash
# Dashboard checks, as CI runs them.
# Usage: bash scripts/ci/dashboard.sh [sync|lint|types|test ...]   (no argument: all, in that order)
set -euo pipefail
# shellcheck source=scripts/ci/require-uv.sh
source "$(dirname -- "${BASH_SOURCE[0]}")/require-uv.sh"
cd -- "$(git -C "$(dirname -- "${BASH_SOURCE[0]}")" rev-parse --show-toplevel)/apps/arm-dashboard"

# The scan extra: the collector tests and pyright need OpenCV.
run=(uv run --locked --extra scan)
step() { printf '\n==> %s\n' "$*"; }

stage_sync() { step sync; uv sync --locked --extra scan; }
stage_lint() {
    step 'ruff check'; "${run[@]}" ruff check .
    step 'ruff format --check'; "${run[@]}" ruff format --check .
}
stage_types() { step pyright; "${run[@]}" pyright; }
stage_test() {
    step 'pytest (unit + contract, legacy and fastapi targets)'; "${run[@]}" pytest -q -rs
    step 'legacy unittest (rollback target, kept until legacy/ is retired)'
    (cd legacy && "${run[@]}" python -m unittest discover -s tests)
}

for stage in "${@:-sync lint types test}"; do
    for s in $stage; do
        case "$s" in sync | lint | types | test) "stage_$s" ;; *) echo "Unknown stage: $s" >&2; exit 2 ;; esac
    done
done
