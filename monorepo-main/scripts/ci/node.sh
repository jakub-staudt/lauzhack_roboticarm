#!/usr/bin/env bash
# Node UI tests (no dependencies). Usage: bash scripts/ci/node.sh
set -euo pipefail
cd -- "$(git -C "$(dirname -- "${BASH_SOURCE[0]}")" rev-parse --show-toplevel)"
node --test tests/*.cjs
