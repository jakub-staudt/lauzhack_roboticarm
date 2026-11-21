#!/usr/bin/env bash
# Package both apps from HEAD and build/verify each release as the server would (deploy.sh verify).
# Usage: bash scripts/ci/build.sh [OUTPUT_DIR]   (default: dist/; the commit must be clean)
set -euo pipefail
# shellcheck source=scripts/ci/require-uv.sh
source "$(dirname -- "${BASH_SOURCE[0]}")/require-uv.sh"
root="$(git -C "$(dirname -- "${BASH_SOURCE[0]}")" rev-parse --show-toplevel)"
out="${1:-$root/dist}"
export UV_PYTHON="${UV_PYTHON:-3.12}"

printf '\n==> package\n'
bash "$root/scripts/deploy.sh" package "$out"
revision="$(git -C "$root" rev-parse HEAD)"
for app in arm-dashboard arm-workbench; do
    printf '\n==> verify %s\n' "$app"
    bash "$root/scripts/deploy.sh" verify "$app" "$out/ramion-$app-$revision.tar.gz"
done
