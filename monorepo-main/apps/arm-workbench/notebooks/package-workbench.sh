#!/usr/bin/env bash
# Prepare a source bundle for the supervised deployment handoff; never change services.
set -euo pipefail
repo_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
bundle_dir="$repo_dir/workspace/notebooks"
mkdir -p "$bundle_dir"
tar -C "$repo_dir" -czf "$bundle_dir/ramion-workbench-notebooks.tgz" \
  --exclude='__pycache__' --exclude='*.pyc' --exclude='.ipynb_checkpoints' \
  pyproject.toml uv.lock conftest.py README.md LICENSE NOTICE packages docs ops notebooks
sha256sum "$bundle_dir/ramion-workbench-notebooks.tgz" > "$bundle_dir/ramion-workbench-notebooks.tgz.sha256"
echo "Prepared $bundle_dir/ramion-workbench-notebooks.tgz"
echo 'Use notebooks/README.md for the supervised handoff. No deployed files or services changed.'
