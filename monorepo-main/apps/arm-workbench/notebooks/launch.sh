#!/usr/bin/env bash
set -euo pipefail
repo_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
python_bin="$repo_dir/.venv-notebooks/bin/python"
if [[ ! -x "$python_bin" ]]; then
  echo 'Run bash notebooks/setup.sh core first.' >&2
  exit 1
fi
exec "$python_bin" -m jupyterlab \
  --no-browser --ip=127.0.0.1 --port="${RAMION_JUPYTER_PORT:-8888}" \
  --ServerApp.root_dir="$repo_dir/notebooks" "$@"
