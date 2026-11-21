#!/usr/bin/env bash
# Install notebook kernels without changing the workbench's running environment.
set -euo pipefail

repo_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
mode="${1:-core}"
case "$mode" in
  core|vla|dream|all) ;;
  *) echo "Usage: bash notebooks/setup.sh [core|vla|dream|all]" >&2; exit 2 ;;
esac
uv_cmd="${UV_BIN:-uv}"
command -v "$uv_cmd" >/dev/null || { echo 'Install uv or set UV_BIN to its executable.' >&2; exit 1; }
api_requirement='machinekind-twin-api @ git+https://github.com/machinekind/twin.git@fb3838f9ab72d6a2bb9cc34d2fdbb113580a4afc#subdirectory=packages/machinekind_twin_api'
core_env="$repo_dir/.venv-notebooks"
export UV_CACHE_DIR="${UV_CACHE_DIR:-$repo_dir/workspace/uv-cache}"

setup_one() {
  local flavor="$1" env_dir kernel_name kernel_label lock_file
  case "$flavor" in
    core) env_dir="$core_env"; kernel_name='ramion-core'; kernel_label='Ramion · setup / Codex' ;;
    vla) env_dir="$repo_dir/.venv-vla"; kernel_name='ramion-vla'; kernel_label='Ramion · SmolVLA GPU' ;;
    dream) env_dir="$repo_dir/.venv-dream"; kernel_name='ramion-dream'; kernel_label='Ramion · Dream-1' ;;
  esac
  if [[ ! -x "$env_dir/bin/python" ]]; then
    "$uv_cmd" venv --python 3.12 "$env_dir"
  fi
  lock_file="$repo_dir/notebooks/requirements-$flavor-lock.txt"
  if [[ -f "$lock_file" ]]; then
    "$uv_cmd" pip install --python "$env_dir/bin/python" \
      -r "$lock_file"
  else
    "$uv_cmd" pip install --python "$env_dir/bin/python" \
      -r "$repo_dir/notebooks/requirements-core.txt" "$api_requirement"
    if [[ "$flavor" != core ]]; then
      "$uv_cmd" pip install --python "$env_dir/bin/python" \
        -r "$repo_dir/notebooks/requirements-$flavor.txt"
    fi
  fi
  "$uv_cmd" pip install --python "$env_dir/bin/python" --no-deps \
    -e "$repo_dir/packages/soarm" -e "$repo_dir/packages/soarm_lab"
  "$uv_cmd" pip check --python "$env_dir/bin/python"
  "$env_dir/bin/python" -m ipykernel install --prefix "$core_env" \
    --name "$kernel_name" --display-name "$kernel_label"
}

setup_one core
case "$mode" in
  vla|dream) setup_one "$mode" ;;
  all) setup_one vla; setup_one dream ;;
esac
echo "Ready. Start Jupyter with: bash $repo_dir/notebooks/launch.sh"
