# shellcheck shell=bash
# Sourced by the CI scripts: fail early with a clear message if uv is missing or too old.
# The locks (revision 5) need uv 0.13.6+; older uv reports them out of date.
min_uv=0.13.6
uv_version="$(uv --version 2>/dev/null | awk '{print $2}')" || true
if [[ -z "$uv_version" ]]; then
    printf 'uv not found on PATH (a uv inside one pyenv version does not count).\n' >&2
    printf 'Install it: curl -LsSf https://astral.sh/uv/install.sh | sh\n' >&2
    exit 127
fi
if [[ "$(printf '%s\n' "$min_uv" "$uv_version" | sort -V | head -1)" != "$min_uv" ]]; then
    printf 'uv %s is too old; these locks need %s or newer. Run: uv self update\n' "$uv_version" "$min_uv" >&2
    exit 1
fi
