#!/usr/bin/env bash
set -euo pipefail

arm=ramion-workbench-arm.service
dashboard_url=http://100.85.56.14:8780
system=false
if [[ "${1:-}" == --system ]]; then
    system=true
    shift
fi

die() {
    printf '%s\n' "$*" >&2
    exit 1
}

service_ctl() {
    if "$system"; then
        systemctl "$@"
    else
        systemctl --user "$@"
    fi
}

package() {
    local output_dir="${1:-$HOME/apps/ramion-releases/incoming}" root revision app archive temp status
    root="$(git -C "$(dirname -- "${BASH_SOURCE[0]}")" rev-parse --show-toplevel)"
    status="$(git -C "$root" status --porcelain)"
    [[ -z "$status" ]] || die 'Commit changes before packaging a release.'
    revision="$(git -C "$root" rev-parse HEAD)"
    mkdir -p -- "$output_dir"
    output_dir="$(cd -- "$output_dir" && pwd -P)"
    for app in arm-workbench arm-dashboard; do
        archive="ramion-$app-$revision.tar.gz"
        temp="$(mktemp "$output_dir/.archive.XXXXXX")"
        git -C "$root" archive --format=tar.gz --output="$temp" "$revision:apps/$app"
        mv -fT -- "$temp" "$output_dir/$archive"
        (cd -- "$output_dir" && sha256sum -- "$archive" > "$archive.sha256")
        printf '%s\n' "$output_dir/$archive" "$output_dir/$archive.sha256"
    done
}

select_app() {
    app="$1"
    config_dir="$HOME/.config/ramion"
    case "$app" in
        arm-workbench)
            base="$HOME/apps/ramion-workbench"
            live="$base/src"
            service=ramion-workbench-preview.service
            runtime_user=ramion-workbench
            config=workbench.env
            ports='sport = :8765 or sport = :8766'
            ;;
        arm-dashboard)
            base="$HOME/apps/ramion-dashboard"
            live="$HOME/ramion-dashboard"
            service=ramion-dashboard.service
            runtime_user=ramion-dashboard
            config=dashboard.env
            ports='sport = :8780'
            ;;
        *) die 'Select arm-workbench or arm-dashboard.' ;;
    esac
    releases="$base/releases"
    previous="$base/previous"
    if "$system"; then
        [[ "$EUID" -eq 0 ]] || die 'Run system release operations with sudo.'
        base=/srv/ramion
        releases="$base/releases/$app"
        live="$base/current/$app"
        previous="$base/previous/$app"
        config_dir=/etc/ramion
    fi
}

require_idle_arm() {
    local state
    state="$(service_ctl show --property=ActiveState --value "$arm")"
    case "$state" in
        inactive|failed) ;;
        *) die "Arm service is $state. End the supervised session and return to preview before deploying." ;;
    esac
}

require_idle_dashboard() {
    local state phase
    if service_ctl is-active --quiet "$service"; then
        state="$(curl -fsS --max-time 3 "$dashboard_url/api/scan/status")" ||
            die 'Could not verify that dashboard collection is idle.'
        phase="$(python3 -c 'import json, sys; print(json.load(sys.stdin)["phase"])' <<< "$state")" ||
            die 'Could not read dashboard collection status.'
        case "$phase" in
            starting|collecting) die 'Wait for the active camera collection to finish.' ;;
        esac
    fi
}

require_idle_app() {
    if [[ "$app" == arm-workbench ]]; then
        require_idle_arm
    else
        require_idle_dashboard
    fi
}

lock_deployment() {
    if "$system"; then
        install -d -m 0755 -- "$base" "$base/releases" "$releases" "$base/current" "$base/previous"
    else
        mkdir -p -- "$releases"
    fi
    local lock="$base/deploy.lock"
    if "$system"; then
        lock="$releases/deploy.lock"
    fi
    exec 9> "$lock"
    flock -n 9 || die 'Another release operation is running.'
    [[ ! -e "$previous" || -L "$previous" ]] || die 'The rollback pointer must be a symlink.'
}

require_switch_ready() {
    [[ "$(service_ctl show --property=LoadState --value "$service")" == loaded ]] ||
        die "Install $service in the selected systemd manager first."
    [[ -r "$config_dir/$config" ]] || die "Configure the host in $config_dir/$config before deploying."
    require_idle_app
}

replace_link() {
    local target="$1" link="$2" temp
    temp="$(mktemp -d "$base/.link.XXXXXX")" || return 1
    if ln -s -- "$target" "$temp/link" && mv -fT -- "$temp/link" "$link"; then
        rmdir -- "$temp"
    else
        return 1
    fi
}

service_healthy() {
    local attempt pid bind_ip state
    for ((attempt = 0; attempt < 20; attempt++)); do
        pid="$(service_ctl show --property=MainPID --value "$service")" || return 1
        if [[ "$app" == arm-dashboard ]]; then
            # /healthz needs nothing but the dashboard itself. Legacy releases have no /healthz,
            # so a rollback to one is checked on / (which also needs the controller).
            if [[ "$pid" =~ ^[1-9][0-9]*$ ]] && {
                curl -fsS --max-time 1 "$dashboard_url/healthz" > /dev/null 2>&1 ||
                    { ! "$system" && curl -fsS --max-time 1 "$dashboard_url/" > /dev/null 2>&1; }
            }; then
                return 0
            fi
            sleep 1
            continue
        fi
        bind_ip=''
        if [[ "$pid" =~ ^[1-9][0-9]*$ && -r "/proc/$pid/environ" ]]; then
            bind_ip="$(tr '\0' '\n' < "/proc/$pid/environ" | sed -n 's/^RAMION_BIND_IP=//p')" || continue
            bind_ip="${bind_ip:-$(/usr/bin/tailscale ip -4)}"
        fi
        if [[ -n "$bind_ip" ]] && state="$(curl -fsS --max-time 1 "http://$bind_ip:8765/api/state" 2>/dev/null)" &&
            python3 -c 'import json, sys; s = json.load(sys.stdin); sys.exit(not (s.get("preview_only") is True and s.get("connected") is False and s.get("port") is None))' <<< "$state"; then
            return 0
        fi
        sleep 1
    done
    return 1
}

switch_to() {
    local next="$1" current='' legacy_dir='' was_active=false listeners
    require_switch_ready
    if [[ -L "$live" ]]; then
        current="$(realpath -e -- "$live")"
    elif [[ -d "$live" ]] && ! "$system"; then
        legacy_dir="$(mktemp -d "$releases/legacy.XXXXXX")"
        current="$legacy_dir/src"
    elif [[ -e "$live" ]]; then
        die "Expected $live to be a directory or symlink."
    fi
    if [[ "$current" == "$next" ]]; then
        "$system" || die 'This release is already selected.'
        if service_ctl is-active --quiet "$service" && service_healthy; then
            printf '%s is already healthy on release %s\n' "$service" "$next"
            return 0
        fi
    fi
    if service_ctl is-active --quiet "$service"; then
        was_active=true
    fi
    service_ctl stop "$service"
    if "$system" && { ! listeners="$(ss -H -ltn "$ports")" || [[ -n "$listeners" ]]; }; then
        if "$was_active"; then
            service_ctl start "$service" || die "Could not restart $service. Inspect its journal."
        fi
        die "Could not confirm that $app ports are free. Stop the old user service before switching."
    fi
    if [[ -n "$legacy_dir" ]] && ! mv -T -- "$live" "$current"; then
        service_ctl start "$service"
        die 'Could not preserve the original source directory.'
    fi
    if replace_link "$next" "$live" && service_ctl start "$service" && service_healthy; then
        if [[ -z "$current" || "$current" == "$next" ]] || replace_link "$current" "$previous"; then
            printf '%s is healthy on release %s\n' "$service" "$next"
            return 0
        fi
    fi
    service_ctl stop "$service"
    if [[ -n "$current" && "$current" != "$next" ]]; then
        replace_link "$current" "$live" || die "Restore $live to $current manually."
        if ! service_ctl start "$service" || ! service_healthy; then
            die "Restored $current, but $service failed its health check. Inspect its journal."
        fi
        die "Service failed its release check. Restored $current."
    fi
    if "$system" && [[ -L "$live" && "$current" != "$next" ]]; then
        rm -f -- "$live"
    fi
    die "Service failed its release check. The service is stopped. Inspect $next and its journal."
}

verify_archive() {
    local name expected actual
    archive="$(realpath -e -- "$1")"
    name="$(basename -- "$archive")"
    [[ "$name" =~ ^ramion-$app-([0-9a-f]{40})\.tar\.gz$ ]] || die 'Use a matching release created by deploy.sh package.'
    revision="${BASH_REMATCH[1]}"
    read -r expected _ < "$archive.sha256"
    actual="$(sha256sum -- "$archive")"
    [[ "$expected" == "${actual%% *}" ]] || die 'Release archive checksum mismatch.'
}

build_release() (
    local app="$1" archive="$2" stage="$3" python="$4" uv_bin
    local -a python_args=()
    uv_bin="$(command -v uv || true)"
    uv_bin="${uv_bin:-$HOME/.local/bin/uv}"
    [[ -x "$uv_bin" ]] || die 'Install uv before deploying.'
    if [[ -n "$python" ]]; then
        python_args=(--python "$python")
        export UV_LINK_MODE=copy UV_PYTHON_DOWNLOADS=never
    fi
    tar -xzf "$archive" -C "$stage"
    cd -- "$stage"
    if [[ "$app" == arm-workbench ]]; then
        UV_PROJECT_ENVIRONMENT="$stage/.venv" "$uv_bin" sync --locked --no-dev --group workbench "${python_args[@]}"
    elif [[ -f pyproject.toml ]]; then
        UV_PROJECT_ENVIRONMENT="$stage/.venv" "$uv_bin" sync --locked --no-dev --extra scan "${python_args[@]}"
    elif [[ -z "$python" ]]; then
        "$uv_bin" venv --python 3.12 "$stage/.venv"
        "$uv_bin" pip install --python .venv/bin/python -r requirements-scan.txt
    else
        die 'The system dashboard unit requires a FastAPI release.'
    fi
)

verify_release() (
    local app="$1" stage="$2"
    cd -- "$stage"
    if [[ "$app" == arm-workbench ]]; then
        .venv/bin/python -c 'import cv2, mujoco, viser, soarm.panel'
        .venv/bin/soarm-panel --help > /dev/null
    elif [[ -f pyproject.toml ]]; then
        .venv/bin/python -c 'import cv2, arm_dashboard.main, arm_dashboard.scan.worker.collector'
        .venv/bin/python -c 'import sys
from arm_dashboard.api.constants import LocalPages
from arm_dashboard.core.settings import Settings
static_dir = Settings(_env_file=None).server.static_dir
sys.exit(not all((static_dir / name).is_file() for name, _ in LocalPages.BY_PATH.values()))' ||
            die 'Dashboard operator or scan assets are missing.'
        [[ -x .venv/bin/arm-dashboard ]] || die 'The arm-dashboard entrypoint is missing.'
    else
        .venv/bin/python -c 'import cv2, numpy, scan_manager, server'
        [[ -s index.html ]] || die 'Dashboard HTML is missing.'
    fi
)

prepare_release() {
    local stage builder builder_uid
    if "$system"; then
        builder="${RAMION_BUILD_USER:-${SUDO_USER:-}}"
        builder_uid="$(id -u "$builder" 2>/dev/null)" ||
            die 'Set RAMION_BUILD_USER to a non-root maintainer account.'
        [[ "$builder_uid" != 0 ]] || die 'Builds require a non-root maintainer account.'
        [[ -x /usr/bin/python3.12 ]] || die 'Install system Python 3.12.'
        id "$runtime_user" > /dev/null
    fi

    # Virtual environments embed absolute paths. Build each one at its final location.
    stage="$(mktemp -d "$releases/${revision:0:12}.XXXXXX")"
    if "$system"; then
        chown -- "$builder" "$stage"
        runuser -u "$builder" -- /bin/bash -c "set -euo pipefail
$(declare -f die build_release)
build_release \"\$@\"" bash "$app" "$archive" "$stage" /usr/bin/python3.12 >&2
        rm -f -- "$stage/.release-commit"
        chown -hR -- "root:$(id -gn "$runtime_user")" "$stage"
        chmod -R -- g+rX,go-w,o-rwx "$stage"
        runuser -u "$runtime_user" -- /bin/bash -c "set -euo pipefail
$(declare -f die verify_release)
verify_release \"\$@\"" bash "$app" "$stage" >&2
    else
        build_release "$app" "$archive" "$stage" '' >&2
        verify_release "$app" "$stage" >&2
    fi
    printf '%s\n' "$revision" > "$stage/.release-commit"
    if "$system"; then
        chmod 0644 -- "$stage/.release-commit"
    fi
    printf '%s\n' "$stage"
}

prepared_path() {
    local target marker
    target="$(realpath -e -- "$1")"
    [[ "$(dirname -- "$target")" == "$releases" && -d "$target" &&
        -f "$target/.release-commit" && ! -L "$target/.release-commit" ]] &&
        [[ "$(stat -c %u -- "$target")" == 0 && "$(stat -c %u -- "$target/.release-commit")" == 0 ]] ||
        die "Select a prepared $app release under $releases."
    marker="$(cat -- "$target/.release-commit")"
    [[ "$marker" =~ ^[0-9a-f]{40}$ ]] || die "Invalid release marker in $target."
    printf '%s\n' "$target"
}

deploy() {
    local stage
    verify_archive "$1"
    lock_deployment
    require_switch_ready
    if [[ -L "$live" && -r "$live/.release-commit" && "$(cat "$live/.release-commit")" == "$revision" ]] &&
        service_ctl is-active --quiet "$service" && service_healthy; then
        printf '%s already runs %s\n' "$service" "$revision"
        return
    fi
    stage="$(set -e; prepare_release)"
    switch_to "$stage"
}

case "${1:-}" in
    package) [[ "$#" -le 2 ]] || die 'Usage: deploy.sh package [OUTPUT_DIR]'; package "${2:-}" ;;
    verify)
        # CI: build and check a packaged release exactly as `deploy` would, in a throwaway directory.
        # Touches no live paths, services or locks.
        [[ "$#" -eq 3 ]] || die 'Usage: deploy.sh verify APP ARCHIVE'
        [[ "$2" == arm-workbench || "$2" == arm-dashboard ]] || die 'Select arm-workbench or arm-dashboard.'
        app="$2"
        verify_archive "$3"
        stage="$(mktemp -d)"
        trap 'rm -rf -- "$stage"' EXIT
        build_release "$app" "$archive" "$stage" ''
        verify_release "$app" "$stage"
        printf 'Release %s of %s builds and passes its release checks.\n' "${revision:0:12}" "$app"
        ;;
    prepare)
        "$system" && [[ "$#" -eq 3 ]] || die 'Usage: deploy.sh --system prepare APP ARCHIVE'
        select_app "$2"
        verify_archive "$3"
        lock_deployment
        prepare_release
        ;;
    activate)
        "$system" && [[ "$#" -eq 3 ]] || die 'Usage: deploy.sh --system activate APP PREPARED_PATH'
        select_app "$2"
        lock_deployment
        target="$(prepared_path "$3")"
        switch_to "$target"
        ;;
    deploy)
        [[ "$#" -eq 3 ]] || die 'Usage: deploy.sh [--system] deploy APP ARCHIVE'
        select_app "$2"
        deploy "$3"
        ;;
    rollback)
        [[ "$#" -eq 2 ]] || die 'Usage: deploy.sh [--system] rollback APP'
        select_app "$2"
        lock_deployment
        [[ -L "$previous" && -d "$previous" ]] || die 'No previous release is available.'
        target="$(realpath -e -- "$previous")"
        if "$system"; then
            target="$(prepared_path "$target")"
        fi
        switch_to "$target"
        ;;
    *) die 'Usage: deploy.sh {package [OUTPUT_DIR]|[--system] deploy APP ARCHIVE|[--system] rollback APP|--system prepare APP ARCHIVE|--system activate APP PREPARED_PATH|verify APP ARCHIVE}' ;;
esac
