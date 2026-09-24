#!/usr/bin/env bash
# 在仓库自己的 build/、install/、log/ 中增量构建多航点接口、导航链和 LaDDS 适配器。
set -euo pipefail
export PATH="/usr/bin:/bin:$PATH"
unset PYTHONHOME

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
workspace="$(cd -- "$script_dir/.." && pwd -P)"
base_install="${LADDS_BASE_INSTALL:-$HOME/.local/share/usv_ros_motion_planning/base/install}"
packages=(common controller path_planner path_processor trajectory_planner
          usv_navigation_interfaces usv_navigation usv_decision usv_control usv_ladds_bridge)

usage() {
    cat <<EOF
Usage: bash ${0##*/} [options]

Options:
  --base-install DIR  Underlay sourced before building (default: $base_install)
  --packages NAME...  Rebuild only these packages
                      (default: the ten route packages)
  -h, --help          Show this help

Outputs: $workspace/build, $workspace/install, $workspace/log.
Existing outputs are reused for a standard incremental colcon build; the script
never deletes them.
EOF
}

fail() { printf 'ERROR: %s\n' "$*" >&2; exit 2; }

while [[ $# -gt 0 ]]; do
    case "$1" in
        --base-install) [[ $# -ge 2 ]] || fail '--base-install requires DIR'; base_install="$2"; shift 2 ;;
        --packages)
            shift
            packages=()
            while [[ $# -gt 0 && "$1" != --* ]]; do
                packages+=("$1")
                shift
            done
            [[ ${#packages[@]} -gt 0 ]] || fail '--packages requires at least one package name'
            ;;
        --source|--output-root)
            fail "$1 was removed; this script now builds into $workspace/install" ;;
        -h|--help) usage; exit 0 ;;
        *) fail "unknown option: $1" ;;
    esac
done

command -v realpath >/dev/null || fail 'realpath is unavailable'
[[ -r /opt/ros/humble/setup.bash ]] || fail 'ROS Humble setup is unavailable'
base_install="$(realpath -e -- "$base_install")"
[[ -r "$base_install/setup.bash" ]] || fail "base install has no setup.bash: $base_install"
[[ -d "$workspace/src/core" ]] || fail "workspace has no src/core: $workspace"

set +u
source /opt/ros/humble/setup.bash
source "$base_install/setup.bash"
set -u
command -v colcon >/dev/null || fail 'colcon is unavailable after loading ROS and the base install'

# --base-paths src keeps deliverables/ copies out of discovery. The workspace
# install is a copy install, so do not switch it to --symlink-install here.
cd -- "$workspace"
printf 'Building route packages in workspace %s\n' "$workspace"
printf 'Base: %s\nInstall: %s\nPackages: %s\n' \
    "$base_install" "$workspace/install" "${packages[*]}"
exec colcon build --base-paths src \
    --packages-select "${packages[@]}" \
    --allow-overriding "${packages[@]}" \
    --executor sequential
