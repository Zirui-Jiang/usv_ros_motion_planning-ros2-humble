#!/usr/bin/env bash
# 在独立 /tmp 输出目录增量构建多航点接口、导航链和 LaDDS 适配器。
set -euo pipefail
export PATH="/usr/bin:/bin:$PATH"
unset PYTHONHOME

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
source_root="$(cd -- "$script_dir/.." && pwd -P)"
base_install="${LADDS_BASE_INSTALL:-}"
output_root="${LADDS_ROUTE_OUTPUT:-/tmp/usv_ladds_multi_waypoint_20260915}"
packages=(common controller path_planner path_processor trajectory_planner
          usv_navigation_interfaces usv_navigation usv_decision usv_control usv_ladds_bridge)

usage() {
    cat <<EOF
Usage: bash ${0##*/} [options]

Options:
  --source DIR        Repository root (default: $source_root)
  --base-install DIR  Existing complete base
                      (default: \$HOME/.local/share/usv_ros_motion_planning/base/install)
  --output-root DIR   New isolated output root (default: $output_root)
  -h, --help          Show this help

Outputs: DIR/build, DIR/log, DIR/install. Existing output paths are reused for
a standard incremental colcon build; the script never deletes them.
EOF
}

fail() { printf 'ERROR: %s\n' "$*" >&2; exit 2; }

while [[ $# -gt 0 ]]; do
    case "$1" in
        --source) [[ $# -ge 2 ]] || fail '--source requires DIR'; source_root="$2"; shift 2 ;;
        --base-install) [[ $# -ge 2 ]] || fail '--base-install requires DIR'; base_install="$2"; shift 2 ;;
        --output-root) [[ $# -ge 2 ]] || fail '--output-root requires DIR'; output_root="$2"; shift 2 ;;
        -h|--help) usage; exit 0 ;;
        *) fail "unknown option: $1" ;;
    esac
done

if [[ -z "$base_install" ]]; then
    base_install="$HOME/.local/share/usv_ros_motion_planning/base/install"
fi

for executable in realpath mkdir; do
    command -v "$executable" >/dev/null || fail "$executable is unavailable"
done
[[ -r /opt/ros/humble/setup.bash ]] || fail 'ROS Humble setup is unavailable'
source_root="$(realpath -e -- "$source_root")"
base_install="$(realpath -e -- "$base_install")"
output_root="$(realpath -m -- "$output_root")"
[[ -d "$source_root/src/core" ]] || fail "source has no src/core: $source_root"
[[ -r "$base_install/setup.bash" ]] || fail "base install has no setup.bash: $base_install"
case "$output_root/" in
    /tmp/*) ;;
    *) fail 'output root must be an explicit path below /tmp' ;;
esac
mkdir -p -- "$output_root"

set +u
source /opt/ros/humble/setup.bash
source "$base_install/setup.bash"
set -u
command -v colcon >/dev/null || fail 'colcon is unavailable after loading ROS and the base install'

printf 'Building route overlay from %s\n' "$source_root"
printf 'Base: %s\nOutput: %s\nPackages: %s\n' \
    "$base_install" "$output_root" "${packages[*]}"
exec colcon --log-base "$output_root/log" build \
    --base-paths "$source_root/src/core" \
    --build-base "$output_root/build" \
    --install-base "$output_root/install" \
    --packages-select "${packages[@]}" \
    --allow-overriding "${packages[@]}" \
    --symlink-install --executor sequential
