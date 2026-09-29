#!/usr/bin/env bash
# 分容器联调的仿真端：只启动 Gazebo、船模和真值定位，感知/决策/航控和 LaDDS 适配器由 start_ladds_split_nav.bash 启动。
set -euo pipefail
export PATH="/usr/bin:/bin:$PATH"
unset PYTHONHOME

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
single_sim="$script_dir/start_ladds_single_leg_sim.bash"
default_base="$HOME/.local/share/usv_ros_motion_planning/base/install"
# Same install selection as start_ladds_route_sim.bash; --install overrides it.
LADDS_SINGLE_LEG_INSTALL="${LADDS_ROUTE_INSTALL:-$(cd -- "$script_dir/.." && pwd -P)/install}"
workspace_install="$LADDS_SINGLE_LEG_INSTALL"
base_install="${LADDS_BASE_INSTALL:-}"
base_source=LADDS_BASE_INSTALL
# 两个容器必须在同一 ROS 域；固定写死，不沿用容器环境里残留的其他值。
ROS_DOMAIN_ID=42
ROS_LOCALHOST_ONLY=1

fail() { printf 'ERROR: %s\n' "$*" >&2; exit 2; }

# 预读转发给单航段脚本的选项，使兜底 base 跟随 --install。
previous=""
for argument in "$@"; do
    [[ "$argument" == -- ]] && break
    case "$previous" in
        --install) workspace_install="$argument" ;;
        --base-install) base_install="$argument"; base_source=--base-install ;;
    esac
    previous="$argument"
done

if [[ ${1:-} == -h || ${1:-} == --help ]]; then
    cat <<EOF
Usage: bash ${0##*/} [single-leg simulation options]

Starts only Gazebo, the vessel model and ground-truth localization
(/otter/odom_gt) through:
  $single_sim
Start perception, decision, control and the LaDDS adapter in the other
container with: bash scripts/start_ladds_split_nav.bash

Defaults:
  workspace:     $workspace_install
  base install:  \$LADDS_BASE_INSTALL, else $default_base
                 when present, else the workspace install
  ROS domain:    $ROS_DOMAIN_ID (fixed, same as start_ladds_split_nav.bash)
  localhost:     $ROS_LOCALHOST_ONLY (fixed)

Both containers need host networking, host IPC and the same ROS domain.
Forwarded options include --world NAME|FILE, --headless true|false,
--base-install DIR, --install DIR, and extra launch arguments after --.
EOF
    exit 0
fi

[[ -r "$workspace_install/setup.bash" ]] \
    || fail "workspace install has no setup.bash: $workspace_install (build the workspace in this container, or pass --install DIR)"
if [[ -n "$base_install" ]]; then
    [[ -r "$base_install/setup.bash" ]] \
        || fail "$base_source points to $base_install, which has no setup.bash; fix or unset it"
elif [[ -r "$default_base/setup.bash" ]]; then
    base_install="$default_base"
else
    # A full workspace build is itself complete, so machines and containers
    # without the old underlay load the workspace install as the base.
    base_install="$workspace_install"
fi
LADDS_BASE_INSTALL="$base_install"
LADDS_SIM_LAUNCH=simulation_support.launch.py
export LADDS_SINGLE_LEG_INSTALL LADDS_BASE_INSTALL LADDS_SIM_LAUNCH ROS_DOMAIN_ID ROS_LOCALHOST_ONLY

[[ -r "$single_sim" ]] || fail "missing simulation profile: $single_sim"
exec bash "$single_sim" "$@"
