#!/usr/bin/env bash
# 分容器联调的算法端：一个终端启动感知、决策、航控和 LaDDS 航路适配器，Gazebo 由 start_ladds_split_sim.bash 在仿真容器启动。
set -euo pipefail
# Select Humble's system Python for this process tree, including under Conda.
export PATH="/usr/bin:/bin:$PATH"
unset PYTHONHOME

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
profile_dir="$script_dir/validation/config"
stack_launch="$script_dir/ladds_split_nav.launch.py"
default_base="$HOME/.local/share/usv_ros_motion_planning/base/install"
base_install="${LADDS_BASE_INSTALL:-}"
route_install="${LADDS_ROUTE_INSTALL:-$(cd -- "$script_dir/.." && pwd -P)/install}"
backend="${LADDS_BACKEND:-native}"
library="${LADDS_LIBRARY_PATH:-}"
relay_host="${LADDS_RELAY_HOST:-}"
relay_port="${LADDS_RELAY_PORT:-17880}"
adapter=true
launch_arguments=()

usage() {
    cat <<EOF
Usage: bash ${0##*/} [options] [-- ROS_LAUNCH_ARGUMENTS...]

Starts perception, decision, control and the LaDDS route adapter without
Gazebo. Start the simulation in the other container first with:
  bash scripts/start_ladds_split_sim.bash

Options:
  --backend native|tcp  LaDDS transport (default: $backend)
                        native: Linux SDK .so loaded in this container
                        tcp: Windows relay start_ladds_route_relay.ps1
  --library FILE        Linux SDK .so for native
                        (default: \$LADDS_LIBRARY_PATH${library:+ = $library})
  --relay-host HOST     tcp only: relay address (default: IPv4 default gateway)
  --relay-port PORT     tcp only: relay port (default: $relay_port)
  --base-install DIR    Underlay loaded before the workspace install
                        (default: $default_base when present)
  --install DIR         Workspace install (default: $route_install)
  --no-adapter          Start only the three navigation modules
  -h, --help            Show this help

native runs ROS from the SDK directory, where the SDK looks for its license
as with the Windows relay. Launch arguments after -- are forwarded, e.g.
controller_name:=lqr. Both split scripts always set ROS_DOMAIN_ID=42 and
ROS_LOCALHOST_ONLY=1, ignoring the caller's values; both containers need
host networking and host IPC.
EOF
}

fail() { printf 'ERROR: %s\n' "$*" >&2; exit 2; }

# 读取 IPv4 默认网关；容器镜像常不带 iproute2，因此直接解析 /proc/net/route。
default_gateway() {
    local interface destination gateway rest
    while read -r interface destination gateway rest; do
        if [[ "$destination" == 00000000 && "$gateway" != 00000000 ]]; then
            # The kernel prints the address as little-endian hexadecimal.
            printf '%d.%d.%d.%d\n' "0x${gateway:6:2}" "0x${gateway:4:2}" \
                "0x${gateway:2:2}" "0x${gateway:0:2}"
            return 0
        fi
    done < /proc/net/route
    return 1
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        --backend) [[ $# -ge 2 ]] || fail '--backend requires native or tcp'; backend="$2"; shift 2 ;;
        --library) [[ $# -ge 2 ]] || fail '--library requires FILE'; library="$2"; shift 2 ;;
        --relay-host) [[ $# -ge 2 ]] || fail '--relay-host requires HOST'; relay_host="$2"; shift 2 ;;
        --relay-port) [[ $# -ge 2 ]] || fail '--relay-port requires PORT'; relay_port="$2"; shift 2 ;;
        --base-install) [[ $# -ge 2 ]] || fail '--base-install requires DIR'; base_install="$2"; shift 2 ;;
        --install) [[ $# -ge 2 ]] || fail '--install requires DIR'; route_install="$2"; shift 2 ;;
        --no-adapter) adapter=false; shift ;;
        -h|--help) usage; exit 0 ;;
        --) shift; launch_arguments=("$@"); break ;;
        *) fail "unknown option: $1 (put ros2 launch arguments after --)" ;;
    esac
done

[[ "$backend" == native || "$backend" == tcp ]] || fail '--backend must be native or tcp'
[[ "$relay_port" =~ ^[0-9]+$ ]] && (( relay_port >= 1 && relay_port <= 65535 )) \
    || fail '--relay-port must be an integer in [1, 65535]'
# The simulation container loads the same profile, so these stay fixed here.
for argument in "${launch_arguments[@]}"; do
    case "$argument" in
        dynamics_config:=*|controller_config:=*|trajectory_config:=*|planner_config:=*|robot_x:=*|robot_y:=*|use_sim_time:=*|ladds_adapter:=*|backend:=*|relay_host:=*|relay_port:=*)
            fail "launch argument is owned by this profile: $argument" ;;
    esac
done
if [[ "$adapter" == true && "$backend" == native ]]; then
    [[ -n "$library" ]] || fail 'native backend needs the Linux SDK: pass --library FILE or set LADDS_LIBRARY_PATH'
    [[ -r "$library" ]] || fail "LaDDS SDK library is unreadable: $library"
    library="$(realpath -e -- "$library")"
    sdk_dir="$(dirname -- "$library")"
fi

# An explicit base must exist; otherwise the old underlay is optional because a
# full workspace build already contains every package.
if [[ -z "$base_install" && -r "$default_base/setup.bash" ]]; then
    base_install="$default_base"
fi
[[ -r /opt/ros/humble/setup.bash ]] || fail 'ROS Humble setup is unavailable'
[[ -z "$base_install" || -r "$base_install/setup.bash" ]] \
    || fail "base install has no setup.bash: $base_install (from --base-install or LADDS_BASE_INSTALL; fix or unset it)"
[[ -r "$route_install/setup.bash" ]] \
    || fail "workspace install has no setup.bash: $route_install (build the workspace in this container, or pass --install DIR)"
[[ -r "$profile_dir/ladds_single_leg_dynamics.yaml" ]] || fail 'LaDDS dynamics profile is missing'
[[ -r "$stack_launch" ]] || fail "launch file is missing: $stack_launch"

# ROS setup scripts may read unset variables; restore strict nounset afterwards.
set +u
source /opt/ros/humble/setup.bash
if [[ -n "$base_install" ]]; then
    source "$base_install/setup.bash"
fi
source "$route_install/setup.bash"
set -u

command -v ros2 >/dev/null || fail 'ros2 is unavailable after loading the selected installs'
navigation_share="$(ros2 pkg prefix --share usv_navigation)" \
    || fail 'usv_navigation is absent from the selected installs'
# Same four YAML files as start_ladds_single_leg_sim.bash passes to the simulation.
dynamics_config="$profile_dir/ladds_single_leg_dynamics.yaml"
controller_config="$navigation_share/config/robots/otter/controller_params_otter.yaml"
trajectory_config="$navigation_share/config/trajectory_planner/trajectory_planner_params.yaml"
planner_config="$navigation_share/config/path_planner/common_planner_params.yaml"
for config in "$controller_config" "$trajectory_config" "$planner_config"; do
    [[ -r "$config" ]] || fail "navigation config is unreadable: $config"
done

adapter_arguments=("ladds_adapter:=$adapter")
if [[ "$adapter" == true ]]; then
    adapter_arguments+=("backend:=$backend")
    if [[ "$backend" == tcp ]]; then
        if [[ -z "$relay_host" ]]; then
            relay_host="$(default_gateway)" || fail 'no IPv4 default gateway; pass --relay-host'
        fi
        adapter_arguments+=("relay_host:=$relay_host" "relay_port:=$relay_port")
    else
        # The bridge reads the SDK path from here when library_path is empty.
        export LADDS_LIBRARY_PATH="$library"
    fi
fi

# 必须与 start_ladds_split_sim.bash 相同；固定写死，不沿用容器环境里残留的其他值。
export ROS_DOMAIN_ID=42 ROS_LOCALHOST_ONLY=1

printf 'LaDDS split navigation (no Gazebo):\n'
printf '  base install: %s\n  workspace: %s\n' "${base_install:-none}" "$route_install"
printf '  dynamics: %s\n  ROS domain: %s, localhost only: %s\n' \
    "$dynamics_config" "$ROS_DOMAIN_ID" "$ROS_LOCALHOST_ONLY"
if [[ "$adapter" != true ]]; then
    printf '  LaDDS adapter: off (/otter/odom comes from the adapter)\n'
elif [[ "$backend" == native ]]; then
    printf '  LaDDS: native SDK %s\n  working directory: %s\n' "$library" "$sdk_dir"
else
    # The Windows relay serves one TCP client at a time.
    printf '  LaDDS relay: %s:%s (do not also run start_ladds_route.bash)\n' "$relay_host" "$relay_port"
fi

# The SDK looks for its license in the working directory, as with the Windows relay.
if [[ "$adapter" == true && "$backend" == native ]]; then
    cd -- "$sdk_dir"
fi
exec ros2 launch "$stack_launch" \
    "dynamics_config:=$dynamics_config" \
    "controller_config:=$controller_config" \
    "trajectory_config:=$trajectory_config" \
    "planner_config:=$planner_config" \
    "use_sim_time:=true" \
    "${adapter_arguments[@]}" \
    "${launch_arguments[@]}"
