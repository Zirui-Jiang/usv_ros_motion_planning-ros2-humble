#!/usr/bin/env bash
# 启动 0211 航路适配器和 LaDDS 桥（rx+tx 同进程）。
set -euo pipefail
export PATH="/usr/bin:/bin:$PATH"
unset PYTHONHOME

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
workspace="$(cd -- "$script_dir/.." && pwd -P)"
route_install="${LADDS_ROUTE_INSTALL:-${LADDS_INSTALL_BASE:-$workspace/install}}"
relay_host="${LADDS_RELAY_HOST:-}"
relay_port="${LADDS_RELAY_PORT:-17880}"
use_sim_time="${LADDS_USE_SIM_TIME:-true}"

usage() {
    cat <<EOF
Usage: bash ${0##*/} [options]

Options:
  --install DIR       Workspace install (default: $route_install)
  --relay-host IPv4   Windows relay address (default: WSL default gateway)
  --relay-port PORT   TCP relay port (default: $relay_port)
  --use-sim-time BOOL ROS simulation clock setting (default: $use_sim_time)
  -h, --help          Show this help

Environment defaults are ROS_DOMAIN_ID=42 and ROS_LOCALHOST_ONLY=1.
EOF
}

fail() { printf 'ERROR: %s\n' "$*" >&2; exit 2; }

while [[ $# -gt 0 ]]; do
    case "$1" in
        --install) [[ $# -ge 2 ]] || fail '--install requires DIR'; route_install="$2"; shift 2 ;;
        --relay-host) [[ $# -ge 2 ]] || fail '--relay-host requires IPv4'; relay_host="$2"; shift 2 ;;
        --relay-port) [[ $# -ge 2 ]] || fail '--relay-port requires PORT'; relay_port="$2"; shift 2 ;;
        --use-sim-time) [[ $# -ge 2 ]] || fail '--use-sim-time requires BOOL'; use_sim_time="$2"; shift 2 ;;
        -h|--help) usage; exit 0 ;;
        *) fail "unknown option: $1" ;;
    esac
done

[[ "$relay_port" =~ ^[0-9]+$ ]] && (( relay_port >= 1 && relay_port <= 65535 )) \
    || fail '--relay-port must be an integer in [1, 65535]'
[[ "$use_sim_time" == true || "$use_sim_time" == false ]] \
    || fail '--use-sim-time must be true or false'

[[ -r "$route_install/setup.bash" ]] \
    || fail "install has no setup.bash: $route_install (run scripts/build_ladds_route.bash first)"

: "${ROS_DOMAIN_ID:=42}"
: "${ROS_LOCALHOST_ONLY:=1}"
export ROS_DOMAIN_ID ROS_LOCALHOST_ONLY

set +u
source /opt/ros/humble/setup.bash
source "$route_install/setup.bash"
set -u

if [[ -z "$relay_host" ]]; then
    relay_host="$(ip -4 route show default | awk 'NR==1 {print $3}')"
fi
if [[ -z "$relay_host" ]]; then
    printf '未找到中继地址，请通过 --relay-host 指定。\n' >&2
    exit 1
fi

exec ros2 launch usv_ladds_bridge route.launch.py backend:=tcp \
    "relay_host:=$relay_host" "relay_port:=$relay_port" \
    "use_sim_time:=$use_sim_time"
