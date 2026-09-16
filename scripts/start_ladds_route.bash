#!/usr/bin/env bash
# 多航点0211入口：复用现有适配器启动脚本，只切换安装区和ROS隔离域。
set -euo pipefail
export PATH="/usr/bin:/bin:$PATH"
unset PYTHONHOME

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
single_adapter="$script_dir/start_ladds_single_leg.bash"
route_install="${LADDS_ROUTE_INSTALL:-/tmp/usv_ladds_multi_waypoint_20260915/install}"
relay_host="${LADDS_RELAY_HOST:-}"
relay_port="${LADDS_RELAY_PORT:-17880}"
use_sim_time="${LADDS_USE_SIM_TIME:-true}"

usage() {
    cat <<EOF
Usage: bash ${0##*/} [options]

Options:
  --install DIR       Multi-waypoint overlay (default: $route_install)
  --relay-host IPv4   Windows relay address (default: WSL default gateway)
  --relay-port PORT   TCP relay port (default: $relay_port)
  --use-sim-time BOOL ROS simulation clock setting (default: $use_sim_time)
  -h, --help          Show this help

Environment defaults are ROS_DOMAIN_ID=42 and ROS_LOCALHOST_ONLY=1.
This wrapper delegates to start_ladds_single_leg.bash; the installed adapter
submits the complete ordered points array through NavigateRoute.
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
[[ -r "$single_adapter" ]] || fail "missing adapter launcher: $single_adapter"

: "${ROS_DOMAIN_ID:=42}"
: "${ROS_LOCALHOST_ONLY:=1}"
export LADDS_INSTALL_BASE="$route_install"
export LADDS_RELAY_PORT="$relay_port"
export LADDS_USE_SIM_TIME="$use_sim_time"
export ROS_DOMAIN_ID ROS_LOCALHOST_ONLY

exec bash "$single_adapter" "$relay_host"
