#!/usr/bin/env bash
# 启动 0211 单段任务适配和 DDS 桥；仿真另用 start_ladds_single_leg_sim.bash。
set -euo pipefail
# Humble apt packages use system Python 3.10; Conda base may use a different ABI.
# Only this script and its children use the adjusted environment.
export PATH="/usr/bin:/bin:$PATH"
unset PYTHONHOME
ladds_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
ladds_install="${LADDS_INSTALL_BASE:-/tmp/usv_ladds_single_leg_20260915/install}"
set +u
source /opt/ros/humble/setup.bash
source "$ladds_install/setup.bash"
set -u
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-1}"
ladds_host="${1:-$(ip -4 route show default | awk 'NR==1 {print $3}')}"
if [[ -z "$ladds_host" ]]; then
    printf '未找到中继地址，请通过第一个参数指定。\n' >&2
    exit 1
fi
exec ros2 launch usv_ladds_bridge single_leg.launch.py backend:=tcp \
    "relay_host:=$ladds_host" "relay_port:=${LADDS_RELAY_PORT:-17880}" \
    "use_sim_time:=${LADDS_USE_SIM_TIME:-true}"
