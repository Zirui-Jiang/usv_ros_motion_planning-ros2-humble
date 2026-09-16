#!/usr/bin/env bash
# Linux/WSL 侧：只启动测试消息桥；不启动 Gazebo 或导航节点。
set -euo pipefail
# Keep /usr/bin/env python3 node scripts on Humble's system Python under Conda.
export PATH="/usr/bin:/bin:$PATH"
unset PYTHONHOME
ladds_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
ladds_install="${LADDS_INSTALL_BASE:-$ladds_root/install}"
# ROS 的 setup 脚本可能读取未设置变量，加载环境时暂时关闭 nounset。
set +u
source /opt/ros/humble/setup.bash
if [[ -f "$ladds_install/setup.bash" ]]; then
    source "$ladds_install/setup.bash"
else
    printf '未找到安装区 %s；请先按通信桥 README 构建。\n' "$ladds_install" >&2
    exit 1
fi
set -u
ladds_relay_host="${1:-$(ip -4 route show default | awk 'NR==1 {print $3}')}"
if [[ -z "$ladds_relay_host" ]]; then
    printf '未找到 Windows 网关地址，请把中继 IPv4 作为第一个参数传入。\n' >&2
    exit 1
fi
exec ros2 launch usv_ladds_bridge bridge.launch.py \
    backend:=tcp "relay_host:=$ladds_relay_host" "relay_port:=${LADDS_RELAY_PORT:-17880}"
