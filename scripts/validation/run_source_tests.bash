#!/usr/bin/env bash
# 加载 ROS 并配置源码导入路径，执行 pytest；附加命令行参数直接转交 pytest。
# 让失败命令或管道失败传播为非零退出，避免后续步骤掩盖前一步错误。
set -eo pipefail
# 先加载系统 ROS，再加载工作区 overlay，使包与消息类型按选定环境解析。
source /opt/ros/humble/setup.bash
set -u
repo="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
export PYTHONPATH="$repo/src/core/shared/common:$repo/src/core/decision_module/path_planner:$repo/src/core/decision_module/path_processor:$repo/src/core/decision_module/trajectory_planner:$repo/src/core/control_module/controller:$repo/src/core/perception_module/perception:$repo/src/sim_env:${PYTHONPATH:-}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-174}"
export OPENBLAS_NUM_THREADS=1
export OMP_NUM_THREADS=1
cd "$repo"
python3 -m pytest -q "$@"
