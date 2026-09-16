#!/usr/bin/env bash
# 加载所选 ROS 安装环境后运行任务闭环验收脚本，可附带场景和重复数参数。
set -euo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/_environment.bash"
mission_ws="$task_source_root"
if [[ $# -gt 0 && "$1" != --* ]]; then
  mission_ws="$(realpath -- "$1")"
  shift
fi
task_source_environment "$mission_ws"
exec python3 "$mission_ws/$task_validation_relative/mission_acceptance.py" --workspace "$mission_ws" "$@"
