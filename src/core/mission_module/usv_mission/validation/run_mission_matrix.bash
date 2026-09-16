#!/usr/bin/env bash
# 加载所选已构建工作区后运行任务矩阵调度器，其余参数原样传给 Python。
set -euo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/_environment.bash"
mission_ws="$task_source_root"
if [[ $# -gt 0 && "$1" != --* ]]; then
  mission_ws="$(realpath -- "$1")"
  shift
fi
task_source_environment "$mission_ws"
exec python3 "$mission_ws/$task_validation_relative/mission_matrix.py" --workspace "$mission_ws" "$@"
