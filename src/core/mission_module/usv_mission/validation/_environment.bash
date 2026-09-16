#!/usr/bin/env bash
# 供验收入口 source 的环境辅助脚本：向上定位源码根，再加载选定安装区和 ROS Humble。
# Sourced by task validation entrypoints. No fixed parent depth is assumed.
task_validation_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
task_source_root="$task_validation_dir"
while [[ ! -d "$task_source_root/src" ]]; do
  if [[ "$task_source_root" == / ]]; then
    echo 'Cannot find a source checkout containing src above this validation script.' >&2
    return 2
  fi
  task_source_root="$(dirname -- "$task_source_root")"
done
task_validation_relative="${task_validation_dir#"$task_source_root/"}"

# 显式加载已构建工作区；单线程 BLAS/OMP 限制数值库隐式并发。
task_source_environment() {
  local selected_workspace="$1"
  if [[ ! -f "$selected_workspace/install/setup.bash" ]]; then
    echo "Workspace is not built: $selected_workspace" >&2
    return 2
  fi
  set +u
  source /opt/ros/humble/setup.bash
  source "$selected_workspace/install/setup.bash"
  set -u
  export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 ROS_LOCALHOST_ONLY=1
}
