#!/usr/bin/env bash
# 在全新 ASCII 临时目录复制依赖闭包并构建，规避 Humble rosidl 的中文路径解析问题。
# 不覆盖现有源码/安装，不启动 ROS/Gazebo 节点；验证目录保留供审计。
set -eo pipefail
repo="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
astar_check="$(mktemp -d /tmp/usv-astar-pruning.XXXXXX)"
printf 'ASTAR_VERIFY_ROOT=%s\n' "$astar_check"
mkdir "$astar_check/src"
packages=(shared/common shared/usv_navigation_interfaces shared/usv_navigation
          control_module/controller decision_module/path_processor
          decision_module/path_planner decision_module/trajectory_planner
          decision_module/usv_decision)
for package in "${packages[@]}"; do
  cp -a -- "$repo/src/core/$package" "$astar_check/src/"
done
unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH PYTHONPATH
source /opt/ros/humble/setup.bash
export PYTHONDONTWRITEBYTECODE=1 PYTHONNOUSERSITE=1 OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1
export MAKEFLAGS=-j2 CMAKE_BUILD_PARALLEL_LEVEL=2 COLCON_DEFAULTS_FILE=/dev/null
export COLCON_HOME="$astar_check/colcon-home"
cd "$astar_check"
colcon --log-base "$astar_check/log" build \
  --base-paths "$astar_check/src" --build-base "$astar_check/build" \
  --install-base "$astar_check/install" --packages-up-to usv_decision \
  --executor sequential --event-handlers console_cohesion- \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
source "$astar_check/install/setup.bash"
python3 -B "$repo/scripts/validation/check_astar_shortcut_install.py" "$astar_check/install"
