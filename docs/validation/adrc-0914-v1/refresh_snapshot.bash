#!/usr/bin/env bash
# 同一隔离快照内刷新本轮接入文件；仅在试验未运行时执行。
set -eo pipefail
adrc_source="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../../.." && pwd)"
adrc_workspace="$(cat "$adrc_source/docs/validation/adrc-0914-v1/workspace.txt")"
case "$adrc_workspace" in /home/usv/adrc-0914-20260914-*) ;; *) exit 2 ;; esac
test -d "$adrc_workspace/src"
tar --exclude=__pycache__ --exclude=.pytest_cache -C "$adrc_source" -cf - \
    src/core/control_module/controller src/core/control_module/usv_control \
    src/core/shared/usv_navigation src/sim_env/test \
    scripts docs/0914-controller-integration.md README.md \
    | tar -C "$adrc_workspace" -xf -
cd "$adrc_workspace"
unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH PYTHONPATH LD_LIBRARY_PATH
export PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin
source /opt/ros/humble/setup.bash
set -u
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MAKEFLAGS=-j1
colcon build --base-paths src --executor sequential \
    --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON 2>&1 | tee build-refresh.log
