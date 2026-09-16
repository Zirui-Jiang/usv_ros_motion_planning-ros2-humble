#!/usr/bin/env bash
# 从当前工作树建立完整快照，不使用旧 install 或仅已提交的 HEAD。
set -eo pipefail
adrc_source="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../../.." && pwd)"
adrc_workspace="$(mktemp -d /home/usv/adrc-0914-20260914-XXXXXX)"
printf '%s\n' "$adrc_workspace" > "$adrc_source/docs/validation/adrc-0914-v1/workspace.txt"
tar --exclude=__pycache__ --exclude=.pytest_cache -C "$adrc_source" -cf - \
    src scripts docs README.md conftest.py pytest.ini 0914controller \
    | tar -C "$adrc_workspace" -xf -
cd "$adrc_workspace"
unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH PYTHONPATH LD_LIBRARY_PATH
export PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin
source /opt/ros/humble/setup.bash
set -u
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MAKEFLAGS=-j1
colcon build --base-paths src --executor sequential \
    --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON 2>&1 | tee build.log
printf '\nBuilt ADRC workspace: %s\n' "$adrc_workspace"
