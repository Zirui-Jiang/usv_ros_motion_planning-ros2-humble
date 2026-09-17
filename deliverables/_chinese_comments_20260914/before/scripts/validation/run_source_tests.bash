#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/humble/setup.bash
set -u
repo="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
export PYTHONPATH="$repo/src/core/shared/common:$repo/src/core/decision_module/path_planner:$repo/src/core/decision_module/path_processor:$repo/src/core/decision_module/trajectory_planner:$repo/src/core/control_module/controller:$repo/src/core/perception_module/perception:$repo/src/sim_env:${PYTHONPATH:-}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-174}"
export OPENBLAS_NUM_THREADS=1
export OMP_NUM_THREADS=1
cd "$repo"
python3 -m pytest -q "$@"
