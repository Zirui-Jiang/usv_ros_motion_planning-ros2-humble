#!/usr/bin/env bash
set -euo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/_environment.bash"
task_source_environment "$task_source_root"
export PYTHONPATH="$task_source_root/src/core/mission_module/usv_mission:${PYTHONPATH:-}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-174}"
cd -- "$task_source_root"
if [[ $# -eq 0 ]]; then
  # Explicit source paths include the new package even with old root testpaths.
  exec python3 -m pytest -q src scripts
fi
exec python3 -m pytest -q "$@"
