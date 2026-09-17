#!/usr/bin/env bash
set -euo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/_environment.bash"
mission_ws="$task_source_root"
if [[ $# -gt 0 && "$1" != --* ]]; then
  mission_ws="$(realpath -- "$1")"
  shift
fi
target=mission
if [[ "${1:-}" == --target ]]; then
  target="${2:?--target requires mission or simulation}"
  shift 2
fi
task_source_environment "$mission_ws"
case "$target" in
  mission) harness="$mission_ws/$task_validation_relative/task_ros_acceptance.py" ;;
  simulation)
    mapfile -t candidates < <(find "$mission_ws/scripts" -type f -name ros_acceptance.py)
    if [[ ${#candidates[@]} -ne 1 ]]; then
      echo 'Expected exactly one legacy ros_acceptance.py below scripts.' >&2
      exit 2
    fi
    harness="${candidates[0]}"
    ;;
  *) echo "Unsupported navigation regression target: $target" >&2; exit 2 ;;
esac
exec python3 "$harness" --workspace "$mission_ws" "$@"
