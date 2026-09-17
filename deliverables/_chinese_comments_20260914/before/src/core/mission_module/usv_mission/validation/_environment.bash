#!/usr/bin/env bash
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
