#!/usr/bin/env bash
# Run only this package's tests, without joining the project simulation domain.
set -e
source /opt/ros/humble/setup.bash
package_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
export PYTHONPATH="${package_root}:${PYTHONPATH:-}"
export ROS_DOMAIN_ID=173
python3 -m pytest -c "${package_root}/setup.cfg" \
  -o "cache_dir=${package_root}/.pytest_cache" "${package_root}/test" -q "$@"
