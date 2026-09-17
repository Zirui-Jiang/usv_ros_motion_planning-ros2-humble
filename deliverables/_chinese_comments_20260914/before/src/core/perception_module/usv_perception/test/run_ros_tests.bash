#!/usr/bin/env bash
# Run this package's ROS tests after sourcing a built workspace with its interfaces.
set -e
source /opt/ros/humble/setup.bash
package_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
export PYTHONPATH="${package_root}:${package_root}/../perception:${package_root}/../../shared/usv_navigation:${PYTHONPATH:-}"
export ROS_DOMAIN_ID=173
python3 -m pytest -o "cache_dir=${package_root}/.pytest_cache" "${package_root}/test" -q "$@"
