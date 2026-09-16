#!/usr/bin/env bash
# 加载 ROS Humble 并配置当前感知源码路径，在独立域 173 运行包内 ROS 测试。
# Run this package's ROS tests after sourcing a built workspace with its interfaces.
set -e
source /opt/ros/humble/setup.bash
package_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
export PYTHONPATH="${package_root}:${package_root}/../perception:${package_root}/../../shared/usv_navigation:${PYTHONPATH:-}"
export ROS_DOMAIN_ID=173
python3 -m pytest -o "cache_dir=${package_root}/.pytest_cache" "${package_root}/test" -q "$@"
