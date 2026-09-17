#!/usr/bin/env bash
# Isolated ROS build/tests. Never uses the vessel DDS domain or existing install.
set -eo pipefail
repo=$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)
stage=${1:-all}
verify_root=${2:-}
case "$stage" in
    all|prepare|build|test|decision|core|integration) ;;
    *) echo 'Usage: run_stop_guard_checks.bash [all|prepare|build|test|decision|core|integration] [/tmp/usv-stop-guard.XXXXXX]' >&2; exit 2 ;;
esac
export PATH=/usr/bin:/bin
unset PYTHONHOME PYTHONPATH AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH
source /opt/ros/humble/setup.bash
export PYTHONDONTWRITEBYTECODE=1 PYTHONNOUSERSITE=1
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1
export MAKEFLAGS=-j2 CMAKE_BUILD_PARALLEL_LEVEL=2 COLCON_DEFAULTS_FILE=/dev/null
if [[ -z "$verify_root" ]]; then
    verify_root=$(mktemp -d /tmp/usv-stop-guard.XXXXXX)
fi

case "$verify_root" in
    /tmp/usv-stop-guard.*) ;;
    *) echo 'Expected an isolated /tmp/usv-stop-guard.* directory' >&2; exit 2 ;;
esac
printf 'VERIFY_ROOT=%s\n' "$verify_root"
export COLCON_HOME="$verify_root/colcon-home"
export ROS_DOMAIN_ID=86 ROS_LOCALHOST_ONLY=1 ROS_LOG_DIR="$verify_root/ros-log"
export LADDS_INSTALL_BASE="$verify_root/install"

if [[ "$stage" == prepare || "$stage" == build || "$stage" == all ]]; then
    mkdir -p "$verify_root/src"
    packages=(shared/usv_navigation_interfaces)
    if [[ "$stage" != prepare ]]; then
        packages+=(shared/common shared/usv_navigation control_module/controller
            control_module/usv_control decision_module/path_processor
            decision_module/path_planner decision_module/trajectory_planner
            decision_module/usv_decision communication_module/usv_ladds_bridge)
    fi
    for package in "${packages[@]}"; do
        cp -a "$repo/src/core/$package" "$verify_root/src/"
    done
    cd "$verify_root"
    colcon --log-base "$verify_root/log" build --base-paths "$verify_root/src" \
        --build-base "$verify_root/build" --install-base "$verify_root/install" \
        --executor sequential --cmake-args -DBUILD_TESTING=OFF
fi

if [[ "$stage" == test || "$stage" == all || "$stage" == decision || "$stage" == core ]]; then
    source "$verify_root/install/setup.bash"
    export PYTHONPATH="$repo/src/core/communication_module/usv_ladds_bridge:${PYTHONPATH:-}"
    cd "$repo"
    tests=(src/core/decision_module/usv_decision/test/test_stop_guard_decision_ros.py)
    if [[ "$stage" != decision ]]; then
        tests=(src/core/control_module/controller/test src/core/control_module/usv_control/test
            src/core/decision_module/usv_decision/test src/core/shared/usv_navigation/test)
        if [[ "$stage" != core ]]; then
            tests+=(src/core/communication_module/usv_ladds_bridge/test)
        fi
    fi
    /usr/bin/python3 -B -m pytest -q --tb=short --show-capture=no -p no:cacheprovider "${tests[@]}" \
        --junitxml="$verify_root/$stage-tests.xml"
fi

if [[ "$stage" == integration || "$stage" == all ]]; then
    source "$verify_root/install/setup.bash"
    cd "$repo"
    # This fixture uses its own domain and must run in a fresh pytest process.
    /usr/bin/python3 -B -m pytest -q --tb=short --show-capture=no -p no:cacheprovider \
        scripts/validation/test_ladds_route_execution_ros.py \
        --junitxml="$verify_root/route-integration.xml"
fi
