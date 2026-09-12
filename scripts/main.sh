#!/bin/bash
# Start the usv_ros_motion_planning simulation (ROS 2 Humble).

set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

# Source the ROS 2 environment and the workspace.
source /opt/ros/humble/setup.bash
if [ -f "${PROJECT_ROOT}/install/setup.bash" ]; then
    source "${PROJECT_ROOT}/install/setup.bash"
else
    echo "[main] Workspace not built, please run scripts/build.sh first." >&2
    exit 1
fi

# Source the classic Gazebo environment so gzserver/gzclient inherit the
# default GAZEBO_RESOURCE_PATH (rtshaderlib) and GAZEBO_PLUGIN_PATH. Without
# this the RTShaderSystem cannot find its shader library and gzclient aborts
# with a null Camera assertion on startup.
if [ -f /usr/share/gazebo/setup.sh ]; then
    source /usr/share/gazebo/setup.sh
fi

cd "${PROJECT_ROOT}"

# Generate gazebo world obstacles from the user config.
if [ -f "src/user_config/user_config.yaml" ]; then
    (cd src/plugins/dynamic_xml_config && /usr/bin/python3 main_generate.py user_config.yaml) || true
fi

# Launch the simulation.
exec ros2 launch sim_env main.launch.py "$@"
