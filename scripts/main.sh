#!/bin/bash
# 加载 ROS 与指定安装区，准备 Gazebo 资源后启动普通导航主入口。
# Start the usv_ros_motion_planning simulation (ROS 2 Humble).

# 让失败命令或管道失败传播为非零退出，避免后续步骤掩盖前一步错误。
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
# 允许通过环境变量选择外部安装区，未指定时使用项目内 install。
USV_SELECTED_INSTALL="${USV_INSTALL_DIR:-${PROJECT_ROOT}/install}"

# Source the ROS 2 environment and the workspace.
# 先加载系统 ROS，再加载工作区 overlay，使包与消息类型按选定环境解析。
source /opt/ros/humble/setup.bash
if [ -f "${USV_SELECTED_INSTALL}/setup.bash" ]; then
    source "${USV_SELECTED_INSTALL}/setup.bash"
else
    printf '[main] Workspace setup not found: %s\n' "${USV_SELECTED_INSTALL}/setup.bash" >&2
    printf '[main] Build the project with scripts/build.sh, or select an existing install: USV_INSTALL_DIR="/absolute/path/to/install" bash scripts/main.sh [launch arguments]\n' >&2
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

# 若 Windows 上 VcXsrv 已在听 6000，把 GUI 指过去，避开 Win10 WSLg 的 COPY MODE。
# Prefer a reachable Windows X server over WSLg so Gazebo/RViz appear as native windows.
_usv_use_windows_x_if_available() {
    local display
    display="$(python3 -c 'from sim_env.windows_display import windows_x_display; print(windows_x_display())' 2>/dev/null || true)"
    [[ -n "$display" ]] || return 0
    export DISPLAY="$display"
    export LIBGL_ALWAYS_INDIRECT="${LIBGL_ALWAYS_INDIRECT:-0}"
    export QT_X11_NO_MITSHM=1
    export QT_QPA_PLATFORM="${QT_QPA_PLATFORM:-xcb}"
    export GDK_BACKEND="${GDK_BACKEND:-x11}"
    unset WAYLAND_DISPLAY || true
    printf '[main] GUI display: %s (Windows X server)\n' "$DISPLAY"
}
_usv_use_windows_x_if_available

# Generate gazebo world obstacles from the user config.
# 此步骤会按场景配置尝试生成障碍；生成失败沿用现有脚本的继续启动行为。
if [ -f "src/user_config/user_config.yaml" ]; then
    (cd src/plugins/dynamic_xml_config && /usr/bin/python3 main_generate.py user_config.yaml) || true
fi

# Launch the simulation.
# 用 launch 替换当前 shell，并原样传递所有参数，保留含空格的参数边界。
exec ros2 launch sim_env main.launch.py "$@"
