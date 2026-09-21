# 探测 Windows 上可达的 VcXsrv，把 GUI 从 WSLg COPY MODE 转到本机窗口。
"""Prefer a reachable Windows X server over WSLg on Windows 10."""
import os
import socket
import subprocess


X11_PORT = 6000
_PROBE_TIMEOUT = 0.4


# 从默认路由取出 Windows 主机地址；WSL2 NAT 下通常是 vEthernet 网关。
def _default_route_ip():
    try:
        output = subprocess.check_output(
            ["ip", "-4", "route", "show", "default"], text=True, timeout=0.5)
    except (OSError, subprocess.SubprocessError):
        return ""
    parts = output.split()
    if "via" in parts:
        return parts[parts.index("via") + 1]
    return ""


# 只探测 TCP 6000 是否可连，避免对 X 服务器发送协议数据。
def _reachable(host):
    try:
        socket.create_connection((host, X11_PORT), _PROBE_TIMEOUT).close()
    except OSError:
        return False
    return True


# 返回应设置的 DISPLAY；没有可达的 Windows X 时返回空字符串，保留 WSLg。
def windows_x_display():
    """Return DISPLAY for a reachable Windows X server, or an empty string."""
    override = os.environ.get("USV_DISPLAY", "").strip()
    if override:
        return override
    host = os.environ.get("USV_WINDOWS_X_HOST", "").strip() or _default_route_ip()
    if not host or not _reachable(host):
        return ""
    return host + ":0.0"


# 子进程显示 GUI 所需的环境；不修改调用方的 os.environ。
def environment_for_display(display):
    """Qt/Gazebo flags required when DISPLAY is a TCP Windows X server."""
    return {
        "DISPLAY": display,
        "QT_X11_NO_MITSHM": "1",
        "QT_QPA_PLATFORM": os.environ.get("QT_QPA_PLATFORM") or "xcb",
        "GDK_BACKEND": os.environ.get("GDK_BACKEND") or "x11",
        "LIBGL_ALWAYS_INDIRECT": os.environ.get("LIBGL_ALWAYS_INDIRECT") or "0",
    }


# 仅在需要 GUI 且探测到 Windows X 时改 launch 子进程环境。
def launch_environment_actions(enabled):
    """Launch actions that point gzclient/RViz at Windows X when available."""
    from launch.actions import LogInfo, SetEnvironmentVariable
    try:
        from launch.actions import UnsetEnvironmentVariable
    except ImportError:  # pragma: no cover - Humble provides this action.
        UnsetEnvironmentVariable = None
    if not enabled:
        return []
    display = windows_x_display()
    if not display:
        return []
    actions = [LogInfo(msg="[sim_env] GUI display: " + display + " (Windows X server)")]
    for name, value in environment_for_display(display).items():
        actions.append(SetEnvironmentVariable(name, value))
    if UnsetEnvironmentVariable is not None:
        actions.append(UnsetEnvironmentVariable("WAYLAND_DISPLAY"))
    return actions
