# 验证 Windows X 探测只在显式覆盖或 6000 端口可达时返回 DISPLAY。
"""Unit tests for Windows X display selection without opening a server."""

from sim_env.windows_display import environment_for_display, windows_x_display


def test_usv_display_override_skips_port_probe(monkeypatch):
    monkeypatch.setenv("USV_DISPLAY", "10.0.0.1:0.0")
    monkeypatch.setattr("sim_env.windows_display._reachable", lambda host: False)
    assert windows_x_display() == "10.0.0.1:0.0"


def test_missing_windows_x_keeps_wslg(monkeypatch):
    monkeypatch.delenv("USV_DISPLAY", raising=False)
    monkeypatch.delenv("USV_WINDOWS_X_HOST", raising=False)
    monkeypatch.setattr("sim_env.windows_display._default_route_ip", lambda: "172.27.32.1")
    monkeypatch.setattr("sim_env.windows_display._reachable", lambda host: False)
    assert windows_x_display() == ""


def test_reachable_gateway_becomes_display(monkeypatch):
    monkeypatch.delenv("USV_DISPLAY", raising=False)
    monkeypatch.delenv("USV_WINDOWS_X_HOST", raising=False)
    monkeypatch.setattr("sim_env.windows_display._default_route_ip", lambda: "172.27.32.1")
    monkeypatch.setattr("sim_env.windows_display._reachable", lambda host: host == "172.27.32.1")
    assert windows_x_display() == "172.27.32.1:0.0"
    assert environment_for_display("172.27.32.1:0.0")["QT_X11_NO_MITSHM"] == "1"
