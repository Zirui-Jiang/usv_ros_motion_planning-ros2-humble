"""Real ROS startup checks with a loopback fake relay; no vessel task is sent."""

import os
from pathlib import Path
import signal
import socket
import subprocess
import sys
import threading
import time

import pytest


ROOT = Path(__file__).resolve().parents[2]
INSTALL = Path(os.environ.get("LADDS_INSTALL_BASE", "/tmp/usv_ladds_single_leg_20260915/install"))
CONDA = Path("/home/usv/miniconda3/bin")
pytestmark = pytest.mark.skipif(
    sys.platform != "linux" or not (INSTALL / "setup.bash").is_file()
    or not (CONDA / "python3").is_file(),
    reason="requires local WSL Humble install and Conda base",
)


def _environment(port):
    env = os.environ.copy()
    env.update(
        PATH=f"{CONDA}:{env['PATH']}",
        LADDS_INSTALL_BASE=str(INSTALL), LADDS_RELAY_PORT=str(port),
        LADDS_USE_SIM_TIME="false", ROS_DOMAIN_ID="84", ROS_LOCALHOST_ONLY="1",
    )
    return env


def _stop(process):
    if process.poll() is None:
        os.killpg(process.pid, signal.SIGINT)
        try:
            process.wait(timeout=10)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait(timeout=5)


@pytest.mark.parametrize("script", ["start_ladds_bridge.bash", "start_ladds_single_leg.bash"])
def test_conda_base_starts_real_ros_nodes_and_completes_tcp_hello(script, tmp_path):
    sys.path.insert(0, str(ROOT / "src/core/communication_module/usv_ladds_bridge"))
    from usv_ladds_bridge.relay import RelayServer

    class IdleDDS:
        version = "startup-test-no-vessel-commands"

        def start(self): pass
        def close(self): pass
        def receive(self): return None
        def send(self, payload): raise AssertionError("startup probe must not send tasks")

    server = RelayServer(IdleDDS(), "127.0.0.1", 0, io_timeout=0.02)
    server.start()
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    env = _environment(server.address[1])
    # Confirm the inherited PATH actually selects Conda before invoking the fix.
    executable = subprocess.check_output(
        ["python3", "-c", "import sys; print(sys.executable)"], env=env, text=True,
    ).strip()
    assert executable.startswith(str(CONDA))
    output = tmp_path / "launch.log"
    process = None
    try:
        with output.open("w") as log:
            process = subprocess.Popen(
                ["bash", str(ROOT / "scripts" / script), "127.0.0.1"],
                env=env, stdout=log, stderr=subprocess.STDOUT, start_new_session=True,
            )
            deadline = time.monotonic() + 12
            while time.monotonic() < deadline and process.poll() is None:
                if "LaDDS bridge started" in output.read_text():
                    break
                time.sleep(0.05)
            assert "LaDDS bridge started" in output.read_text(), output.read_text()
            assert "ModuleNotFoundError" not in output.read_text()
    finally:
        if process is not None:
            _stop(process)
        server.close()
        thread.join(timeout=1)


def test_connection_failure_shuts_down_single_leg_adapter(tmp_path):
    # Bound, non-listening socket reserves a port where connect must fail.
    with socket.socket() as unused:
        unused.bind(("127.0.0.1", 0))
        output = tmp_path / "failed-launch.log"
        with output.open("w") as log:
            process = subprocess.Popen(
                ["bash", str(ROOT / "scripts/start_ladds_single_leg.bash"), "127.0.0.1"],
                env=_environment(unused.getsockname()[1]), stdout=log,
                stderr=subprocess.STDOUT, start_new_session=True,
            )
            try:
                process.wait(timeout=12)
            finally:
                _stop(process)
        result = output.read_text()
        assert "LaDDS bridge failed to start" in result, result
        assert "ModuleNotFoundError" not in result, result
        assert "[ladds_single_leg_node-2]: process has finished cleanly" in result, result
