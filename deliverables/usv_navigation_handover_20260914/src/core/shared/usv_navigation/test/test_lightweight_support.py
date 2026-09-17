"""Perception lifecycle helpers must not load navigation algorithm modules."""
from pathlib import Path
import subprocess
import sys

import pytest


@pytest.mark.parametrize('module', ['heartbeat', 'node_runner'])
def test_lightweight_helper_import_does_not_load_business_or_algorithm_modules(module):
    if module == 'node_runner':
        pytest.importorskip('rclpy')
    package = Path(__file__).resolve().parents[1]
    script = (
        'import importlib, sys\n'
        f'sys.path.insert(0, {str(package)!r})\n'
        f'importlib.import_module("usv_navigation.{module}")\n'
        'forbidden = ("usv_decision", "usv_control", "sim_env", '
        '"controller", "path_planner", "trajectory_planner")\n'
        'loaded = [name for name in sys.modules '
        'if any(name == prefix or name.startswith(prefix + ".") for prefix in forbidden)]\n'
        'assert not loaded, loaded\n'
    )
    result = subprocess.run([sys.executable, '-c', script], text=True, capture_output=True)
    assert result.returncode == 0, result.stdout + result.stderr


def test_ros_support_retains_the_shared_runner_alias():
    pytest.importorskip('rclpy')
    from usv_navigation.node_runner import run_node
    from usv_navigation.ros_support import run_node as compatibility_run_node
    assert compatibility_run_node is run_node
