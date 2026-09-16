# 验证感知可用的轻量心跳/生命周期工具不会导入业务算法，并保留运行器兼容别名。
"""Perception lifecycle helpers must not load navigation algorithm modules."""
from pathlib import Path
import subprocess
import sys

import pytest


# 在独立解释器导入工具，检查 sys.modules 未加载业务或导航算法包。
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


# ROS 支持模块导出的 run_node 与共享运行器为同一个对象。
def test_ros_support_retains_the_shared_runner_alias():
    pytest.importorskip('rclpy')
    from usv_navigation.node_runner import run_node
    from usv_navigation.ros_support import run_node as compatibility_run_node
    assert compatibility_run_node is run_node
