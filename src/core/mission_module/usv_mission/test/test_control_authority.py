# 控制发布权回归：检查实际重映射话题的独占性、冲突锁存和任务/仿真导入边界。
"""Actuator ownership is fail closed and cannot resume after a conflict."""
import ast
from pathlib import Path

from usv_mission.control_authority import ControlAuthority


# 验证：每个实际输出话题都已发现且恰有一个发布者才放行。
def test_every_remapped_output_must_be_exclusive_and_discovered():
    gate = ControlAuthority()
    assert not gate.observe({})
    assert not gate.observe({'/robot/motor_a': 1, '/robot/motor_b': 0})
    assert not gate.conflicted
    assert gate.observe({'/robot/motor_a': 1, '/robot/motor_b': 1})
    assert gate.status()['restart_required'] is False


# 验证：第二发布者消失后冲突仍锁存，必须重启才能恢复。
def test_second_publisher_latches_even_when_the_competitor_disappears():
    gate = ControlAuthority()
    assert not gate.observe({'/robot/motor_a': 1, '/robot/motor_b': 2})
    reason = gate.reason
    assert gate.conflicted and '/robot/motor_b' in reason and 'restart' in reason
    assert not gate.observe({'/robot/motor_a': 1, '/robot/motor_b': 1})
    assert gate.reason == reason
    assert gate.status()['restart_required'] is True


# 验证：图查询异常阻止输出，但不能擦除已发现的冲突。
def test_graph_failure_inhibits_output_and_cannot_clear_conflict():
    gate = ControlAuthority()
    assert not gate.unavailable('DDS graph unavailable')
    assert gate.observe({'/left': 1, '/right': 1})
    assert not gate.observe({'/left': 2, '/right': 1})
    reason = gate.reason
    assert not gate.unavailable('DDS graph unavailable')
    assert gate.reason == reason and gate.conflicted


# 验证：任务运行时代码不反向导入仿真模块。
def test_runtime_imports_do_not_cross_back_into_simulation():
    source = Path(__file__).parents[1] / 'usv_mission'
    for path in source.rglob('*.py'):
        tree = ast.parse(path.read_text(encoding='utf-8'))
        for node in ast.walk(tree):
            if isinstance(node, ast.Import):
                assert all(name.name.split('.')[0] != 'sim_env' for name in node.names), path
            elif isinstance(node, ast.ImportFrom):
                assert (node.module or '').split('.')[0] != 'sim_env', path
