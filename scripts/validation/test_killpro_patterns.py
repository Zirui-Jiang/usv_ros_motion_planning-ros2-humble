# 只检查清理脚本的命令行匹配范围，不实际执行 kill 或清理用户进程。
"""Check cleanup command-line coverage without executing cleanup or signals."""
from pathlib import Path
import re
import shlex

import pytest


SCRIPT = Path(__file__).resolve().parents[1] / 'killpro.sh'


def cleanup_patterns():
    # Read only the literal array. Never source/execute the cleanup script.
    text = SCRIPT.read_text(encoding='utf-8')
    match = re.search(r'^PATTERNS=\(\n(.*?)^\)', text, re.MULTILINE | re.DOTALL)
    assert match, 'The cleanup script must expose its literal command-line patterns'
    return shlex.split(match.group(1), comments=True)


@pytest.mark.parametrize('command', [
    '/ws/install/usv_perception/lib/usv_perception/costmap_support_node --ros-args',
    '/usr/bin/python3 /ws/install/usv_perception/lib/usv_perception/perception_mapping_node',
    '/usr/bin/python3 /ws/install/usv_perception/lib/usv_perception/perception_monitor',
    '/usr/bin/python3 /ws/install/usv_decision/lib/usv_decision/decision_node',
    '/usr/bin/python3 /ws/install/usv_control/lib/usv_control/control_node',
    '/ws/install/sim_env/lib/sim_env/fake_localization_node',
    '/ws/install/perception/lib/perception/perception_mapping_node',
    '/usr/bin/python3 /ws/install/usv_navigation/lib/usv_navigation/control_node',
])
def test_cleanup_matches_new_business_and_legacy_installed_entrypoints(command):
    # killpro uses grep -F: the production predicate is literal containment,
    # not an executable basename or a regular expression.
    assert any(pattern in command for pattern in cleanup_patterns()), command


@pytest.mark.parametrize('command', [
    '/usr/bin/python3 /ws/tools/control_node.py',
    '/other/install/custom_usv_control/lib/custom_usv_control/control_node',
    '/other/install/usv_control_backup/lib/usv_control/control_node',
    '/usr/bin/python3 /ws/scripts/distributed_yaml_probe.py',
])
def test_new_patterns_do_not_match_similarly_named_unrelated_commands(command):
    assert not any(pattern in command for pattern in cleanup_patterns()), command
