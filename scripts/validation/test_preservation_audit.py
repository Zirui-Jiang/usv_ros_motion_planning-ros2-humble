# 用有意变更的临时源码验证保留审计：移动目录可接受，算法常量、默认值和函数体变化必须被发现。
"""Audit guards must detect behavior/configuration drift across directory moves."""
import importlib.util
from pathlib import Path

import pytest


def module(path):
    spec = importlib.util.spec_from_file_location('preservation_test_target', path)
    loaded = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(loaded)
    return loaded


AUDIT = module(Path(__file__).with_name('verify_navigation_preservation.py'))
PARITY = module(Path(__file__).parent / 'parity/run.py')


@pytest.mark.parametrize('discover', [AUDIT.packages, PARITY.package_paths])
def test_package_discovery_accepts_old_new_and_exported_layouts(tmp_path, discover):
    for relative, name in (
        ('src/core/controller/controller', 'controller'),
        ('src/core/decision_module/path_planner', 'path_planner'),
        ('src/common', 'common'),
    ):
        root = tmp_path / relative
        root.mkdir(parents=True)
        (root / 'package.xml').write_text(f'<package><name>{name}</name></package>')
    actual = discover(tmp_path)
    assert actual['controller'] == tmp_path / 'src/core/controller/controller'
    assert actual['path_planner'] == tmp_path / 'src/core/decision_module/path_planner'
    assert actual['common'] == tmp_path / 'src/common'


@pytest.mark.parametrize('discover', [AUDIT.packages, PARITY.package_paths])
def test_duplicate_package_is_rejected_instead_of_selecting_one(tmp_path, discover):
    for relative in ('old', 'new'):
        package = tmp_path / 'src' / relative
        package.mkdir(parents=True)
        (package / 'package.xml').write_text('<package><name>common</name></package>')
    with pytest.raises(ValueError, match='duplicate'):
        discover(tmp_path)


# 配置审计必须比较全部语句，旧传感器键或新增常量也不能被忽略。
@pytest.mark.parametrize('replacement', [
    "DEFAULTS = {'navigation.radar_timeout': 1.0}\n",
    "DEFAULTS = {}\n",
    "DEFAULTS = {'navigation.radar_timeout': 0.5}\nEXTRA = 1\n",
])
def test_complete_configuration_ast_does_not_exempt_radar_or_new_statements(tmp_path, replacement):
    before, after = tmp_path / 'before.py', tmp_path / 'after.py'
    before.write_text("DEFAULTS = {'navigation.radar_timeout': 0.5}\n")
    after.write_text(replacement)
    assert AUDIT.configuration_ast(before) != AUDIT.configuration_ast(after)


# 分别改变常量、默认参数和函数体，审计都应发现差异。
@pytest.mark.parametrize('replacement', [
    'TIMEOUT = 2.0\ndef compute(dt=0.1):\n    return dt\n',
    'TIMEOUT = 1.0\ndef compute(dt=0.2):\n    return dt\n',
    'TIMEOUT = 1.0\ndef compute(dt=0.1):\n    return -dt\n',
])
def test_definition_audit_detects_constants_defaults_and_function_body(tmp_path, replacement):
    before, after = tmp_path / 'before.py', tmp_path / 'after.py'
    before.write_text('TIMEOUT = 1.0\ndef compute(dt=0.1):\n    return dt\n')
    after.write_text(replacement)
    assert AUDIT.definitions(before) != AUDIT.definitions(after)


# 只调整导入位置而定义保持一致时，定义级比较应保持通过。
def test_definition_audit_allows_only_module_import_relocation(tmp_path):
    before, after = tmp_path / 'before.py', tmp_path / 'after.py'
    body = '\ndef compute(dt=0.1):\n    return dt\n'
    before.write_text('from old_package import Model\n' + body)
    after.write_text('from new_package import Model\n' + body)
    assert AUDIT.definitions(before) == AUDIT.definitions(after)
