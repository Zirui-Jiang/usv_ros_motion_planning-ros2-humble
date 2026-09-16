# 用临时 Bash 环境替代外部安装区和 ros2 命令，验证路径选择、带空格参数与失败传播。
"""Run the real Bash launcher against temporary setup/ros2 boundaries only."""
import json
import os
from pathlib import Path
import shlex
import shutil
import subprocess

import pytest


SOURCE = Path(__file__).resolve().parents[1] / 'main.sh'


@pytest.fixture
def launcher(tmp_path):
    if os.name != 'posix' or not shutil.which('bash'):
        pytest.skip('The launcher black-box test requires Linux/WSL Bash')
    project = tmp_path / 'project workspace'
    script = project / 'scripts/main.sh'
    script.parent.mkdir(parents=True)
    shutil.copyfile(SOURCE, script)
    tools = tmp_path / 'fake-bin'
    tools.mkdir()
    record = tmp_path / 'ros2-call.json'
    fake_ros2 = tools / 'ros2'
    fake_ros2.write_text(
        '#!/usr/bin/python3\n'
        'import json, os, pathlib, sys\n'
        'pathlib.Path(os.environ["TEST_ROS2_RECORD"]).write_text(json.dumps({\n'
        '    "argv": sys.argv[1:], "cwd": os.getcwd(),\n'
        '    "install": os.environ.get("TEST_SELECTED_INSTALL")}))\n')
    fake_ros2.chmod(0o755)
    fake_ros_setup = tmp_path / 'ros-setup.bash'
    fake_ros_setup.write_text(': "$TEST_OPTIONAL_UNSET"\nexport TEST_ROS_SOURCED=yes\n')
    fake_gazebo_setup = tmp_path / 'gazebo-setup.bash'
    fake_gazebo_setup.write_text(': "$TEST_OPTIONAL_UNSET"\n')
    boundary = tmp_path / 'bash-boundaries.bash'
    # BASH_ENV substitutes only external absolute setup sources. The copied
    # launcher and selected workspace setup execute unchanged in real Bash.
    # No real ROS setup can replace our fake ros2 on PATH.
    boundary.write_text('''source() {
    case "$1" in
        /opt/ros/humble/setup.bash) builtin source "$TEST_ROS_SETUP" ;;
        /usr/share/gazebo/setup.sh) builtin source "$TEST_GAZEBO_SETUP" ;;
        *) builtin source "$@" ;;
    esac
}
''')
    environment = os.environ.copy()
    for name in ('USV_INSTALL_DIR', 'TEST_OPTIONAL_UNSET', 'SHELLOPTS', 'BASHOPTS'):
        environment.pop(name, None)
    environment.update(BASH_ENV=str(boundary), PATH=str(tools) + ':/usr/bin:/bin',
                       TEST_ROS_SETUP=str(fake_ros_setup), TEST_GAZEBO_SETUP=str(fake_gazebo_setup),
                       TEST_ROS2_RECORD=str(record))

    def install(path, contents=None):
        path.mkdir(parents=True)
        (path / 'setup.bash').write_text(contents if contents is not None else (
            ': "$TEST_OPTIONAL_UNSET"\n'
            'export TEST_SELECTED_INSTALL=' + shlex.quote(str(path)) + '\n'))
        return path

    def run(selected=None, arguments=()):
        env = dict(environment)
        if selected is not None:
            env['USV_INSTALL_DIR'] = str(selected)
        result = subprocess.run(['bash', str(script), *arguments], cwd=tmp_path,
                                env=env, capture_output=True, text=True, timeout=10)
        observed = json.loads(record.read_text()) if record.exists() else None
        return result, observed

    return project, install, run


# 显式指定外部安装区时，不应误用项目内已有 install。
def test_explicit_install_selection_overrides_existing_project_install(launcher, tmp_path):
    project, install, run = launcher
    install(project / 'install')
    selected = install(tmp_path / 'separate-install')
    result, observed = run(selected)
    assert result.returncode == 0, result.stderr
    assert observed['install'] == str(selected)


# 没有指定安装区时使用项目默认 install。
def test_unset_install_selection_defaults_to_project_install(launcher):
    project, install, run = launcher
    selected = install(project / 'install')
    result, observed = run()
    assert result.returncode == 0, result.stderr
    assert observed == {'argv': ['launch', 'sim_env', 'main.launch.py'],
                        'cwd': str(project), 'install': str(selected)}


# 选定安装区不存在时立即失败，并指出具体缺失路径。
@pytest.mark.parametrize('explicit', [False, True])
def test_missing_selected_setup_fails_with_path_and_override_instructions(launcher, tmp_path, explicit):
    project, install, run = launcher
    missing = tmp_path / 'missing external install' if explicit else project / 'install'
    if explicit:
        # An explicit invalid path must not silently fall back to an old build.
        install(project / 'install')
    result, observed = run(missing if explicit else None)
    assert result.returncode != 0
    assert observed is None
    assert str(missing / 'setup.bash') in result.stderr
    assert 'USV_INSTALL_DIR' in result.stderr


# 安装路径含空格时保持一个完整参数，不拆分加载。
def test_explicit_install_path_with_spaces_is_sourced_intact(launcher, tmp_path):
    _, install, run = launcher
    selected = install(tmp_path / 'installation with spaces')
    result, observed = run(selected)
    assert result.returncode == 0, result.stderr
    assert observed['install'] == str(selected)


# 启动参数原样传递，不做额外拆词或执行参数中的 shell 文本。
def test_launch_arguments_are_forwarded_without_splitting_or_evaluation(launcher):
    project, install, run = launcher
    install(project / 'install')
    arguments = ['headless:=true', 'robot_namespace:=boat two',
                 'controller_config:=/config/with spaces/controller.yaml',
                 'literal:=$(printf must-not-run)', '', 'backslash:=a\\b']
    result, observed = run(arguments=arguments)
    assert result.returncode == 0, result.stderr
    assert observed['argv'] == ['launch', 'sim_env', 'main.launch.py', *arguments]


# 环境加载失败后不得继续启动 ros2。
def test_setup_failure_stops_before_ros2_launch(launcher):
    project, install, run = launcher
    install(project / 'install', 'return 23\n')
    result, observed = run()
    assert result.returncode == 23
    assert observed is None
