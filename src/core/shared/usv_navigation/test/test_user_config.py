# 验证用户场景配置 user_config.yaml 的查找顺序与船型读取：环境变量、工作区内的仓库副本、主目录副本。
"""Lookup order and vessel selection of the optional user_config.yaml."""
from pathlib import Path

import pytest
import yaml

from usv_navigation import launch_support
from usv_navigation.launch_support import (
    ROBOT_TYPES, USER_CONFIG_ENVIRONMENT, load_user_config, user_config_path, user_robot_type)

REPOSITORY = Path(__file__).resolve().parents[5]


def write(path, robot_type=None, world=None):
    values = {}
    if world is not None:
        values["world"] = world
    if robot_type is not None:
        values["robots_config"] = [{"robot1_type": robot_type, "robot1_controller": "adrc"}]
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(yaml.safe_dump(values), encoding="utf-8")
    return path


# 把"本包所在位置"和主目录都挪到临时目录，测试不碰开发者真实的配置文件。
@pytest.fixture
def isolated(monkeypatch, tmp_path):
    workspace = tmp_path / "workspace"
    package = workspace / "install" / "usv_navigation" / "lib" / "python3.10" / "site-packages" / "usv_navigation"
    package.mkdir(parents=True)
    home = tmp_path / "home"
    home.mkdir()
    monkeypatch.delenv(USER_CONFIG_ENVIRONMENT, raising=False)
    monkeypatch.setattr(launch_support, "__file__", str(package / "launch_support.py"))
    monkeypatch.setenv("HOME", str(home))
    return workspace, home


# 三处都没有文件时不报错，船型回到 otter。
def test_missing_user_config_falls_back_to_otter(isolated):
    assert user_config_path() == ""
    assert load_user_config() == ("", {})
    assert user_robot_type(*load_user_config()) == "otter"


# 查找顺序：环境变量指定的文件优先，其次是安装区所在工作区里的仓库副本，最后是主目录副本。
def test_lookup_prefers_environment_then_workspace_then_home(isolated, monkeypatch, tmp_path):
    workspace, home = isolated
    in_home = write(home / ".rmp_user_config.yaml", "otter")
    assert user_config_path() == str(in_home)
    in_workspace = write(workspace / "src" / "user_config" / "user_config.yaml", "wamv", world="example_course")
    assert user_config_path() == str(in_workspace)
    path, values = load_user_config()
    assert (user_robot_type(path, values), values["world"]) == ("wamv", "example_course")
    explicit = write(tmp_path / "elsewhere" / "choice.yaml", "otter")
    monkeypatch.setenv(USER_CONFIG_ENVIRONMENT, str(explicit))
    assert user_config_path() == str(explicit)
    assert user_robot_type(*load_user_config()) == "otter"


# 环境变量指向不存在的文件时报错，而不是悄悄改用别的文件。
def test_explicit_missing_file_is_an_error(isolated, monkeypatch, tmp_path):
    workspace, _ = isolated
    write(workspace / "src" / "user_config" / "user_config.yaml", "wamv")
    monkeypatch.setenv(USER_CONFIG_ENVIRONMENT, str(tmp_path / "missing.yaml"))
    with pytest.raises(ValueError, match="does not exist"):
        user_config_path()


# 没写 robot1_type 或文件为空时是 otter；未登记的船型和非字典内容报错并指出文件。
def test_robot_type_parsing_and_validation(isolated, monkeypatch, tmp_path):
    path = write(tmp_path / "config.yaml", world="ocean")
    monkeypatch.setenv(USER_CONFIG_ENVIRONMENT, str(path))
    assert user_robot_type(*load_user_config()) == "otter"
    path.write_text("", encoding="utf-8")
    assert load_user_config() == (str(path), {})
    write(path, "catamaran")
    with pytest.raises(ValueError, match="config.yaml: robot1_type must be one of otter, wamv"):
        user_robot_type(*load_user_config())
    path.write_text("- just\n- a list\n", encoding="utf-8")
    with pytest.raises(ValueError, match="Expected a YAML mapping"):
        load_user_config()


# 仓库自带的 user_config.yaml 必须可读，船型是已登记的取值；本包从源码目录导入时能在工作区里找到它。
def test_repository_user_config_is_found_from_the_source_tree_and_valid(monkeypatch):
    monkeypatch.delenv(USER_CONFIG_ENVIRONMENT, raising=False)
    expected = REPOSITORY / "src" / "user_config" / "user_config.yaml"
    assert Path(user_config_path()) == expected
    assert user_robot_type(*load_user_config()) in ROBOT_TYPES
