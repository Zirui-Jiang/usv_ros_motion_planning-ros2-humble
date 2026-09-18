"""Regression tests for selecting the scenario-specific Otter wind plugin."""

import importlib.util
from pathlib import Path
import re

import pytest


MODULE_PATH = Path(__file__).resolve().parents[1] / "sim_env" / "wind_profiles.py"
assert MODULE_PATH.is_file(), f"missing wind profile helper: {MODULE_PATH}"
_SPEC = importlib.util.spec_from_file_location("wind_profiles", MODULE_PATH)
wind_profiles = importlib.util.module_from_spec(_SPEC)
_SPEC.loader.exec_module(wind_profiles)
XACRO_PATH = MODULE_PATH.parent.parent / "urdf" / "otter" / "otter.urdf.xacro"
WIND_XACRO_PATH = XACRO_PATH.parent / "dynamics" / "otter_gazebo_wind_plugin.xacro"
WIND_PLUGIN_PATH = (MODULE_PATH.parents[2] / "plugins" / "gazebo_plugins" /
                    "usv_gazebo_plugins" / "src" / "usv_gazebo_wind_plugin.cc")


def test_stationkeeping_world_disables_embedded_default_wind_plugin():
    assert wind_profiles.xacro_wind_arguments("stationkeeping_vrx", "otter") == (
        "wind_enabled:=false",
    )


def test_regular_worlds_keep_the_legacy_xacro_wind_plugin():
    assert wind_profiles.xacro_wind_arguments("ocean", "boat_two") == ()
    assert wind_profiles.xacro_wind_arguments("example_course.world", "boat_two") == ()


def test_stationkeeping_world_rejects_a_model_name_the_static_wind_plugin_cannot_find():
    with pytest.raises(ValueError, match="requires robot_namespace=otter"):
        wind_profiles.xacro_wind_arguments("stationkeeping_vrx", "boat_two")


def test_world_name_matching_ignores_path_and_suffix():
    assert wind_profiles.is_stationkeeping_world(
        "/tmp/share/worlds/stationkeeping_vrx.world"
    )
    assert not wind_profiles.is_stationkeeping_world("stationkeeping_vrx_backup")


def test_otter_xacro_exposes_a_default_preserving_wind_gate():
    text = XACRO_PATH.read_text(encoding="utf-8")
    assert re.search(r'<xacro:arg\s+name="wind_enabled"\s+default="true"', text)
    assert '<xacro:if value="$(arg wind_enabled)">' in text


def test_embedded_classic_wind_uses_the_otter_local_link_name():
    text = WIND_XACRO_PATH.read_text(encoding="utf-8")
    assert "<name>otter</name>" in text
    assert "<link_name>base_link</link_name>" in text
    assert "<link_name>otter/base_link</link_name>" not in text


def test_classic_wind_plugin_guards_zero_or_invalid_first_step():
    """The random-wind update must not turn 0 * inf into a persistent NaN."""

    text = WIND_PLUGIN_PATH.read_text(encoding="utf-8")
    assert ".Double() - 0.001" in text
    assert "dT > 0.0" in text
    assert "std::isfinite(dT)" in text


def test_both_launch_stacks_forward_the_stationkeeping_wind_override():
    root = MODULE_PATH.parent.parent.parent
    sim_support = (root / "sim_env" / "sim_env" / "launch_support.py").read_text(encoding="utf-8")
    mission_support = (root / "core" / "mission_module" / "usv_mission" /
                       "usv_mission" / "launch_support.py").read_text(encoding="utf-8")
    assert "xacro_wind_arguments" in sim_support
    assert "wind_enabled:=false" in mission_support
    assert "from sim_env.wind_profiles" not in mission_support

    spec = importlib.util.spec_from_file_location("mission_wind_launch", root / "core" /
                                                  "mission_module" / "usv_mission" /
                                                  "usv_mission" / "launch_support.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    assert module._stationkeeping_wind_arguments("stationkeeping_vrx", "otter") == (
        "wind_enabled:=false",
    )
    assert module._stationkeeping_wind_arguments("ocean.world", "boat_two") == ()
    with pytest.raises(ValueError, match="requires robot_namespace=otter"):
        module._stationkeeping_wind_arguments("stationkeeping_vrx", "boat_two")
