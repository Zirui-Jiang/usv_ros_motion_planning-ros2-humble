"""Static checks for the VRX-inspired Station Keeping disturbance world.

The test deliberately parses the SDF instead of starting Gazebo.  This keeps the
configuration regression check fast and makes sure the physical and visual wave
plugins cannot silently drift apart.
"""

from pathlib import Path
import math
import xml.etree.ElementTree as ET


WORLD = Path(__file__).resolve().parents[1] / "worlds" / "stationkeeping_vrx.world"


def _text(element: ET.Element, name: str) -> str:
    child = element.find(name)
    assert child is not None, f"missing <{name}> in {element.tag}"
    assert child.text is not None, f"empty <{name}> in {element.tag}"
    return child.text.strip()


def _plugins(root: ET.Element, filename: str):
    return [plugin for plugin in root.iter("plugin")
            if plugin.get("filename") == filename]


def _wave_signature(wave: ET.Element) -> dict[str, object]:
    direction = tuple(float(value) for value in _text(wave, "direction").split())
    return {
        "model": _text(wave, "model"),
        "period": float(_text(wave, "period")),
        "number": int(_text(wave, "number")),
        "scale": float(_text(wave, "scale")),
        "gain": float(_text(wave, "gain")),
        "direction": direction,
        "angle": float(_text(wave, "angle")),
        "tau": float(_text(wave, "tau")),
        "amplitude": float(_text(wave, "amplitude")),
        "steepness": float(_text(wave, "steepness")),
    }


def test_stationkeeping_world_declares_otter_and_vrx_practice_wind():
    root = ET.parse(WORLD).getroot()
    assert root.tag == "sdf"
    world = root.find("world")
    assert world is not None
    assert world.get("name") == "stationkeeping_vrx"

    wind_plugins = _plugins(root, "libusv_gazebo_wind_plugin.so")
    assert len(wind_plugins) == 1
    wind = wind_plugins[0]
    wind_obj = wind.find("wind_obj")
    assert wind_obj is not None
    assert _text(wind_obj, "name") == "otter"
    # Gazebo Classic Model::GetLink() receives the model-local link name.
    # The Otter URDF declares <link name="base_link">; the scoped WAM-V
    # spelling from the official Gazebo Sim world is not valid here.
    assert _text(wind_obj, "link_name") == "base_link"
    assert math.isclose(float(_text(wind, "wind_direction")), 120.0)
    assert math.isclose(float(_text(wind, "wind_mean_velocity")), 5.0)
    assert math.isclose(float(_text(wind, "var_wind_gain_constants")), 2.0)
    assert math.isclose(float(_text(wind, "var_wind_time_constants")), 2.0)
    assert int(_text(wind, "random_seed")) == 15
    assert math.isclose(float(_text(wind, "update_rate")), 10.0)


def test_stationkeeping_world_keeps_physical_and_visual_wave_parameters_equal():
    root = ET.parse(WORLD).getroot()
    physical = _plugins(root, "libWavefieldModelPlugin.so")
    visual = _plugins(root, "libWavefieldVisualPlugin.so")
    assert len(physical) == 1
    assert len(visual) == 1

    physical_wave = physical[0].find("wave")
    visual_wave = visual[0].find("wave")
    assert physical_wave is not None
    assert visual_wave is not None
    assert _wave_signature(physical_wave) == _wave_signature(visual_wave)

    signature = _wave_signature(physical_wave)
    assert signature["model"] == "PMS"
    assert math.isclose(signature["period"], 6.0)
    assert math.isclose(signature["gain"], 0.6)
    direction = signature["direction"]
    assert isinstance(direction, tuple) and len(direction) == 2
    assert all(math.isclose(value, 0.70710678, rel_tol=0.0, abs_tol=1e-7)
               for value in direction)


def test_stationkeeping_world_references_the_wave_model_for_buoyancy():
    root = ET.parse(WORLD).getroot()
    wave_models = [model for model in root.iter("model") if model.get("name") == "ocean_waves"]
    assert len(wave_models) == 1
    buoyancy_plugins = _plugins(root, "libbuoyancy_gazebo_plugin.so")
    assert buoyancy_plugins
    assert all(_text(plugin, "wave_model") == "ocean_waves" for plugin in buoyancy_plugins)
