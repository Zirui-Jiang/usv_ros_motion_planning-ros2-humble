# 检查 WAM-V 仿真资产与导航配置相互一致：网格存在、推力映射线性、最大推力与船模 YAML 相同、双艉推布局的几何与船模参数一致。
"""Static consistency checks between the WAM-V URDF assets and the navigation model."""
from pathlib import Path
import re
import xml.etree.ElementTree as ET

import yaml

PACKAGE = Path(__file__).resolve().parents[1]
URDF = PACKAGE / "urdf" / "wamv"
SOURCE = PACKAGE.parent
DYNAMICS = SOURCE / "core/shared/usv_navigation/config/robots/wamv/dynamics_wamv.yaml"
XACRO = "{http://ros.org/wiki/xacro}"


# 默认启动用到的 xacro（艇体、发动机、电池、机箱、GPS 外观）引用的网格都必须随包安装。
def test_meshes_used_by_default_wamv_description_exist():
    missing = []
    for name in ("wamv_base.urdf.xacro", "thrusters/engine.xacro", "battery.xacro", "cpu_cases.xacro",
                 "components/wamv_gps.xacro"):
        for reference in re.findall(r'package://sim_env/([^"]+)', (URDF / name).read_text(encoding="utf-8")):
            if not (PACKAGE / reference).is_file():
                missing.append(f"{name}: {reference}")
    assert not missing, "\n".join(missing)


# 艇体与发动机网格引用的纹理须可读，否则 RViz 载入船模时找不到材质。
def test_wamv_texture_references_are_readable():
    namespace = {"c": "http://www.collada.org/2005/11/COLLADASchema"}
    missing = []
    for mesh in sorted((PACKAGE / "meshes" / "wamv").glob("*/mesh/*.dae")):
        for entry in ET.parse(mesh).findall("c:library_images/c:image/c:init_from", namespace):
            if not (mesh.parent / entry.text.strip()).is_file():
                missing.append(f"{mesh.name}: {entry.text.strip()}")
    assert not missing, "\n".join(missing)


# 导航侧按 推力 = 命令 × max_force 建模：插件须用线性映射，且正反向最大力等于船模 YAML。
def test_thruster_mapping_is_linear_and_matches_navigation_model():
    macro = ET.parse(URDF / "thruster_layouts" / "wamv_gazebo_thruster_config.xacro").getroot().find(XACRO + "macro")
    thruster = macro.find("wamv_thruster")
    dynamics = yaml.safe_load(DYNAMICS.read_text(encoding="utf-8"))["dynamics"]
    assert thruster.findtext("mappingType") == "0"
    assert float(thruster.findtext("maxCmd")) == 1.0
    assert float(thruster.findtext("maxForceFwd")) == dynamics["max_force"]
    assert float(thruster.findtext("maxForceRev")) == -dynamics["max_force"]
    # 航控发布的归一化推力和桨角话题名；全回转需要桨角可控，仿真限位 ±π/2 须不小于航控的最大桨角。
    assert thruster.findtext("cmdTopic") == "${thruster_namespace}${name}_thrust_cmd"
    assert thruster.findtext("angleTopic") == "${thruster_namespace}${name}_thrust_angle"
    assert thruster.findtext("enableAngle") == "${enable_angle}" and macro.get("params").endswith("enable_angle:=1")
    assert thruster.findtext("maxAngle") == "${pi/2}"


# 双艉推布局（全回转和固定桨差动共用）：两台艉推对称于中心线，横向力臂等于 dynamics.D，回转轴纵向位置等于 dynamics.thruster_x。
def test_aft_layout_geometry_matches_navigation_model():
    root = ET.parse(URDF / "thruster_layouts" / "wamv_aft_thrusters.xacro").getroot()
    engines = {engine.get("prefix"): [float(value) for value in engine.get("position").split()]
               for engine in root.findall(XACRO + "engine")}
    assert set(engines) == {"left", "right"}
    dynamics = yaml.safe_load(DYNAMICS.read_text(encoding="utf-8"))["dynamics"]
    assert dynamics["type"] == "Azimuth"
    assert engines["left"][0] == engines["right"][0] and engines["left"][1] == -engines["right"][1] > 0
    assert abs(engines["left"][1] - dynamics["D"]) < 1e-3
    assert abs(engines["left"][0] - dynamics["thruster_x"]) < 1e-3
    plugin = root.find("gazebo/plugin")
    assert plugin.get("filename") == "libwamv_usv_gazebo_thrust_plugin.so"
    assert [config.get("name") for config in plugin.findall(XACRO + "wamv_gazebo_thruster_config")] == ["left", "right"]


# T 型布局（Omni 推进形式用）：两台艉推对称于中心线且力臂等于 dynamics.D，第三台在 x = 0 处横置、正命令推向左舷。
def test_t_layout_has_symmetric_aft_thrusters_and_a_port_pointing_lateral_thruster():
    root = ET.parse(URDF / "thruster_layouts" / "wamv_t_thrusters.xacro").getroot()
    engines = {engine.get("prefix"): engine for engine in root.findall(XACRO + "engine")}
    assert set(engines) == {"left", "right", "lateral"}
    left = [float(value) for value in engines["left"].get("position").split()]
    right = [float(value) for value in engines["right"].get("position").split()]
    assert left[0] == right[0] and left[1] == -right[1] > 0
    arm = yaml.safe_load(DYNAMICS.read_text(encoding="utf-8"))["dynamics"]["D"]
    assert abs(left[1] - arm) < 1e-3
    lateral = engines["lateral"]
    assert [float(value) for value in lateral.get("position").split()][:2] == [0.0, 0.0]
    # 绕 z 轴转 +90°：发动机推力轴指向艇体系 +y（左舷）。
    assert abs(float(lateral.get("orientation").split()[2]) - 1.5707) < 1e-3
    plugin = root.find("gazebo/plugin")
    assert plugin.get("filename") == "libwamv_usv_gazebo_thrust_plugin.so"
    names = [config.get("name") for config in plugin.findall(XACRO + "wamv_gazebo_thruster_config")]
    assert names == ["left", "right", "lateral"]


# 激光最小量程须盖过扫描高度上的自身结构：GPS 天线远端距激光 0.3 + 0.85 + 0.15 = 1.30 米，否则自身回波会被标成障碍。
def test_lidar_minimum_range_excludes_own_structure():
    root = ET.parse(URDF / "wamv.urdf.xacro").getroot()
    lidar = next(element for element in root.iter(XACRO + "wamv_multibeam_laser"))
    gps = next(element for element in root.iter(XACRO + "wamv_gps"))
    gps_collision_radius = 0.15
    farthest_own_return = float(lidar.get("x")) - float(gps.get("x")) + gps_collision_radius
    assert abs(farthest_own_return - 1.30) < 1e-9
    # 至少留 0.15 米余量，盖过同一高度上略远的后支柱。
    assert float(lidar.get("min_range")) >= farthest_own_return + 0.15 - 1e-9


# 船模入口按 thruster_config 选择布局：launch 对固定桨和全回转传 H，对独立侧推器传 T。
def test_vessel_description_offers_the_layouts_launch_selects():
    text = (URDF / "wamv.urdf.xacro").read_text(encoding="utf-8")
    assert "thruster_conf == 'H'" in text and "wamv_aft_thrusters.xacro" in text
    assert "thruster_conf == 'T'" in text and "wamv_t_thrusters.xacro" in text
