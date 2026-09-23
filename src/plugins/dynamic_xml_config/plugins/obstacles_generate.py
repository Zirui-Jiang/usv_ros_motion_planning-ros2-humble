#!/usr/bin/python
# -*- coding: utf-8 -*-

""" 
******************************************************************************************
*  Copyright (C) 2023 Yang Haodong, All Rights Reserved                                  *
*                                                                                        *
*  @brief    generate launch file dynamicly based on user configure.                     *
*  @author   Haodong Yang                                                                *
*  @version  1.0.2                                                                       *
*  @date     2023.03.15                                                                  *
*  @license  GNU General Public License (GPL)                                            *
******************************************************************************************
"""
# 按用户障碍物 YAML 重建选定世界中的 BOX/CYLINDER/SPHERE 模型，并附加浮力插件。
import re
import xml.etree.ElementTree as ET

from .xml_generate import XMLGenerator

# 继承 XMLGenerator 复用配置路径和 XML 工具，负责障碍物读取、几何生成及旧模型清理。
class ObstacleGenerator(XMLGenerator):
    # 仅在 plugins.obstacles 配置有效时加载障碍物文件，否则仍可清理旧生成模型。
    def __init__(self) -> None:
        super().__init__()
        if "plugins" in self.user_cfg.keys() and "obstacles" in self.user_cfg["plugins"] and self.user_cfg["plugins"]["obstacles"]:
            self.obs_cfg = ObstacleGenerator.yamlParser(self.root_path + "user_config/" + self.user_cfg["plugins"]["obstacles"])
        else:
            self.obs_cfg = None

        self.box_obs, self.cylinder_obs, self.sphere_obs = [], [], []

    def __str__(self) -> str:
        return "Obstacles Generator"

    # 统一插件入口调用 spawn 更新世界文件；返回的注册列表在当前实现中为空。
    def plugin(self):
        """
        Implement of obstacles application.
        """
        app_register = []
        self.spawn()
        return app_register

    # 解析 sim_env/worlds 下的目标世界，先清理旧生成项，再追加当前配置并写回同一文件。
    def spawn(self):
        """
        Create configure file to construct obstacles environment.
        """
        world_file = self.root_path + "sim_env/worlds/" + self.user_cfg["world"] + ".world"
        tree = ET.parse(world_file)
        self.cleanObstacles(tree.getroot())
        
        if not self.obs_cfg is None:
            world = tree.getroot().find("world")
            index_map = {"BOX": 0, "CYLINDER": 0, "SPHERE": 0}
            # 键存在但没有列表项时，PyYAML 得到 None；空清单只清理旧模型。
            for obs in self.obs_cfg.get("obstacles") or []:
                params = {k : float(v) for k, v in obs["props"].items()}
                # pose 按六项解析为 x/y/z/roll/pitch/yaw；本版本生成时仅使用 x/y/z，转角未写入模型。
                pose = [float(i) for i in obs["pose"].split()]
                params["x"], params["y"], params["z"], params["r"], params["p"], params["yaw"] = pose
                params["color"] = obs["color"]
                world.append(self.createObstacleProto(obs["type"], index_map[obs["type"].upper()], **params))
                index_map[obs["type"].upper()] += 1

        with open(world_file, "wb+") as f:
            tree.write(f, encoding="utf-8", xml_declaration=True)

    # 按类型返回一个完整 SDF model；质量用 kg，几何长度和位置用 m，惯量用 kg·m²。
    def createObstacleProto(self, model_type: str, index: int, **kwargs) -> ET.Element:
        """
        Construct the specific type of obstacle.

        Parameters
        ----------
        model_type: str
            specific type of obstacle. Optional is `BOX`, `CYLINDER` and `SPHERE`
        index: int
            the index of obstacle
        kwargs: dict
            parameters of obstacle, such as mass, color, height, etc.

        Return
        ----------
        model: ET.Element
            model node
        """
        # parameters checking
        # 根据均匀实心体公式计算三个主惯量，非对角惯量统一写零；类型只支持箱体、圆柱、球。
        if model_type.upper() == "BOX":
            assert (
                "mass" in kwargs and "width" in kwargs and "depth" in kwargs and "height" in kwargs and "color" in kwargs
            ), f"Parameters of {model_type} are `mass`, `width`, `depth`, `height`, `color`"
            ixx = (kwargs["mass"] / 12.0) * (pow(kwargs["depth"], 2) + pow(kwargs["height"], 2))
            iyy = (kwargs["mass"] / 12.0) * (pow(kwargs["width"], 2) + pow(kwargs["height"], 2))
            izz = (kwargs["mass"] / 12.0) * (pow(kwargs["width"], 2) + pow(kwargs["depth"], 2))
        elif model_type.upper() == "CYLINDER":
            assert (
                "mass" in kwargs and "radius" in kwargs and "height" in kwargs and "color" in kwargs
            ), f"Parameters of {model_type} are `mass`, `radius`, `height`, `color`"
            ixx = (kwargs["mass"] / 12.0) * (3 * pow(kwargs["radius"], 2) + pow(kwargs["height"], 2))
            iyy = (kwargs["mass"] / 12.0) * (3 * pow(kwargs["radius"], 2) + pow(kwargs["height"], 2))
            izz = (kwargs["mass"] * pow(kwargs["radius"], 2)) / 2.0
        elif model_type.upper() == "SPHERE":
            assert (
                "mass" in kwargs and "radius" in kwargs and "color" in kwargs
            ), f"Parameters of {model_type} are `mass`, `radius`, `color`"
            ixx = (2 * kwargs["mass"] * pow(kwargs["radius"], 2)) / 5.0
            iyy = (2 * kwargs["mass"] * pow(kwargs["radius"], 2)) / 5.0
            izz = (2 * kwargs["mass"] * pow(kwargs["radius"], 2)) / 5.0

        obstacle_name = f"{model_type.upper()}_{index}"

        # inertia matrix
        # 惯性、碰撞和视觉元素共用所构造的位置偏移，根模型没有单独追加 pose。
        inertial = ObstacleGenerator.createElement("inertial")
        pose = ObstacleGenerator.createElement("pose", text=str(kwargs["x"]) + " " + str(kwargs["y"]) + " " + str(kwargs["z"]) + " 0 0 0")
        mass = ObstacleGenerator.createElement("mass", text=str(kwargs["mass"]))
        inertia = ObstacleGenerator.createElement("inertia")
        inertia.append(ObstacleGenerator.createElement("ixx", text=str(ixx)))
        inertia.append(ObstacleGenerator.createElement("ixy", text="0"))
        inertia.append(ObstacleGenerator.createElement("ixz", text="0"))
        inertia.append(ObstacleGenerator.createElement("iyy", text=str(iyy)))
        inertia.append(ObstacleGenerator.createElement("iyz", text="0"))
        inertia.append(ObstacleGenerator.createElement("izz", text=str(izz)))
        inertial.append(pose)
        inertial.append(mass)
        inertial.append(inertia)

        # collision
        collision = ObstacleGenerator.createElement("collision", props={"name": f"{obstacle_name}_link_collision"})
        geometry = ObstacleGenerator.createElement("geometry")
        if model_type.upper() == "BOX":
            box = ObstacleGenerator.createElement("box")
            size = ObstacleGenerator.createElement("size", text=str(kwargs["width"]) + " " + str(kwargs["depth"]) + " " + str(kwargs["height"]))
            box.append(size)
            geometry.append(box)
        elif model_type.upper() == "CYLINDER":
            cylinder = ObstacleGenerator.createElement("cylinder")
            cylinder.append(ObstacleGenerator.createElement("radius", text=str(kwargs["radius"])))
            cylinder.append(ObstacleGenerator.createElement("length", text=str(kwargs["height"])))
            geometry.append(cylinder)   
        elif model_type.upper() == "SPHERE":
            sphere = ObstacleGenerator.createElement("sphere")
            sphere.append(ObstacleGenerator.createElement("radius", text=str(kwargs["radius"])))
            geometry.append(sphere) 

        # 设置 ODE 接触刚度、阻尼与摩擦，决定障碍物发生接触时的物理响应。
        surface = ObstacleGenerator.createElement("surface")
        contact = ObstacleGenerator.createElement("contact")
        ode1 = ObstacleGenerator.createElement("ode")
        ode1.append(ObstacleGenerator.createElement("kp", text="100000"))
        ode1.append(ObstacleGenerator.createElement("kd", text="100000"))
        contact.append(ode1)
        friction = ObstacleGenerator.createElement("friction")
        torsional = ObstacleGenerator.createElement("torsional")
        ode2 = ObstacleGenerator.createElement("ode")
        ode2.append(ObstacleGenerator.createElement("mu", text="10"))
        ode2.append(ObstacleGenerator.createElement("mu2", text="10"))
        torsional.append(ObstacleGenerator.createElement("ode"))
        friction.append(ode2)
        friction.append(torsional)
        surface.append(contact)
        surface.append(friction)
        surface.append(ObstacleGenerator.createElement("bounce"))

        collision.append(pose)
        collision.append(geometry)
        collision.append(surface)
        collision.append(ObstacleGenerator.createElement("max_contacts", text="10"))

        # visual
        # 视觉几何沿用碰撞几何，颜色引用 Gazebo 材质脚本中的命名材质。
        visual = ObstacleGenerator.createElement("visual", props={"name": f"{obstacle_name}_link_visual"})
        material = ObstacleGenerator.createElement("material")
        script = ObstacleGenerator.createElement("script")
        script.append(ObstacleGenerator.createElement("name", text="Gazebo/" + kwargs["color"]))
        script.append(ObstacleGenerator.createElement("uri", text="file://media/materials/scripts/gazebo.material"))
        material.append(script)
        visual.append(pose)
        visual.append(geometry)
        visual.append(material)

        link = ObstacleGenerator.createElement("link", props={"name": f"{obstacle_name}_link"})
        link.append(inertial)
        link.append(collision)
        link.append(visual)
        link.append(ObstacleGenerator.createElement("self_collide", text="0"))
        link.append(ObstacleGenerator.createElement("enable_wind", text="0"))
        link.append(ObstacleGenerator.createElement("kinematic", text="0"))
        
        # plugin
        # 为生成障碍物挂接浮力插件，波场固定指向 ocean_waves，平均水位固定为零。
        plugin = ObstacleGenerator.createElement("plugin", props={"name": "BuoyancyPlugin", "filename": "libbuoyancy_gazebo_plugin.so"})
        plugin.append(ObstacleGenerator.createElement("wave_model", text="ocean_waves"))
        plugin.append(ObstacleGenerator.createElement("fluid_density", text="1000"))
        plugin.append(ObstacleGenerator.createElement("fluid_level", text="0"))
        plugin.append(ObstacleGenerator.createElement("linear_drag", text="1000.0"))
        plugin.append(ObstacleGenerator.createElement("angular_drag", text="200.0"))
        buoyancy = ObstacleGenerator.createElement("buoyancy", props={"name": "buoyancy_base"})
        buoyancy.append(ObstacleGenerator.createElement("link_name", text=f"{obstacle_name}_link"))
        buoyancy.append(ObstacleGenerator.createElement("pose", text=str(kwargs["x"]) + " " + str(kwargs["y"]) + " " + str(kwargs["z"]) + " 0 0 0"))
        # 当前排水几何统一采用固定半径球体，不随障碍物碰撞形状或尺寸同步变化。
        buoyancy_sphere = ObstacleGenerator.createElement("sphere")
        buoyancy_sphere.append(ObstacleGenerator.createElement("radius", text="0.28794119114848615"))
        buoyancy_geometry = ObstacleGenerator.createElement("geometry")
        buoyancy_geometry.append(buoyancy_sphere)
        buoyancy.append(buoyancy_geometry)
        plugin.append(buoyancy)
        
        model = ObstacleGenerator.createElement("model", props={"name": obstacle_name})
        model.append(link)
        model.append(plugin)

        ObstacleGenerator.indent(model)

        return model

    # 只清理由本生成器命名的三类编号模型，保留世界中的其它模型；先收集再删除避免边遍历边移除。
    def cleanObstacles(self, root_node: ET.Element) -> None:
        """
        Clean up the obstacles.
        """
        world = root_node.find("world")
        pattern = r'^(BOX_\d+|CYLINDER_\d+|SPHERE_\d+)$'
        
        remove_models = []
        for model in world:
            if "name" in model.attrib.keys() and re.match(pattern, model.attrib["name"]):
                remove_models.append(model)
        for model in remove_models:
            world.remove(model)