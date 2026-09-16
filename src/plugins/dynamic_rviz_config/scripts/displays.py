#!/usr/bin/python
# -*- coding: utf-8 -*-

""" 
******************************************************************************************
*  Copyright (C) 2023 Yang Haodong, All Rights Reserved                                  *
*                                                                                        *
*  @brief    rviz display pannel configure class.                                        *
*  @author   Haodong Yang                                                                *
*  @version  1.0.1                                                                       *
*  @date     2022.07.06                                                                  *
*  @license  GNU General Public License (GPL)                                            *
******************************************************************************************
"""


# 以列表子类构建 RViz 显示项；每项是插件类名、话题和显示属性组成的字典。
from dataclasses import fields


# 继承 list 保留普通列表的添加与嵌套能力，最终由专用 YAML representer 序列化。
class Displays(list):
    def __init__(self):
        None

    # 统一入口：设置名称、插件类和启用状态，合并可选话题、颜色及额外字段。
    def addDisplay(self, name, class_name, topic=None, color=None, fields={}, enabled=True):
        d = {"Name": name, "Class": class_name, "Enabled": enabled}
        if topic:
            d["Topic"] = topic
        if color:
            d["Color"] = "%d; %d; %d" % color
        d.update(fields)
        self.append(d)

    # 坐标轴绑定参考坐标系；length/radius 使用米，alpha 控制透明度。
    def addAxes(self, frame="map", name="Axes", length=5.0, radius=0.5, alpha=1.0):
        fields = {
            "Reference Frame": frame,
            "Length": length,
            "Radius": radius,
            "Alpha": alpha,
        }
        self.addDisplay(name, "rviz_default_plugins/Axes", fields=fields)

    # 配置栅格地图显示的话题、透明度和颜色方案；可选值通过真值判断决定是否写入。
    def addMap(self, topic="/map", name="Map", alpha=None, scheme=None):
        fields = {}
        if alpha:
            fields["Alpha"] = alpha
        if scheme:
            fields["Color Scheme"] = scheme
        self.addDisplay(name, "rviz_default_plugins/Map", topic, fields=fields)

    # 添加 RViz 默认网格，具体尺寸使用 RViz 插件默认值。
    def addGrid(self):
        self.addDisplay("Grid", "rviz_default_plugins/Grid")

    # 添加 TF 树显示并默认关闭，需要时可在界面勾选开启。
    def addTf(self, scale=None):
        fields = {}
        if scale:
            fields["Marker Scale"] = scale
        self.addDisplay("TF", "rviz_default_plugins/TF", fields=fields, enabled=False)

    # 把另一组 Displays 嵌套成面板分组，便于按船艇或数据类别整理。
    def addGroup(self, name, displays):
        self.addDisplay(name, "rviz_common/Group", fields={"Displays": displays})

    # 指定机器人模型描述来源，并可添加 TF 前缀以区分多艇坐标系。
    def addModel(self, parameter="robot_description", tf_prefix=None):
        fields = {"Robot Description": parameter}
        if tf_prefix:
            fields["TF Prefix"] = tf_prefix
        self.addDisplay("RobotModel", "rviz_default_plugins/RobotModel", fields=fields)

    # 配置 LaserScan 渲染，size 对应点的米制大小，color_transform 决定如何着色。
    def addLaserscan(self, topic="/base_scan", name="LaserScan", color=(46, 255, 0), size=0.1, alpha=None, color_transform="FlatColor"):
        if name is None:
            name = topic
        fields = {"Size (m)": size, "Style": "Flat Squares", "Color Transformer": color_transform}
        if alpha:
            fields["Alpha"] = alpha
        self.addDisplay(name, "rviz_default_plugins/LaserScan", topic, color, fields)

    # 配置三维点云渲染，使用平面方块样式并写入点尺寸和颜色变换器。
    def addPointCloud2(self, topic="/base_scan", name="PointCloud2", color=(46, 255, 0), size=0.1, alpha=None, color_transform="FlatColor"):
        if name is None:
            name = topic
        fields = {"Size (m)": size, "Style": "Flat Squares", "Color Transformer": color_transform}
        if alpha:
            fields["Alpha"] = alpha
        self.addDisplay(name, "rviz_default_plugins/PointCloud2", topic, color, fields)

    # 配置 PoseArray 显示，可用于查看定位粒子集合。
    def addPoseArray(self, topic="/particlecloud", color=None):
        self.addDisplay("AMCL Cloud", "rviz_default_plugins/PoseArray", topic, color)

    # 用 Polygon 显示机器人轮廓，接收的话题应与该显示类型兼容。
    def addFootprint(self, topic, color=(0, 170, 255)):
        self.addDisplay("Robot Footprint", "rviz_default_plugins/Polygon", topic, color)

    # 添加路径线，Billboards 样式才设置线宽，name 区分全局路径与局部轨迹。
    def addPath(self, topic, name, color=None, line_style="Billboards", line_width=0.5):
        fields = {"Line Style": line_style}
        if line_style == "Billboards":
            fields["Line Width"] = line_width
        self.addDisplay(name, "rviz_default_plugins/Path", topic, color, fields)

    # 显示单个位姿；arrow_shape 四项依次设置箭头头部长度/半径、杆部长度/半径。
    def addPose(self, topic, name="Current Goal", color=None, alpha=1.0, arrow_shape=None):
        fields = {"Alpha": alpha}
        if arrow_shape:
            fields["Head Length"] = arrow_shape[0]
            fields["Head Radius"] = arrow_shape[1]
            fields["Shaft Length"] = arrow_shape[2]
            fields["Shaft Radius"] = arrow_shape[3]
            fields["Shape"] = "Arrow"
        self.addDisplay(name, "rviz_default_plugins/Pose", topic, color, fields)

    # 添加图像显示并默认关闭，避免启动即打开额外图像面板。
    def addImage(self, topic="/camera/rgb/image_raw", name="Image"):
        self.addDisplay(name, "rviz_default_plugins/Image", topic, enabled=False)

    # Marker 插件使用 Marker Topic 字段，与普通显示项的 Topic 字段有所不同。
    def addMarker(self, topic, name="Marker"):
        fields = {"Marker Topic": topic}
        self.addDisplay(name, "rviz_default_plugins/Marker", fields=fields)

import yaml


# 将自定义 Displays 对象输出为标准 YAML sequence，避免写出 Python 专用类型标签。
def display_representer(dumper, data):
    return dumper.represent_sequence("tag:yaml.org,2002:seq", list(data))


yaml.add_representer(Displays, display_representer)
