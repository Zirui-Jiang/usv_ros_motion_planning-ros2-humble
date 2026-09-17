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


from dataclasses import fields


class Displays(list):
    def __init__(self):
        None

    def addDisplay(self, name, class_name, topic=None, color=None, fields={}, enabled=True):
        d = {"Name": name, "Class": class_name, "Enabled": enabled}
        if topic:
            d["Topic"] = topic
        if color:
            d["Color"] = "%d; %d; %d" % color
        d.update(fields)
        self.append(d)

    def addAxes(self, frame="map", name="Axes", length=5.0, radius=0.5, alpha=1.0):
        fields = {
            "Reference Frame": frame,
            "Length": length,
            "Radius": radius,
            "Alpha": alpha,
        }
        self.addDisplay(name, "rviz_default_plugins/Axes", fields=fields)

    def addMap(self, topic="/map", name="Map", alpha=None, scheme=None):
        fields = {}
        if alpha:
            fields["Alpha"] = alpha
        if scheme:
            fields["Color Scheme"] = scheme
        self.addDisplay(name, "rviz_default_plugins/Map", topic, fields=fields)

    def addGrid(self):
        self.addDisplay("Grid", "rviz_default_plugins/Grid")

    def addTf(self, scale=None):
        fields = {}
        if scale:
            fields["Marker Scale"] = scale
        self.addDisplay("TF", "rviz_default_plugins/TF", fields=fields, enabled=False)

    def addGroup(self, name, displays):
        self.addDisplay(name, "rviz_common/Group", fields={"Displays": displays})

    def addModel(self, parameter="robot_description", tf_prefix=None):
        fields = {"Robot Description": parameter}
        if tf_prefix:
            fields["TF Prefix"] = tf_prefix
        self.addDisplay("RobotModel", "rviz_default_plugins/RobotModel", fields=fields)

    def addLaserscan(self, topic="/base_scan", name="LaserScan", color=(46, 255, 0), size=0.1, alpha=None, color_transform="FlatColor"):
        if name is None:
            name = topic
        fields = {"Size (m)": size, "Style": "Flat Squares", "Color Transformer": color_transform}
        if alpha:
            fields["Alpha"] = alpha
        self.addDisplay(name, "rviz_default_plugins/LaserScan", topic, color, fields)

    def addPointCloud2(self, topic="/base_scan", name="PointCloud2", color=(46, 255, 0), size=0.1, alpha=None, color_transform="FlatColor"):
        if name is None:
            name = topic
        fields = {"Size (m)": size, "Style": "Flat Squares", "Color Transformer": color_transform}
        if alpha:
            fields["Alpha"] = alpha
        self.addDisplay(name, "rviz_default_plugins/PointCloud2", topic, color, fields)

    def addPoseArray(self, topic="/particlecloud", color=None):
        self.addDisplay("AMCL Cloud", "rviz_default_plugins/PoseArray", topic, color)

    def addFootprint(self, topic, color=(0, 170, 255)):
        self.addDisplay("Robot Footprint", "rviz_default_plugins/Polygon", topic, color)

    def addPath(self, topic, name, color=None, line_style="Billboards", line_width=0.5):
        fields = {"Line Style": line_style}
        if line_style == "Billboards":
            fields["Line Width"] = line_width
        self.addDisplay(name, "rviz_default_plugins/Path", topic, color, fields)

    def addPose(self, topic, name="Current Goal", color=None, alpha=1.0, arrow_shape=None):
        fields = {"Alpha": alpha}
        if arrow_shape:
            fields["Head Length"] = arrow_shape[0]
            fields["Head Radius"] = arrow_shape[1]
            fields["Shaft Length"] = arrow_shape[2]
            fields["Shaft Radius"] = arrow_shape[3]
            fields["Shape"] = "Arrow"
        self.addDisplay(name, "rviz_default_plugins/Pose", topic, color, fields)

    def addImage(self, topic="/camera/rgb/image_raw", name="Image"):
        self.addDisplay(name, "rviz_default_plugins/Image", topic, enabled=False)

    def addMarker(self, topic, name="Marker"):
        fields = {"Marker Topic": topic}
        self.addDisplay(name, "rviz_default_plugins/Marker", fields=fields)

import yaml


def display_representer(dumper, data):
    return dumper.represent_sequence("tag:yaml.org,2002:seq", list(data))


yaml.add_representer(Displays, display_representer)
