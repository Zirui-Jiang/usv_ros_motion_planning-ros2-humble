#!/usr/bin/python
# -*- coding: utf-8 -*-

""" 
******************************************************************************************
*  Copyright (C) 2023 Yang Haodong, All Rights Reserved                                  *
*                                                                                        *
*  @brief    rviz basic pannel configure class.                                          *
*  @author   Haodong Yang                                                                *
*  @version  1.0.1                                                                       *
*  @date     2022.07.06                                                                  *
*  @license  GNU General Public License (GPL)                                            *
******************************************************************************************
"""
import os
import sys
import yaml

dir_path = os.path.dirname(os.path.realpath(__file__))
sys.path.insert(0, dir_path)

from config import RVizConfig
from displays import Displays

topics = {
    "robot_model": "/robot_description",
    "lidar_2d": "/sensors/lidars/lidar/scan",
    "lidar_3d": "/sensors/lidars/lidar/points",
    "global_map": "/map",
    "global_costmap": "/usv_simulation/costmap/costmap",
    "local_trajectory": "/local_trajectory",
    "global_path": "/global_path",
    "lookahead_point": "/lookahead_point",
    "goal_pose": "/goal",
}

def updateRvizDisplay(robot_namespace: str, display: Displays, rviz: RVizConfig):
    meta_robot = Displays()
    robot_model_topic = robot_namespace + topics["robot_model"] if robot_namespace else topics["robot_model"]
    meta_robot.addModel(parameter=robot_model_topic, tf_prefix=robot_namespace)

    laser_data = Displays()
    lidar2d_topic = robot_namespace + topics["lidar_2d"] if robot_namespace else topics["lidar_2d"]
    lidar3d_topic = robot_namespace + topics["lidar_3d"] if robot_namespace else topics["lidar_3d"]
    laser_data.addLaserscan(topic=lidar2d_topic, size=1, alpha=1, color_transform="AxisColor")
    laser_data.addPointCloud2(topic=lidar3d_topic, size=0.3, alpha=1, color=(25, 255, 0))
    meta_robot.addGroup("LiDAR", laser_data)

    map_data = Displays()
    global_map_topic = robot_namespace + topics["global_map"] if robot_namespace else topics["global_map"]
    global_costmap_topic = robot_namespace + topics["global_costmap"] if robot_namespace else topics["global_costmap"]
    map_data.addMap(topic=global_map_topic, name="global_map", alpha=0.7, scheme="costmap")
    map_data.addMap(topic=global_costmap_topic, name="global_costmap", alpha=0.3, scheme="costmap")
    meta_robot.addGroup("Map", map_data)

    plan_data = Displays()
    global_map_topic = robot_namespace + topics["global_map"] if robot_namespace else topics["global_map"]
    global_costmap_topic = robot_namespace + topics["global_costmap"] if robot_namespace else topics["global_costmap"]
    plan_data.addPath(topic=f"{robot_namespace}/local_trajectory", name="local_trajectory", color=(25, 255, 0))
    plan_data.addPath(topic=f"{robot_namespace}/global_path", name="global_path", color=(239, 41, 41), line_width=0.3)
    meta_robot.addGroup("Plan", plan_data)

    marker_data = Displays()
    lookahead_point_topic = robot_namespace + topics["lookahead_point"] if robot_namespace else topics["lookahead_point"]
    marker_data.addMarker(topic=lookahead_point_topic, name="lookahead_point")
    meta_robot.addGroup("Marker", marker_data)

    pose_data = Displays()
    goal_pose_topic = robot_namespace + topics["goal_pose"] if robot_namespace else topics["goal_pose"]
    pose_data.addPose(topic=goal_pose_topic, name="goal", color=(255, 25, 0), arrow_shape=(3.0, 1.0, 3.0, 0.5))
    meta_robot.addGroup("Pose", pose_data)

    display.addGroup(robot_namespace, meta_robot)
    rviz.appendCustomGoal(topic=f"{robot_namespace}/goal", name=f"{robot_namespace} goal")

'''
custom configure
'''
user_config = None
user_config_path = sys.argv[1]
with open(user_config_path, "r") as f:
    user_config = yaml.load(f, Loader=yaml.FullLoader)
if user_config is None:
    raise RuntimeError(f"Configure in {user_config_path} is invalid.")

robots_num = len(user_config["robots_config"]) if "robots_config" in user_config.keys() else 0
if robots_num == 0:
    raise RuntimeError("There is no robot!")

'''
Rviz configure
'''
r = RVizConfig("cache", os.path.split(os.path.realpath(__file__))[0] + "/../../../sim_env/rviz/")
# set Rviz standard tools
r.setStdTools()
# visualize Rviz models
d = Displays()
d.addTf()
d.addAxes(frame="world")

# single robot
if robots_num == 1:
    robot_type = user_config["robots_config"][0]["robot1_type"]
    updateRvizDisplay(robot_type, d, r)
else:
    # multi-robot
    for i in range(robots_num):
        index = i + 1
        robot_type = user_config["robots_config"][i][f"robot{index}_type"]
        robot_namespace = f"{robot_type}{index}"
        updateRvizDisplay(robot_namespace, d, r)

r.setStdPanels()
r.setDisplays(d)
r.setGlobalOptions(frame="world", color=(114, 159, 207))
r.run()
