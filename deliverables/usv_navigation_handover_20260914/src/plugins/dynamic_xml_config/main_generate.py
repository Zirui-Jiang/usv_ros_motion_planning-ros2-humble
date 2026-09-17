#!/usr/bin/python
# -*- coding: utf-8 -*-

"""
******************************************************************************************
*  Copyright (C) 2023 Yang Haodong, All Rights Reserved                                  *
*                                                                                        *
*  @brief    generate gazebo world obstacles and user config dynamically.                *
*  @author   Haodong Yang                                                                *
*  @version  1.1.0                                                                       *
*  @date     2023.07.19                                                                  *
*  @license  GNU General Public License (GPL)                                            *
******************************************************************************************
"""
import sys

from plugins import ObstacleGenerator


class MainGenerator:
    def __init__(self, *plugins) -> None:
        self.app_list = [app for app in plugins]

    def generate(self):
        """
        Run all registered generators (obstacles spawn into the gazebo world, etc).
        """
        for app in self.app_list:
            app.plugin()


# dynamic generator: only the obstacle generator is still used on ROS 2;
# robot/launch generation is handled by sim_env/launch/main.launch.py.
main_gen = MainGenerator(ObstacleGenerator())
main_gen.generate()
