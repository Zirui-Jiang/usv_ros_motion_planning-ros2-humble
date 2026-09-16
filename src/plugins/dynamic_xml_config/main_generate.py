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
# 动态场景生成入口，目前注册障碍物生成器；机器人启动配置由仿真包的 launch 管理。
import sys

from plugins import ObstacleGenerator


# 保存传入的生成器对象，调用统一 plugin 接口依次执行。
class MainGenerator:
    def __init__(self, *plugins) -> None:
        self.app_list = [app for app in plugins]

    # 顺序运行已注册生成器；任意生成器异常会向上传播，不在这里隐藏失败。
    def generate(self):
        """
        Run all registered generators (obstacles spawn into the gazebo world, etc).
        """
        for app in self.app_list:
            app.plugin()


# dynamic generator: only the obstacle generator is still used on ROS 2;
# robot/launch generation is handled by sim_env/launch/main.launch.py.
# 脚本运行时立即执行障碍物生成，会更新用户所选世界文件。
main_gen = MainGenerator(ObstacleGenerator())
main_gen.generate()
