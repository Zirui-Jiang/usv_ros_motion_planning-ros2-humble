#!/usr/bin/env python3
# 组合仿真支撑、感知、决策和航控的主入口；具体动作由 sim_env.launch_support 统一构建。
"""Compatible simulation entry composing perception, decision and control."""
from sim_env.launch_support import launch_description


# 向 ROS launch 返回启动描述对象，由启动系统解释执行。
def generate_launch_description():
    return launch_description("main")
