#!/usr/bin/env python3
# 独立感知 launch 入口，具体参数读取与三个业务节点的组合由 launch_support 负责。
"""Start the independent LiDAR perception module."""
from usv_perception.launch_support import launch_description


def generate_launch_description():
    return launch_description()
