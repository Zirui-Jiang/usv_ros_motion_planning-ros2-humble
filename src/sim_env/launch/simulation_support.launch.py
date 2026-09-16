# 仅启动 Gazebo、船模、定位与显示资源；具体动作由 sim_env.launch_support 统一构建。
"""Gazebo, model/spawn, sensor TF and visualization; no business nodes."""
from sim_env.launch_support import launch_description


# 向 ROS launch 返回启动描述对象，由启动系统解释执行。
def generate_launch_description():
    return launch_description("simulation_support")
