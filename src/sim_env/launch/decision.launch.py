# 兼容旧命令，转发到独立决策包；具体动作由 sim_env.launch_support 统一构建。
"""Ordinary-navigation decision/planning only; inputs are supplied externally."""
from sim_env.launch_support import launch_description


# 向 ROS launch 返回启动描述对象，由启动系统解释执行。
def generate_launch_description():
    return launch_description("decision")
