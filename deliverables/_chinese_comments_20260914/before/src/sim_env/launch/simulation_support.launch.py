"""Gazebo, model/spawn, sensor TF and visualization; no business nodes."""
from sim_env.launch_support import launch_description


def generate_launch_description():
    return launch_description("simulation_support")
