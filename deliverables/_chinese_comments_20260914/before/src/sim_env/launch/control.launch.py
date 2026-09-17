"""Ordinary-navigation control only; references and inputs come from peers."""
from sim_env.launch_support import launch_description


def generate_launch_description():
    return launch_description("control")
