"""Ordinary-navigation decision/planning only; inputs are supplied externally."""
from sim_env.launch_support import launch_description


def generate_launch_description():
    return launch_description("decision")
