#!/usr/bin/env python3
"""Compatible simulation entry composing perception, decision and control."""
from sim_env.launch_support import launch_description


def generate_launch_description():
    return launch_description("main")
