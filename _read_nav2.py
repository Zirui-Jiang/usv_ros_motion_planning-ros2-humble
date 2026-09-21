#!/usr/bin/env python3
from pathlib import Path
for p in [
    Path("/opt/ros/humble/include/nav2_costmap_2d/nav2_costmap_2d/obstacle_layer.hpp"),
    Path("/opt/ros/humble/include/nav2_costmap_2d/nav2_costmap_2d/observation_buffer.hpp"),
]:
    print("=" * 20, p)
    print(p.read_text()[:8000])
