#!/usr/bin/env python3
"""检查一条 0211 三点航路在当前规划逻辑和 1473 米地图上能否搜完。"""
import json
import math
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
for relative in (
    "src/core/shared/common",
    "src/core/decision_module/path_planner",
    "src/core/decision_module/path_processor",
    "src/core/decision_module/trajectory_planner",
    "src/core/control_module/controller",
    "src/core/shared/usv_navigation",
    "src/core/decision_module/usv_decision",
    "src/core/communication_module/usv_ladds_bridge",
):
    sys.path.insert(0, str(ROOT / relative))

from common.geodesy import GeoReference
from common.models import AlgorithmStatus, GridSnapshot, VesselState
from usv_decision.runtime import PlanningRequest, plan_navigation
from usv_ladds_bridge.route_task import parse_route
from usv_navigation.configuration import load_project_parameters

MESSAGE = {
    "content": {
        "areas": None,
        "areasCount": None,
        "count": 3,
        "guardingHeading": None,
        "guardingRadius": None,
        "points": [
            {"latitude": 30.548, "longitude": 114.39, "speed": 4, "type": 1},
            {"latitude": 30.545, "longitude": 114.393, "speed": 4, "type": 1},
            {"latitude": 30.55, "longitude": 114.397, "speed": 4, "type": 1},
        ],
        "taskType": 16,
        "travelId": 17,
    },
    "head": {
        "packageSeq": 25,
        "packageType": 0,
        "recvPlatform": 1000,
        "sendPlatform": 1000,
        "time": "2026-09-22 18:59:08.355",
    },
}

# 与当前仿真一致：艇初始位置、滚动代价地图边长和分辨率。
START = (23.0, -10.0)
MAP_SIZE_M = 1473.0
RESOLUTION = 1.0
RADIUS = 1.5


def rolling_grid(center):
    """空的滚动代价地图，中心在给定船位，未知区按可通行的 0 处理。"""
    cells = int(MAP_SIZE_M / RESOLUTION)
    origin_x = center[0] - MAP_SIZE_M / 2.0
    origin_y = center[1] - MAP_SIZE_M / 2.0
    data = np.zeros((cells, cells), dtype=np.uint8)
    return GridSnapshot(data, RESOLUTION, origin_x, origin_y, stamp=1.0)


def inside(grid, x, y):
    return grid.world_to_map(x, y) is not None


def anchors_hit(route, waypoints):
    found = []
    for index, point in enumerate(waypoints):
        hit = np.any(np.linalg.norm(route[:, :2] - (point[0], point[1]), axis=1) < 1.0)
        found.append(hit)
        print(f"  航点{index + 1} ({point[0]:.1f}, {point[1]:.1f}) 在路径上: {hit}")
    return found


def plan_once(waypoints, state, grid, parameters):
    request = PlanningRequest(
        version=1,
        goal=waypoints[0][:3],
        state=state,
        grid=grid,
        route=np.empty((0, 3)),
        parameters=parameters,
        waypoints=tuple(waypoints),
        waypoint_radius=RADIUS,
    )
    return plan_navigation(request)


def main():
    origin = GeoReference(30.5472, 114.3891)
    task = parse_route(json.dumps(MESSAGE, ensure_ascii=False), origin)
    waypoints = tuple((point.x, point.y, 0.0, point.speed_mps) for point in task.points)
    parameters = load_project_parameters(
        ROOT / "scripts/validation/config/ladds_single_leg_dynamics.yaml",
        ROOT / "src/core/shared/usv_navigation/config/robots/otter/controller_params_otter.yaml",
        ROOT / "src/core/shared/usv_navigation/config/trajectory_planner/trajectory_planner_params.yaml",
        ROOT / "src/core/shared/usv_navigation/config/path_planner/common_planner_params.yaml",
    )
    print(f"规划时限 {parameters['navigation.planning_timeout']:.1f} 秒")
    print(f"速度上限 {parameters['dynamics.velocity_max']:.2f} 米/秒")
    grid = rolling_grid(START)
    print(f"地图原点 ({grid.origin_x:.1f}, {grid.origin_y:.1f})，边长 {MAP_SIZE_M:.0f} 米")
    for index, point in enumerate(waypoints):
        print(f"  航点{index + 1} 在初始地图内: {inside(grid, point[0], point[1])}")

    yaw = math.atan2(waypoints[0][1] - START[1], waypoints[0][0] - START[0])
    state = VesselState(START[0], START[1], yaw, stamp=1.0)
    print("开始一次规划，起点是艇初始位置 (23, -10)")
    result = plan_once(waypoints, state, grid, parameters)
    print(f"状态 {result.status.value}，耗时 {result.elapsed:.2f} 秒")
    if result.message:
        print(f"原因 {result.message}")
    if result.route.size == 0:
        print("没有搜出路径，航路不能跑完。")
        return 1
    found = anchors_hit(result.route, waypoints)
    length = float(np.linalg.norm(np.diff(result.route[:, :2], axis=0), axis=1).sum())
    print(f"路径长度 {length:.1f} 米，轨迹点数 {len(result.trajectory)}")
    if all(found):
        print("三个航点都在同一次规划结果里，这段航路可以一次搜完。")
        return 0
    missing = [index + 1 for index, hit in enumerate(found) if not hit]
    print(f"第 {missing} 个点没有搜进这次路径。")
    next_index = found.index(False)
    start_xy = waypoints[next_index - 1][:2]
    print(f"改从航点{next_index} ({start_xy[0]:.1f}, {start_xy[1]:.1f}) 继续搜剩下的点")
    grid = rolling_grid(start_xy)
    for index, point in enumerate(waypoints[next_index - 1:], start=next_index):
        print(f"  航点{index} 在新地图内: {inside(grid, point[0], point[1])}")
    yaw = math.atan2(waypoints[next_index][1] - start_xy[1], waypoints[next_index][0] - start_xy[0])
    state = VesselState(start_xy[0], start_xy[1], yaw, stamp=2.0)
    result = plan_once(waypoints[next_index - 1:], state, grid, parameters)
    print(f"续搜状态 {result.status.value}，耗时 {result.elapsed:.2f} 秒")
    if result.message:
        print(f"原因 {result.message}")
    if result.route.size == 0:
        print("续搜没有路径，后面的航点跑不完。")
        return 1
    found_rest = anchors_hit(result.route, waypoints[next_index - 1:])
    if all(found_rest):
        print("船到前一个点之后，剩下的点可以搜完。整条航路要分两次规划才能走完。")
        return 0
    print("续搜仍有航点失败，这段航路按当前限制跑不完。")
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
