# 0141 接收侧：剥外壳，按与 0411 相同的原点把目标换到地图坐标，再画成占用栅格。
"""Turn received 0141 target lists into an occupancy grid for the costmap.

Target latitude/longitude are converted with the same ``GeoReference`` that
decodes 0411 into odometry, so obstacles and the vessel share one map frame.
Each target becomes a filled disc, and every packet yields a complete grid
that replaces the previous one. This module has no ROS dependency.
"""
from __future__ import annotations

from array import array
from dataclasses import dataclass
import json
import math
from typing import Any, Dict, List, Optional, Sequence, Tuple

from common.geodesy import GeoReference

from .config import validate_obstacle_object


# nav_msgs/OccupancyGrid 的占用与空闲取值；costmap 静态层把 100 当作致命障碍。
OCCUPIED = 100
FREE = 0


@dataclass(frozen=True)
class ObstacleDisc:
    """地图坐标系中的一个圆形障碍，单位米；x 向东、y 向北。"""

    x: float
    y: float
    radius: float


@dataclass(frozen=True)
class ObstacleGrid:
    """行优先占用栅格；origin 是第 0 行第 0 列格子左下角的地图坐标。"""

    origin_x: float
    origin_y: float
    resolution: float
    width: int
    height: int
    data: array
    # 圆心和圆面都落在窗口外、因此没有画出的目标数。
    dropped: int = 0


# 解析一包 0141：带 head/content 外壳时取 content，再校验 count 与 targets。
def parse_obstacle_message(text: str) -> Tuple[Dict[str, Any], bool]:
    """Return the 0141 business object and whether it arrived wrapped.

    A wrapped message must carry object ``head`` and ``content`` like the other
    received LaDDS topics; bare business objects remain accepted.
    """
    if not isinstance(text, str):
        raise ValueError("0141 payload must be JSON text")
    try:
        root = json.loads(text)
    except (json.JSONDecodeError, TypeError, ValueError) as exc:
        raise ValueError(f"invalid obstacle JSON: {exc}") from exc
    if type(root) is not dict:
        raise ValueError("obstacle JSON root must be an object")
    wrapped = "content" in root
    if wrapped:
        if type(root.get("head")) is not dict or type(root["content"]) is not dict:
            raise ValueError("wrapped 0141 requires object head and content")
        root = root["content"]
    return validate_obstacle_object(root), wrapped


def _finite_number(value: Any) -> bool:
    # bool is excluded because type(True) is bool, not int.
    return type(value) in (int, float) and math.isfinite(value)


# 经纬度用解 0411 的同一个原点换成地图坐标；半径优先取 threatingRadius。
def target_discs(
    content: Dict[str, Any],
    origin: GeoReference,
    *,
    default_radius: float,
    radius_scale: float = 1.0,
) -> Tuple[List[ObstacleDisc], int]:
    """Convert validated targets to map-frame discs; return discs and skip count.

    A target without finite in-range latitude/longitude is skipped rather than
    invalidating the whole list. ``threatingRadius`` times ``radius_scale``
    gives metres; a missing or non-positive value uses ``default_radius``.
    """
    discs: List[ObstacleDisc] = []
    skipped = 0
    for target in content["targets"]:
        latitude = target.get("latitude")
        longitude = target.get("longitude")
        if not (
            _finite_number(latitude)
            and _finite_number(longitude)
            and -90.0 <= latitude <= 90.0
            and -180.0 <= longitude <= 180.0
        ):
            skipped += 1
            continue
        x, y, _ = origin.to_local(float(latitude), float(longitude))
        radius = target.get("threatingRadius")
        if _finite_number(radius) and radius > 0:
            radius = float(radius) * radius_scale
        else:
            radius = default_radius
        if not (math.isfinite(x) and math.isfinite(y) and math.isfinite(radius)):
            skipped += 1
            continue
        discs.append(ObstacleDisc(float(x), float(y), float(radius)))
    return discs, skipped


# 把圆画进覆盖它们的最小栅格；窗口以锚点为中心限制边长，窗口外的圆不画。
def rasterize(
    discs: Sequence[ObstacleDisc],
    *,
    resolution: float,
    max_size: float,
    anchor: Optional[Tuple[float, float]] = None,
) -> ObstacleGrid:
    """Draw discs into one grid that replaces the previously published grid.

    The grid spans the discs' bounding box, clipped to a ``max_size`` square
    centred on ``anchor`` (normally the latest 0411 position, otherwise the
    first disc). Without discs, one free cell still gives the costmap a valid
    map so that the previous obstacles are cleared.
    """
    if anchor is None:
        anchor = (discs[0].x, discs[0].y) if discs else (0.0, 0.0)
    half = max_size / 2.0
    low_x, high_x = anchor[0] - half, anchor[0] + half
    low_y, high_y = anchor[1] - half, anchor[1] + half
    inside = [
        disc for disc in discs
        if disc.x + disc.radius > low_x and disc.x - disc.radius < high_x
        and disc.y + disc.radius > low_y and disc.y - disc.radius < high_y
    ]
    dropped = len(discs) - len(inside)
    if not inside:
        origin_x = math.floor(anchor[0] / resolution) * resolution
        origin_y = math.floor(anchor[1] / resolution) * resolution
        return ObstacleGrid(origin_x, origin_y, resolution, 1, 1, array("b", [FREE]), dropped)

    min_x = max(low_x, min(disc.x - disc.radius for disc in inside))
    max_x = min(high_x, max(disc.x + disc.radius for disc in inside))
    min_y = max(low_y, min(disc.y - disc.radius for disc in inside))
    max_y = min(high_y, max(disc.y + disc.radius for disc in inside))
    # Snap to whole cells so successive packets share cell boundaries.
    origin_x = math.floor(min_x / resolution) * resolution
    origin_y = math.floor(min_y / resolution) * resolution
    width = max(1, math.ceil((max_x - origin_x) / resolution))
    height = max(1, math.ceil((max_y - origin_y) / resolution))
    data = array("b", bytes(width * height))
    for disc in inside:
        _fill_disc(data, width, height, origin_x, origin_y, resolution, disc)
    return ObstacleGrid(origin_x, origin_y, resolution, width, height, data, dropped)


# 格心落在圆内的格子标为占用；圆心所在格子必画，避免小于一格的目标消失。
def _fill_disc(
    data: array,
    width: int,
    height: int,
    origin_x: float,
    origin_y: float,
    resolution: float,
    disc: ObstacleDisc,
) -> None:
    radius_squared = disc.radius * disc.radius
    first_column = max(0, math.floor((disc.x - disc.radius - origin_x) / resolution))
    last_column = min(width - 1, math.floor((disc.x + disc.radius - origin_x) / resolution))
    first_row = max(0, math.floor((disc.y - disc.radius - origin_y) / resolution))
    last_row = min(height - 1, math.floor((disc.y + disc.radius - origin_y) / resolution))
    for row in range(first_row, last_row + 1):
        dy = origin_y + (row + 0.5) * resolution - disc.y
        offset = row * width
        for column in range(first_column, last_column + 1):
            dx = origin_x + (column + 0.5) * resolution - disc.x
            if dx * dx + dy * dy <= radius_squared:
                data[offset + column] = OCCUPIED
    center_column = math.floor((disc.x - origin_x) / resolution)
    center_row = math.floor((disc.y - origin_y) / resolution)
    if 0 <= center_column < width and 0 <= center_row < height:
        data[center_row * width + center_column] = OCCUPIED


__all__ = [
    "FREE",
    "OCCUPIED",
    "ObstacleDisc",
    "ObstacleGrid",
    "parse_obstacle_message",
    "rasterize",
    "target_discs",
]
