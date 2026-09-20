"""Serialize a ROS 2 costmap snapshot into the temporary 0141 JSON envelope.

The current navigation stack publishes a grid rather than a tracked-target list.
For the LaDDS transport smoke test we keep that grid information intact and put a
sparse list of occupied cells in one ``targets`` entry.  This module deliberately
has no ROS imports so that the serializer can be tested on the development host.
"""

from __future__ import annotations

import json
from typing import Any, Dict, List, Sequence, Tuple


DEFAULT_COST_THRESHOLD = 254
DEFAULT_MAX_CELLS = 4096


def _field(value: Any, name: str, default: Any = None) -> Any:
    """Read an attribute from a ROS message or a mapping used by tests."""

    if value is None:
        return default
    if isinstance(value, dict):
        return value.get(name, default)
    return getattr(value, name, default)


def _time_payload(value: Any) -> Dict[str, int]:
    return {
        "sec": int(_field(value, "sec", 0)),
        "nanosec": int(_field(value, "nanosec", 0)),
    }


def _pose_payload(value: Any) -> Dict[str, Dict[str, float]]:
    position = _field(value, "position", None)
    orientation = _field(value, "orientation", None)
    return {
        "position": {
            "x": float(_field(position, "x", 0.0)),
            "y": float(_field(position, "y", 0.0)),
            "z": float(_field(position, "z", 0.0)),
        },
        "orientation": {
            "x": float(_field(orientation, "x", 0.0)),
            "y": float(_field(orientation, "y", 0.0)),
            "z": float(_field(orientation, "z", 0.0)),
            "w": float(_field(orientation, "w", 1.0)),
        },
    }


def _metadata_payload(metadata: Any) -> Dict[str, Any]:
    return {
        "map_load_time": _time_payload(_field(metadata, "map_load_time", None)),
        "update_time": _time_payload(_field(metadata, "update_time", None)),
        "layer": str(_field(metadata, "layer", "")),
        "resolution": float(_field(metadata, "resolution", 0.0)),
        "size_x": int(_field(metadata, "size_x", 0)),
        "size_y": int(_field(metadata, "size_y", 0)),
        "origin": _pose_payload(_field(metadata, "origin", None)),
    }


def _validate_options(
    cost_threshold: int,
    max_cells: int,
) -> Tuple[int, int]:
    threshold = int(cost_threshold)
    limit = int(max_cells)
    if not 0 <= threshold <= 255:
        raise ValueError("cost_threshold must be between 0 and 255")
    if limit <= 0:
        raise ValueError("max_cells must be greater than zero")
    return threshold, limit


def _occupied_cells(
    data: Sequence[Any],
    *,
    size_x: int,
    cost_threshold: int,
    max_cells: int,
    include_unknown: bool,
) -> Tuple[List[Dict[str, int]], int, bool]:
    cells: List[Dict[str, int]] = []
    total = 0
    for index, raw_cost in enumerate(data):
        cost = max(0, min(255, int(raw_cost)))
        # nav2 costmaps use 255 for NO_INFORMATION.  Unknown space is not an
        # observed obstacle unless the caller explicitly asks to include it.
        if cost < cost_threshold or (cost == 255 and not include_unknown):
            continue
        total += 1
        if len(cells) >= max_cells:
            continue
        x = index % size_x if size_x > 0 else index
        y = index // size_x if size_x > 0 else 0
        cells.append({"x": int(x), "y": int(y), "cost": cost})
    return cells, total, total > len(cells)


def costmap_to_0141_dict(
    message: Any,
    *,
    sequence: int = 1,
    cost_threshold: int = DEFAULT_COST_THRESHOLD,
    max_cells: int = DEFAULT_MAX_CELLS,
    include_unknown: bool = False,
) -> Dict[str, Any]:
    """Return a JSON-compatible dictionary for one costmap snapshot.

    ``count`` is intentionally one: it denotes one snapshot, while
    ``targets[0].obstacle_cells`` carries the sparse grid cells.  This avoids
    pretending that each grid cell is a separately tracked vessel target and
    keeps the message below the bridge's default 1 MiB payload limit.
    """

    threshold, limit = _validate_options(cost_threshold, max_cells)
    metadata = _metadata_payload(_field(message, "metadata", None))
    data_value = _field(message, "data", ())
    data = [] if data_value is None else list(data_value)
    frame_id = str(_field(_field(message, "header", None), "frame_id", ""))
    stamp = _time_payload(
        _field(_field(message, "header", None), "stamp", None)
    )
    cells, occupied_count, truncated = _occupied_cells(
        data,
        size_x=metadata["size_x"],
        cost_threshold=threshold,
        max_cells=limit,
        include_unknown=bool(include_unknown),
    )

    # The sequence is placed in the protocol's 16-bit batch slot.  The rest of
    # the target object is an explicitly named test envelope whose contents are
    # the native costmap metadata and obstacle cells.
    batch = int(sequence) % 65535
    target = {
        "batch": batch,
        "kind": "costmap_snapshot",
        "source_topic": "costmap_raw",
        "frame_id": frame_id,
        "stamp": stamp,
        "metadata": metadata,
        "cost_threshold": threshold,
        "include_unknown": bool(include_unknown),
        "data_length": len(data),
        "obstacle_cell_count": occupied_count,
        "obstacle_cells": cells,
        "truncated": truncated,
    }
    return {
        "format": "costmap_snapshot_v1",
        "source": "costmap_raw",
        "count": 1,
        "targets": [target],
    }


def costmap_to_0141_json(
    message: Any,
    *,
    sequence: int = 1,
    cost_threshold: int = DEFAULT_COST_THRESHOLD,
    max_cells: int = DEFAULT_MAX_CELLS,
    include_unknown: bool = False,
) -> str:
    """Serialize one costmap snapshot as compact UTF-8-compatible JSON text."""

    payload = costmap_to_0141_dict(
        message,
        sequence=sequence,
        cost_threshold=cost_threshold,
        max_cells=max_cells,
        include_unknown=include_unknown,
    )
    return json.dumps(payload, ensure_ascii=False, separators=(",", ":"), allow_nan=False)
