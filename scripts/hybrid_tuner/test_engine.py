# 调参台数值合同：膨胀公式、默认三种曲率、以及终点艏向税在空图上可测。
"""Hybrid tuner engine checks; imports the local engine module by file path."""
from __future__ import annotations

import importlib.util
import math
from pathlib import Path

import numpy as np
import pytest

_ENGINE_PATH = Path(__file__).with_name("engine.py")
_SPEC = importlib.util.spec_from_file_location("hybrid_tuner_engine", _ENGINE_PATH)
engine = importlib.util.module_from_spec(_SPEC)
_SPEC.loader.exec_module(engine)


def test_inflate_matches_nav2_inscribed_and_exponential_ring():
    occupancy = engine.empty_occupancy()
    occupancy[40, 40] = engine.LETHAL
    occupancy[40, 42] = engine.UNKNOWN
    costs = engine.inflate(occupancy, inscribed_radius=1.3, inflation_radius=3.0, cost_scaling_factor=2.0)
    assert costs[40, 40] == 254
    assert costs[40, 41] == 253
    expected = int(252.0 * math.exp(-2.0 * (1.5 - 1.3)))
    assert costs[40, 43] == expected
    assert costs[40, 47] == 0
    assert costs[40, 42] == 255


def test_empty_eastbound_plan_is_nearly_straight():
    result = engine.plan(engine.empty_occupancy(), (8.0, 12.0, 0.0), (24.0, 12.0, 0.0),
                         {"timeout": 3.0, "heuristic_weight": 1.0, "max_expansions": 5000})
    assert result["status"] == "success", result["message"]
    assert result["length"] == pytest.approx(16.0, abs=0.6)
    assert result["expanded"] >= 1


def test_forced_goal_heading_is_longer_than_free_heading_on_open_water():
    occupancy = engine.empty_occupancy()
    start, goal = (8.0, 12.0, 0.0), (28.0, 12.0, math.pi / 2)
    params = {"timeout": 3.0, "heuristic_weight": 1.0, "max_expansions": 8000, "with_runtime": False}
    data = engine.heading_experiment(occupancy, start, goal, params, samples=8)
    assert data["fixed"]["status"] == "success", data["fixed"]["message"]
    assert data["best"] is not None and data["best"]["status"] == "success"
    assert data["best"]["length"] + 0.4 < data["fixed"]["length"]


def test_extra_turning_radii_still_plan_through_the_public_api():
    result = engine.plan(engine.empty_occupancy(), (8.0, 12.0, 0.0), (20.0, 12.0, 0.0),
                         {"turning_radius_multipliers": [1.0, 2.0], "timeout": 3.0})
    assert result["status"] == "success", result["message"]
    assert np.asarray(result["path"]).shape[1] == 3
