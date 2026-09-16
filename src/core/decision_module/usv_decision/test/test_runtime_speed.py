# 单次导航请求的期望速度只影响该次规划，不回写共享启动参数。
from copy import deepcopy

import numpy as np

from common.models import AlgorithmStatus, GridSnapshot, VesselState
from usv_decision.runtime import PlanningRequest, plan_navigation
from usv_navigation.configuration import resolve_parameters


def test_request_speed_overrides_planning_model_without_mutating_parameters():
    parameters = resolve_parameters({})
    original = deepcopy(parameters)
    request = PlanningRequest(
        version=1,
        goal=(18.5, 10.5, 0.),
        state=VesselState(2.5, 10.5, 0., stamp=10.),
        grid=GridSnapshot(np.zeros((30, 30), np.uint8), 1., stamp=10.),
        route=np.empty((0, 3)),
        parameters=parameters,
        desired_speed_mps=.4,
    )

    result = plan_navigation(request)

    assert result.status == AlgorithmStatus.SUCCESS, result.message
    assert np.max(result.trajectory.points[:, 6]) == .4
    assert parameters == original


def test_legacy_six_argument_request_keeps_configured_cruising_speed():
    parameters = resolve_parameters({'dynamics.cruising_speed': .6})
    request = PlanningRequest(
        1, (18.5, 10.5, 0.), VesselState(2.5, 10.5, 0., stamp=10.),
        GridSnapshot(np.zeros((30, 30), np.uint8), 1., stamp=10.),
        np.empty((0, 3)), parameters)

    result = plan_navigation(request)

    assert result.status == AlgorithmStatus.SUCCESS, result.message
    assert np.max(result.trajectory.points[:, 6]) == .6
