# 验证圆航线连续恒速、径向收敛与偏航角速度可行性，覆盖顺逆时针。
import math
import numpy as np
import pytest

from common.models import AlgorithmStatus, USVModel, VesselState
from trajectory_planner import LosTrajectoryPlanner
from trajectory_planner.circle import CircleReference


# 多圈参考不停车且半径误差和转速受限。
@pytest.mark.parametrize('clockwise', [True, False])
def test_circle_rollout_closes_without_stopping_and_respects_turn_limit(clockwise):
    sign = -1 if clockwise else 1
    circle = CircleReference((0., 0.), 2., .5, clockwise)
    planner = LosTrajectoryPlanner(USVModel(), {'trajectory_time': 80., 'lookahead_distance': 1.})
    result = planner.plan_circle(circle, VesselState(x=2., psi=sign*math.pi/2))
    assert result.status == AlgorithmStatus.SUCCESS
    p = result.trajectory.points
    assert result.trajectory.goal is None
    np.testing.assert_allclose(p[:, 6], .5)
    assert p[-1, 1] == pytest.approx(40.)
    assert np.max(np.abs(np.linalg.norm(p[:, 2:4], axis=1)-2.)) < .04
    assert np.max(np.abs(p[:, 5]*p[:, 6])) <= .3+1e-12
    progress = sign*np.unwrap(np.arctan2(p[:, 3], p[:, 2]))
    assert progress[-1]-progress[0] > 3*2*math.pi


# 圆内外初始位置均降低径向误差，速度不附加减速坡道。
@pytest.mark.parametrize('x', [9., 11.])
def test_circle_guidance_reduces_radial_error_without_speed_ramp(x):
    planner = LosTrajectoryPlanner(USVModel(), {'trajectory_time': 20., 'lookahead_distance': 1.})
    result = planner.plan_circle(CircleReference((0., 0.), 10., .5, False), VesselState(x=x, psi=math.pi/2))
    assert result.status == AlgorithmStatus.SUCCESS
    p = result.trajectory.points
    assert abs(np.linalg.norm(p[-1, 2:4])-10.) < .1
    np.testing.assert_allclose(p[:, 6], .5)


# 拒绝所需角速度过大及非有限状态。
def test_circle_rejects_infeasible_yaw_rate_and_nonfinite_state():
    planner = LosTrajectoryPlanner(USVModel())
    assert planner.plan_circle(CircleReference((0., 0.), 1., .5), VesselState()).status == AlgorithmStatus.INVALID_INPUT
    assert planner.plan_circle(CircleReference((0., 0.), 10., .5), VesselState(x=math.nan)).status == AlgorithmStatus.INVALID_INPUT
