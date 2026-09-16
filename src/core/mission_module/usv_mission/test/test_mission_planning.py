# 任务规划回归：用合成障碍验证切点、候选筛选、整船扫掠、统一时限和条件起步几何。
from concurrent.futures import Future
from dataclasses import replace
import math
import numpy as np
import pytest
from common.models import AlgorithmStatus, GridSnapshot, VesselState
from common.collision import footprint_path_is_clear
from common.dynamics import braking_command, predict_control_path
from common.models import USVModel
from usv_mission.configuration import resolve_parameters
from usv_mission.missions import MissionManager, MissionSpec
from usv_mission.missions.geometry import (entry_candidates, escape_candidates, pose_is_clear,
                                      turning_envelope_is_clear)
from usv_mission.runtime import LatestPlanWorker, PlanningRequest, plan_navigation
from trajectory_planner.circle import CircleReference


def scenario(mode, state, **kwargs):
    cfg = resolve_parameters({})
    manager = MissionManager(MissionSpec(mode, 0., 0., **kwargs), (0., 0., 0.), 0., config=cfg)
    grid = GridSnapshot(np.zeros((200, 200), np.uint8), .5, -50., -50., stamp=state.stamp)
    intent = manager.tick(state, grid, state.stamp)
    return manager, PlanningRequest(1, (0., 0., 0.), state, grid, np.empty((0, 3)), cfg, intent)


# 验证：圆周入圈候选覆盖各象限并含船外两条精确切线的切点。
def test_geometric_circle_entries_include_two_exact_tangencies_and_all_quadrants():
    circle = CircleReference((0., 0.), 10., .5, False)
    points = entry_candidates(circle, VesselState(x=20.))
    assert len(points) >= 24
    assert any(np.allclose(p[:2], [5., math.sqrt(75.)]) for p in points)
    assert any(np.allclose(p[:2], [5., -math.sqrt(75.)]) for p in points)
    for p in points:
        assert np.linalg.norm(p[:2]) == pytest.approx(10.)


# 验证：恢复候选沿指定方向覆盖余下圆周并排除当前点。
@pytest.mark.parametrize('clockwise', [True, False])
def test_forward_recovery_covers_the_remaining_circle_without_the_current_point(clockwise):
    circle = CircleReference((0., 0.), 8., .5, clockwise)
    state = VesselState(x=8.)
    points = entry_candidates(circle, state, forward_only=True)
    assert len(points) == 23
    assert not any(np.allclose(pose[:2], state.pose[:2]) for pose in points)
    progress = [(circle.direction*math.atan2(pose[1], pose[0])) % (2*math.pi) for pose in points]
    assert min(progress) == pytest.approx(math.radians(15))
    assert max(progress) == pytest.approx(math.radians(345))


# 验证：前半圆全被阻挡时仍能在后半圆找到安全恢复点。
@pytest.mark.parametrize('clockwise', [True, False])
def test_recovery_finds_safe_arc_when_the_first_half_circle_is_blocked(clockwise):
    cfg = resolve_parameters({})
    direction = -1 if clockwise else 1
    state = VesselState(x=8., psi=direction*math.pi/2)
    data = np.zeros((160, 160), np.uint8)
    data[int((direction*8.+40.)/.5), :] = 254
    grid = GridSnapshot(data, .5, -40., -40.)
    manager = MissionManager(MissionSpec('orbit', 0., 0., orbit_radius=8., clockwise=clockwise),
                             (0., 0., 0.), 0., config=cfg)
    manager.request_escape('first half of recovery circle has no clear release envelope')
    intent = manager.tick(state, grid, 0.)
    assert intent.kind == 'escape' and intent.candidates
    assert all(direction*pose[1] < 0. for pose in intent.candidates)
    response = plan_navigation(PlanningRequest(1, None, state, grid, np.empty((0, 3)), cfg, intent))
    assert response.status == AlgorithmStatus.SUCCESS, response.message
    assert np.linalg.norm(response.selected_goal[:2]) == pytest.approx(8.)
    assert 1 <= response.metadata['candidates_checked'] <= 4
    assert not response.metadata['budget_exhausted']
    manager.on_plan(response)
    assert manager.spec.clockwise == clockwise
    assert manager.spec.orbit_radius == 8.
    assert manager.target_pose == (0., 0., 0.)


# 验证：前半圆有安全候选时，不被更近的上游候选替换。
@pytest.mark.parametrize('clockwise', [True, False])
def test_recovery_keeps_nearer_upstream_arc_out_of_a_nonempty_primary_group(clockwise):
    cfg = resolve_parameters({})
    direction = -1 if clockwise else 1
    state = VesselState(x=8., psi=direction*math.pi/2)
    data = np.zeros((160, 160), np.uint8)
    data[int((direction*3.+40.)/.5), int((7.5+40.)/.5)] = 254
    grid = GridSnapshot(data, .5, -40., -40.)
    manager = MissionManager(MissionSpec('orbit', 0., 0., orbit_radius=8., clockwise=clockwise),
                             (0., 0., 0.), 0., config=cfg)
    manager.request_escape('shorter upstream entry must not displace available primary recovery')
    intent = manager.tick(state, grid, 0.)
    response = plan_navigation(PlanningRequest(1, None, state, grid, np.empty((0, 3)), cfg, intent))
    assert response.status == AlgorithmStatus.SUCCESS, response.message
    progress = (direction*math.atan2(response.selected_goal[1], response.selected_goal[0])) % (2*math.pi)
    assert 0. < progress <= math.pi+1e-9
    assert all((direction*math.atan2(p[1], p[0])) % (2*math.pi) <= math.pi+1e-9
               for p in intent.candidates)
    assert response.metadata['candidates_checked'] <= 4


# 验证：顺逆时针主候选集都包含 pi 边界的对侧点。
@pytest.mark.parametrize('clockwise', [True, False])
@pytest.mark.parametrize('theta', [.17, math.pi-.2, -math.pi+.3])
def test_primary_recovery_includes_pi_boundary_for_either_direction(clockwise, theta):
    cfg = resolve_parameters({})
    state = VesselState(x=8*math.cos(theta), y=8*math.sin(theta))
    grid = GridSnapshot(np.zeros((160, 160), np.uint8), .5, -40., -40.)
    manager = MissionManager(MissionSpec('orbit', 0., 0., orbit_radius=8., clockwise=clockwise),
                             (0., 0., 0.), 0., config=cfg)
    manager.request_escape('pi boundary remains in the primary group')
    intent = manager.tick(state, grid, 0.)
    assert len(intent.candidates) == 12
    assert any(np.allclose(pose[:2], (-state.x, -state.y)) for pose in intent.candidates)
    assert not any(np.allclose(pose[:2], state.pose[:2]) for pose in intent.candidates)


# 验证：从圆心、圆内和圆外均可选出通过验证的切向入圈航路。
@pytest.mark.parametrize('x', [0., 3., 18.])
def test_entry_selects_verified_tangent_route_from_inside_center_and_outside(x):
    manager, job = scenario('orbit', VesselState(x=x))
    response = plan_navigation(job)
    assert response.status == AlgorithmStatus.SUCCESS, response.message
    assert response.selected_goal is not None
    assert np.linalg.norm(response.selected_goal[:2]) == pytest.approx(10.)
    assert response.metadata['candidates_checked'] >= 1
    assert response.metadata['candidate_cost'] >= 0.
    manager.on_plan(response)
    assert manager.temporary_goal == response.selected_goal
    # Runtime uses task speed/precision without mutating the caller's snapshot.
    assert np.max(response.trajectory.points[:, 6]) <= .5+1e-12
    assert job.parameters['dynamics.cruising_speed'] == 1.


# 验证：绕圈参考不复用有限旧航路、不做终点减速。
def test_circle_runtime_does_not_reuse_prune_or_slow_for_old_finite_route():
    _, job = scenario('orbit', VesselState(x=10., psi=-math.pi/2))
    job = replace(job, route=np.array([[10., 0., 0.], [10., -.1, 0.]]))
    response = plan_navigation(job)
    assert response.status == AlgorithmStatus.SUCCESS, response.message
    assert response.trajectory.goal is None
    assert response.route.shape == (0, 3)
    assert not response.searched
    np.testing.assert_allclose(response.trajectory.points[:, 6], .5)


# 验证：未来圆周扫掠出现障碍时规划失败并抢占为退让。
def test_circle_obstacle_on_future_sweep_preempts_into_escape():
    manager, job = scenario('orbit', VesselState(x=10., psi=-math.pi/2))
    data = job.grid.data.copy()
    x, y = 10*math.cos(.7), -10*math.sin(.7)
    ix, iy = job.grid.world_to_map(x, y)
    data[iy, ix] = 254
    blocked = replace(job, grid=GridSnapshot(data, .5, -50., -50.))
    result = plan_navigation(blocked)
    assert result.status == AlgorithmStatus.NO_PATH
    manager.on_plan(result)
    assert manager.tick(job.state, blocked.grid, 0.).kind == 'escape'


# 验证：退让最多验证四个候选，同时保留参与规划的地图时间戳。
def test_exit_search_checks_at_most_four_candidates_and_preserves_map_stamp():
    manager, job = scenario('station_keep', VesselState())
    data = job.grid.data.copy()
    ix, iy = job.grid.world_to_map(3., 0.)
    data[iy, ix] = 254
    grid = GridSnapshot(data, .5, -50., -50.)
    intent = manager.tick(job.state, grid, 0.)
    response = plan_navigation(replace(job, grid=grid, mission_intent=intent))
    assert response.status == AlgorithmStatus.SUCCESS, response.message
    assert 1 <= response.metadata['candidates_checked'] <= 4
    assert response.metadata['map_stamp'] == 0.
    assert response.selected_goal is not None


# 验证：待办绕圈请求不继承普通工作结果中的有限航路。
def test_pending_circle_request_does_not_inherit_route_from_worker_result():
    class Executor:
        def __init__(self):
            self.jobs = []
        def submit(self, fn, request):
            future = Future()
            self.jobs.append((request, future))
            return future
    _, job = scenario('orbit', VesselState(x=10., psi=-math.pi/2))
    executor = Executor()
    worker = LatestPlanWorker(executor)
    worker.offer(job)
    worker.offer(job)
    result = replace(plan_navigation(job), route=np.array([[1., 2., 3.]]))
    executor.jobs[0][1].set_result(result)
    worker.poll()
    assert len(executor.jobs[1][0].route) == 0


# 验证：规划路径终点航向变化不会覆盖用户所需的圆周切向朝向。
def test_selected_circle_goal_keeps_requested_tangent_when_route_heading_differs(monkeypatch):
    import usv_mission.navigation as runtime
    from usv_mission.missions.planning import plan_mission
    _, job = scenario('orbit', VesselState())
    ordinary_navigation = runtime.plan_navigation

    def planner_with_grid_endpoint_heading(request):
        response = ordinary_navigation(request)
        if len(response.route):
            route = response.route.copy()
            route[-1, 2] = request.goal[2]+.7
            response = replace(response, route=route)
        return response

    monkeypatch.setattr(runtime, 'plan_navigation', planner_with_grid_endpoint_heading)
    response = plan_mission(job)
    assert response.status == AlgorithmStatus.SUCCESS
    x, y, yaw = response.selected_goal
    tangent = (math.atan2(y, x)-math.pi/2+math.pi) % (2*math.pi)-math.pi
    assert yaw == pytest.approx(tangent)
    assert response.route[-1, 2] != pytest.approx(yaw)


# 验证：候选终点必须给任意航向保留足迹空间。
def test_exit_candidate_requires_clearance_for_arbitrary_heading():
    data = np.zeros((400, 400), np.uint8)
    data[258, 280] = 254  # (8.0, 5.8): outside aligned envelope, inside turning square.
    grid = GridSnapshot(data, .1, -20., -20.)
    cfg = resolve_parameters({})
    goal = (8., 0., 0.)
    assert pose_is_clear(grid, goal, cfg, margin=5.)
    candidates = escape_candidates(VesselState(), grid, cfg)
    assert not any(np.allclose(pose[:2], goal[:2]) for pose in candidates)


# 验证：转向包络包含扩展船体旋转后的角点。
def test_turning_envelope_contains_rotated_release_rectangle_corners():
    data = np.zeros((400, 400), np.uint8)
    data[200, 275] = 254  # x=[7.5,7.6], y=[0,.1], outside the old 6.253 m square.
    grid = GridSnapshot(data, .1, -20., -20.)
    cfg = resolve_parameters({})
    goal = (0., 0., math.pi/4)
    assert not pose_is_clear(grid, goal, cfg, margin=5.)
    assert not turning_envelope_is_clear(grid, goal, cfg)


# 验证：转向包络还覆盖到达位置容差引入的误差。
def test_turning_envelope_covers_arrival_position_error():
    data = np.zeros((400, 400), np.uint8)
    data[200, 285] = 254  # x=[8.5,8.6]: clear at goal, unsafe .4 m towards the cell.
    grid = GridSnapshot(data, .1, -20., -20.)
    cfg = resolve_parameters({})
    goal = (0., 0., math.atan2(5.6, 6.1))
    assert turning_envelope_is_clear(grid, goal, cfg)
    assert not pose_is_clear(grid, (.4, 0., goal[2]), cfg, margin=5.)
    assert not turning_envelope_is_clear(grid, goal, cfg, arrival_tolerance=.5)


# 验证：到达圆不安全的退让候选在工作进程内再次被拒绝。
def test_worker_rejects_escape_goal_with_unsafe_arrival_disk():
    _, job = scenario('station_keep', VesselState())
    data = job.grid.data.copy()
    ix, iy = job.grid.world_to_map(16.5, 0.)
    data[iy, ix] = 254
    goal = (8., 0., 0.)
    grid = replace(job.grid, data=data)
    intent = replace(job.mission_intent, kind='escape', goal=None, candidates=(goal,))
    result = plan_navigation(replace(job, grid=grid, mission_intent=intent))
    assert result.status == AlgorithmStatus.NO_PATH
    assert result.selected_goal is None


def near_obstacle_escape(speed=.35, yaw=0., obstacle_x=2.3):
    """Synthetic counterpart of global-route-clear / forward-LOS-unsafe failure."""
    cfg = resolve_parameters({})
    data = np.zeros((240, 240), np.uint8)
    axis = (np.arange(240)+.5)*.25-30.
    x, y = np.meshgrid(axis, axis)
    data[(x-obstacle_x)**2+y**2 <= .5**2] = 254
    grid = GridSnapshot(data, .25, -30., -30.)
    state = VesselState(psi=yaw, u=speed)
    manager = MissionManager(MissionSpec('orbit', 0., 0., orbit_radius=8., clockwise=False),
                             (0., 8., 0.), 0., config=cfg)
    manager.request_escape('synthetic regression')
    intent = manager.tick(state, grid, 0.)
    return PlanningRequest(1, None, state, grid, np.empty((0, 3)), cfg, intent)


# 验证：前向 LOS 不安全时可提供已验证制动后对准的条件方案。
def test_escape_can_offer_braking_then_alignment_when_forward_los_is_unsafe():
    job = near_obstacle_escape()
    response = plan_navigation(job)
    assert response.status == AlgorithmStatus.SUCCESS, response.message
    departure = response.metadata['departure_pose']
    model = USVModel()
    stopping, stopped = predict_control_path(job.state, braking_command(job.state, model), model, .1)
    assert stopped
    assert footprint_path_is_clear(job.grid, stopping, 2.4, 1.4)
    np.testing.assert_allclose(departure[:2], stopping[-1, :2])
    assert np.linalg.norm(np.asarray(departure[:2])-np.asarray(job.state.pose[:2])) > .1
    assert footprint_path_is_clear(job.grid, [stopping[-1], departure], 2.4, 1.4)
    assert footprint_path_is_clear(job.grid, response.trajectory.points[:, [2, 3, 4]], 2.2, 1.2)
    assert response.metadata['departure_requires_replan']
    assert response.selected_goal in job.mission_intent.candidates
    assert response.metadata['candidates_checked'] <= 4
    assert job.mission_intent.spec.speed == .5


# 验证：动力学制动尾迹碰撞时不能提供旋转方案。
def test_escape_does_not_offer_rotation_when_dynamic_braking_tail_is_unsafe():
    job = near_obstacle_escape(speed=1.)
    response = plan_navigation(job)
    assert response.status == AlgorithmStatus.NO_PATH
    assert 'departure_pose' not in response.metadata
    assert response.trajectory.empty()


# 验证：已经对准的退让直接使用正常 LOS，不重复前序。
def test_aligned_escape_uses_normal_los_without_repeating_alignment():
    job = near_obstacle_escape(speed=0., yaw=math.pi/2)
    response = plan_navigation(job)
    assert response.status == AlgorithmStatus.SUCCESS, response.message
    assert 'departure_pose' not in response.metadata


# 验证：实测对准后对同一目标重新规划，不继续携带旧条件起步标记。
def test_selected_goal_is_replanned_after_measured_alignment_without_reusing_departure():
    job = near_obstacle_escape()
    first = plan_navigation(job)
    departure = first.metadata['departure_pose']
    observed_stop = VesselState(*departure)
    followup = replace(job, state=observed_stop, goal=first.selected_goal,
                       route=first.route,
                       mission_intent=replace(job.mission_intent, goal=first.selected_goal, candidates=()))
    result = plan_navigation(followup)
    assert result.status == AlgorithmStatus.SUCCESS
    assert result.selected_goal == first.selected_goal
    assert 'departure_pose' not in result.metadata


# 验证：停止位置转向空间不足时先验证短距离倒退再转向。
def test_escape_backs_away_before_turning_when_stopped_hull_has_no_rotation_room():
    job = near_obstacle_escape(speed=.65)
    response = plan_navigation(job)
    assert response.status == AlgorithmStatus.SUCCESS, response.message
    assert response.metadata['backoff_distance'] == .5
    stopped = response.metadata['predicted_braking_stop_pose']
    departure = response.metadata['departure_pose']
    backed = (stopped[0]-.5*math.cos(stopped[2]), stopped[1]-.5*math.sin(stopped[2]), stopped[2])
    np.testing.assert_allclose(departure[:2], backed[:2])
    assert not footprint_path_is_clear(job.grid, [stopped, (*stopped[:2], departure[2])], 2.4, 1.4)
    assert footprint_path_is_clear(job.grid, [stopped, backed], 2.4, 1.4)
    assert footprint_path_is_clear(job.grid, [backed, departure], 2.4, 1.4)
    assert footprint_path_is_clear(job.grid, response.trajectory.points[:, [2, 3, 4]], 2.2, 1.2)
    assert response.metadata['departure_requires_replan']
    assert response.selected_goal in job.mission_intent.candidates
    # A new global reference starts at the backed position, not at the original
    # stop: inheriting the old prefix would immediately lead towards the hazard.
    np.testing.assert_allclose(response.route[0, :2], departure[:2])
    assert response.metadata['candidates_checked'] <= 4


# 验证：船后障碍阻挡时不能提供穿越障碍的倒退方案。
def test_escape_does_not_offer_backoff_through_a_rear_obstacle():
    job = near_obstacle_escape(speed=.65)
    data = job.grid.data.copy()
    ix, iy = job.grid.world_to_map(-1.5, -2.)
    data[iy:iy+16, ix] = 254
    response = plan_navigation(replace(job, grid=replace(job.grid, data=data)))
    assert response.status == AlgorithmStatus.NO_PATH
    assert 'backoff_distance' not in response.metadata
    assert 'departure_pose' not in response.metadata


# 验证：原位可以安全对准时优先使用原位，不做多余倒退。
def test_escape_prefers_original_position_alignment_over_unnecessary_backoff():
    response = plan_navigation(near_obstacle_escape())
    assert response.status == AlgorithmStatus.SUCCESS
    assert 'departure_pose' in response.metadata
    assert 'backoff_distance' not in response.metadata
