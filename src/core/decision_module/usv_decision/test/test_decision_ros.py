# 验证决策目标生命周期，确保旧完成报告和旧规划版本不影响替换后的任务。
"""Decision task lifecycle: old results cannot complete a replacement task."""
import importlib
import importlib.util
import pytest

rclpy = pytest.importorskip('rclpy')
from geometry_msgs.msg import PoseStamped
from rclpy.parameter import Parameter
from usv_navigation_interfaces.msg import ControlReport
from usv_navigation_interfaces.srv import NavigateTo
from usv_navigation.configuration import T


@pytest.fixture
def decision():
    assert importlib.util.find_spec('usv_decision.decision') is not None, 'Decision node is missing'
    rclpy.init(args=['--ros-args', '-r', '__ns:=/decision_test'])
    node = importlib.import_module('usv_decision.decision').DecisionNode()
    try:
        yield node
    finally:
        node.destroy_node()
        rclpy.shutdown()


def goal(node, x=5., frame=None):
    message = PoseStamped()
    message.header.frame_id = node.frame if frame is None else frame
    message.pose.position.x = x
    message.pose.orientation.w = 1.
    return message


def navigation_request(node, request_id='travel-53', x=5., speed=.4):
    request = NavigateTo.Request()
    request.request_id = request_id
    request.goal = goal(node, x=x)
    request.desired_speed_mps = speed
    return request


def navigate(node, request):
    return node._navigate(request, NavigateTo.Response())


# 新服务原子接受任务；相同请求幂等返回原任务号，不重新执行，冲突请求不改状态。
def test_atomic_navigation_is_idempotent_and_rejects_conflicts(decision):
    request = navigation_request(decision)
    accepted = navigate(decision, request)
    assert accepted.accepted and accepted.reason == 'ACCEPTED'
    assert accepted.planner_session == decision.session
    assert accepted.task_id == decision.accepted_task_id == decision.task_id
    snapshot = (decision.version, decision.task_id, decision.goal)

    repeated = navigate(decision, request)
    assert repeated.accepted and repeated.reason == 'IDEMPOTENT'
    assert repeated.task_id == accepted.task_id
    assert (decision.version, decision.task_id, decision.goal) == snapshot

    conflict = navigation_request(decision, x=6.)
    rejected = navigate(decision, conflict)
    assert not rejected.accepted and 'different parameters' in rejected.reason
    assert (decision.version, decision.task_id, decision.goal) == snapshot


# 活动目标拒绝不同请求；超速在任何状态变更前拒绝，旧 goal 入口仍可替换并清关联。
def test_busy_speed_validation_and_legacy_goal_compatibility(decision):
    before = (decision.version, decision.task_id, decision.goal)
    too_fast = navigate(decision, navigation_request(
        decision, request_id='too-fast', speed=decision.model.velocity_max + .01))
    assert not too_fast.accepted and 'velocity_max' in too_fast.reason
    assert (decision.version, decision.task_id, decision.goal) == before

    accepted = navigate(decision, navigation_request(decision))
    busy = navigate(decision, navigation_request(decision, request_id='travel-54', x=7.))
    assert not busy.accepted and busy.reason.startswith('BUSY')
    assert decision.task_id == accepted.task_id and decision.goal == (5., 0., 0.)

    decision._on_goal(goal(decision, x=8.))
    assert decision.goal == (8., 0., 0.)
    assert decision.request_id == '' and decision.accepted_task_id == 0
    assert decision.desired_speed_mps == decision.model.cruising_speed


# 完成、取消或模块重启都保留外部关联；完成后的重传仍不启动第二次任务。
def test_terminal_states_keep_external_request_correlation(decision):
    decision.peers['control'].observe('controller', 1, 0.)
    request = navigation_request(decision)
    accepted = navigate(decision, request)
    report = ControlReport(control_session='controller', planner_session=decision.session,
                           task_id=decision.task_id, task_finished=True, state='GOAL_REACHED')
    decision._on_control_report(report)
    assert decision.mode == 'GOAL_REACHED' and decision.goal is None
    assert decision.request_id == request.request_id
    assert decision.accepted_task_id == accepted.task_id
    assert decision.desired_speed_mps == request.desired_speed_mps
    terminal = (decision.version, decision.task_id)
    repeated = navigate(decision, request)
    assert repeated.accepted and repeated.reason == 'IDEMPOTENT'
    assert (decision.version, decision.task_id) == terminal

    decision._reset_task('CANCELLED')
    assert decision.request_id == request.request_id
    decision._on_peer_restart('perception')
    assert decision.request_id == request.request_id


# 非法替换目标清掉当前目标并增加任务号。
def test_invalid_replacement_clears_existing_task(decision):
    decision._on_goal(goal(decision))
    first = decision.task_id
    decision._on_goal(goal(decision, frame=''))
    assert decision.task_id > first
    assert decision.goal is None
    assert decision.mode == 'INVALID_GOAL'


# 旧任务完成报告不能清除新目标，当前任务完成可正常结束。
def test_old_completion_does_not_clear_new_goal(decision):
    decision.peers['control'].observe('controller', 1, 0.)
    decision._on_goal(goal(decision))
    first = decision.task_id
    decision._on_goal(goal(decision, x=8.))
    report = ControlReport(control_session='controller', planner_session=decision.session,
                           task_id=first, task_finished=True, state='GOAL_REACHED')
    decision._on_control_report(report)
    assert decision.goal == (8., 0., 0.)
    report.task_id = decision.task_id
    decision._on_control_report(report)
    assert decision.goal is None
    assert decision.mode == 'GOAL_REACHED'


# LOS 热更新增加版本并失效化参考，同时保留任务号。
def test_los_hot_update_invalidates_reference_without_replacing_task(decision):
    decision._on_goal(goal(decision))
    task, version = decision.task_id, decision.version
    result = decision.set_parameters([Parameter(T + 'los.lookahead_distance', value=2.)])
    assert result[0].successful
    decision._apply_parameters()
    assert decision.task_id == task
    assert decision.version == version + 1
    assert decision.goal == (5., 0., 0.)
    assert decision.trajectory is None


# 模块重启清理任务且要求新目标。
def test_module_restart_requires_a_new_goal(decision):
    decision._on_goal(goal(decision))
    task = decision.task_id
    decision._on_peer_restart('perception')
    assert decision.goal is None
    assert decision.task_id > task
    assert decision.mode == 'MODULE_RESTART'
