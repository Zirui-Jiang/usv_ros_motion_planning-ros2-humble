"""Decision task lifecycle: old results cannot complete a replacement task."""
import importlib
import importlib.util
import pytest

rclpy = pytest.importorskip('rclpy')
from geometry_msgs.msg import PoseStamped
from rclpy.parameter import Parameter
from usv_navigation_interfaces.msg import ControlReport
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


def test_invalid_replacement_clears_existing_task(decision):
    decision._on_goal(goal(decision))
    first = decision.task_id
    decision._on_goal(goal(decision, frame=''))
    assert decision.task_id > first
    assert decision.goal is None
    assert decision.mode == 'INVALID_GOAL'


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


def test_module_restart_requires_a_new_goal(decision):
    decision._on_goal(goal(decision))
    task = decision.task_id
    decision._on_peer_restart('perception')
    assert decision.goal is None
    assert decision.task_id > task
    assert decision.mode == 'MODULE_RESTART'
