# 验证轨迹字段无损传输、心跳拒旧及退休进程不可重放恢复。
"""Detect lost trajectory fields, stale heartbeats and retired process replay."""
import importlib
import importlib.util
import numpy as np
import pytest
from trajectory_planner.trajectory import Trajectory


@pytest.fixture
def protocol():
    assert importlib.util.find_spec('usv_navigation.protocol') is not None, 'Distributed protocol is missing'
    return importlib.import_module('usv_navigation.protocol')


# 完整轨迹往返保留八列、源时间、坐标系、目标及只读标志。
def test_full_reference_roundtrip_preserves_source_time_and_goal(protocol):
    points = [[0., 0., 3., 4., .3, .04, 1.2, .1],
              [.5, .6, 3.5, 4.1, .4, .05, 1.1, -.2]]
    original = Trajectory(points, 123.125, 'otter/map', (8., 9.))
    values = protocol.trajectory_to_values(original)
    restored = protocol.trajectory_from_values(**values)
    np.testing.assert_array_equal(restored.points, points)
    assert restored.stamp == 123.125
    assert restored.frame_id == 'otter/map'
    assert restored.goal == (8., 9.)
    assert not restored.points.flags.writeable


# 空参考只编码为 None，不在编解码层改变任务。
def test_empty_reference_keeps_task_handling_out_of_trajectory_codec(protocol):
    assert protocol.trajectory_from_values([], 0., '', (8., 9.)) is None


# 非有限轨迹消息必须被构造校验拒绝。
def test_malformed_reference_is_rejected(protocol):
    with pytest.raises(ValueError):
        protocol.trajectory_from_values([[0., 0., 1., 2., 3., 4., float('nan'), 0.]], 2., 'map', (1., 2.))


# 已退休进程的迟到心跳不能恢复旧会话。
def test_retired_process_heartbeat_cannot_restore_old_session(protocol):
    peer = protocol.PeerSession()
    assert peer.observe('before', 1, 10.) == 'new'
    assert peer.observe('after', 1, 11.) == 'restart'
    assert peer.observe('before', 2, 12.) == 'ignored'
    assert peer.session == 'after'
    assert peer.last_seen == 11.


# 重复乱序心跳不刷新存活时间，新序号可刷新。
def test_repeated_or_out_of_order_heartbeat_does_not_refresh_liveness(protocol):
    peer = protocol.PeerSession()
    peer.observe('live', 4, 10.)
    assert peer.observe('live', 4, 11.) == 'ignored'
    assert peer.observe('live', 3, 12.) == 'ignored'
    assert not peer.fresh(12.)
    assert peer.observe('live', 5, 12.) == 'update'
    assert peer.fresh(12.)


# 未来或非有限心跳时间不能视为新鲜。
def test_future_or_nonfinite_heartbeat_time_is_not_fresh(protocol):
    peer = protocol.PeerSession()
    peer.observe('live', 1, 10.)
    assert not peer.fresh(9.)
    assert not peer.fresh(float('nan'))


# 公开协议中的心跳类型和周期/超时别名与轻量模块一致。
def test_protocol_preserves_public_heartbeat_aliases(protocol):
    from usv_navigation import heartbeat
    assert protocol.PeerSession is heartbeat.PeerSession
    assert protocol.HEARTBEAT_PERIOD == heartbeat.HEARTBEAT_PERIOD
    assert protocol.PEER_TIMEOUT == heartbeat.PEER_TIMEOUT


def test_pass_through_reference_has_no_controller_terminal_goal(protocol):
    from types import SimpleNamespace
    values = dict(zip(protocol.POINT_FIELDS, [0., 0., 3., 4., 0., 0., 1., 0.]))
    message = SimpleNamespace(points=[SimpleNamespace(**values)],
        trajectory_stamp=1., trajectory_frame='otter/map',
        goal=SimpleNamespace(x=3., y=4.), pass_through=True)
    assert protocol.trajectory_from_message(message).goal is None
    message.pass_through = False
    assert protocol.trajectory_from_message(message).goal == (3., 4.)


def test_missing_terminal_goal_still_requires_explicit_pass_through(protocol):
    points = [[0., 0., 3., 4., 0., 0., 1., 0.]]
    with pytest.raises(ValueError):
        protocol.trajectory_from_values(points, 1., 'map', None)
    assert protocol.trajectory_from_values(points, 1., 'map', None, pass_through=True).goal is None


def test_pass_through_roundtrip_preserves_nonterminal_semantics(protocol):
    original = Trajectory([[0., 0., 3., 4., 0., 0., 1., 0.]], 1., 'map', None)
    restored = protocol.trajectory_from_values(**protocol.trajectory_to_values(original))
    assert restored.goal is None
    np.testing.assert_array_equal(restored.points, original.points)


# 航线折线随参考往返：值编码保留全部点，消息里拆成等长的 x/y 两个数组。
def test_route_polyline_roundtrip_through_values_and_message(protocol):
    from types import SimpleNamespace
    original = Trajectory([[0., 0., 3., 4., 0., 0., 1., 0.]], 1., 'map', (8., 9.),
                          route=[[3., 4.], [5.5, 4.], [8., 9.]])
    restored = protocol.trajectory_from_values(**protocol.trajectory_to_values(original))
    np.testing.assert_array_equal(restored.route, original.route)
    assert not restored.route.flags.writeable
    values = dict(zip(protocol.POINT_FIELDS, [0., 0., 3., 4., 0., 0., 1., 0.]))
    message = SimpleNamespace(points=[SimpleNamespace(**values)], trajectory_stamp=1., trajectory_frame='map',
                              goal=SimpleNamespace(x=8., y=9.), route_x=[3., 5.5, 8.], route_y=[4., 4., 9.])
    np.testing.assert_array_equal(protocol.trajectory_from_message(message).route, original.route)
    # 旧消息没有航线字段时得到空航线，而不是报错。
    del message.route_x, message.route_y
    assert protocol.trajectory_from_message(message).route.shape == (0, 2)


# x/y 数组不等长或含非有限数的航线必须拒绝，不能悄悄截断。
@pytest.mark.parametrize('route_x, route_y', [([1., 2.], [1.]), ([1., float('inf')], [1., 2.])])
def test_malformed_route_message_is_rejected(protocol, route_x, route_y):
    from types import SimpleNamespace
    values = dict(zip(protocol.POINT_FIELDS, [0., 0., 3., 4., 0., 0., 1., 0.]))
    message = SimpleNamespace(points=[SimpleNamespace(**values)], trajectory_stamp=1., trajectory_frame='map',
                              goal=SimpleNamespace(x=8., y=9.), route_x=route_x, route_y=route_y)
    with pytest.raises(ValueError, match='route'):
        protocol.trajectory_from_message(message)


# 航线窗口以最近路点为中心：向前、向后各取到首次达到给定弧长的那一点，并受点数上限约束。
def test_route_window_keeps_the_part_of_the_route_near_the_vessel(protocol):
    route = [[float(x), 0., 0.] for x in range(21)]
    window = protocol.route_window(route, (7.2, 3.), ahead=4.5, behind=2.)
    np.testing.assert_array_equal(window[:, 0], [5., 6., 7., 8., 9., 10., 11., 12.])
    assert window.shape[1] == 2
    # 航线比窗口短时原样返回；最近点在末端时只剩向后的部分。
    np.testing.assert_array_equal(protocol.route_window(route[:3], (0., 0.), ahead=50., behind=50.)[:, 0], [0., 1., 2.])
    np.testing.assert_array_equal(protocol.route_window(route, (99., 0.), ahead=5., behind=1.5)[:, 0], [18., 19., 20.])
    assert len(protocol.route_window(route, (0., 0.), ahead=100., limit=6)) == 6
    assert protocol.route_window([], (0., 0.), ahead=5.).shape == (0, 2)
    np.testing.assert_array_equal(protocol.route_window([[4., 5., 0.]], (0., 0.), ahead=5.), [[4., 5.]])


@pytest.mark.parametrize('arguments', [
    dict(route=[[0., float('nan'), 0.]], position=(0., 0.), ahead=1.),
    dict(route=[[0., 0., 0.]], position=(float('inf'), 0.), ahead=1.),
    dict(route=[[0., 0., 0.]], position=(0., 0.), ahead=-1.),
    dict(route=[[0., 0., 0.]], position=(0., 0.), ahead=1., limit=1),
])
def test_route_window_rejects_invalid_input(protocol, arguments):
    with pytest.raises(ValueError):
        protocol.route_window(**arguments)
