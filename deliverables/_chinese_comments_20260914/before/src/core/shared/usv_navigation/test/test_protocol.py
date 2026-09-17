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


def test_empty_reference_keeps_task_handling_out_of_trajectory_codec(protocol):
    assert protocol.trajectory_from_values([], 0., '', (8., 9.)) is None


def test_malformed_reference_is_rejected(protocol):
    with pytest.raises(ValueError):
        protocol.trajectory_from_values([[0., 0., 1., 2., 3., 4., float('nan'), 0.]], 2., 'map', (1., 2.))


def test_retired_process_heartbeat_cannot_restore_old_session(protocol):
    peer = protocol.PeerSession()
    assert peer.observe('before', 1, 10.) == 'new'
    assert peer.observe('after', 1, 11.) == 'restart'
    assert peer.observe('before', 2, 12.) == 'ignored'
    assert peer.session == 'after'
    assert peer.last_seen == 11.


def test_repeated_or_out_of_order_heartbeat_does_not_refresh_liveness(protocol):
    peer = protocol.PeerSession()
    peer.observe('live', 4, 10.)
    assert peer.observe('live', 4, 11.) == 'ignored'
    assert peer.observe('live', 3, 12.) == 'ignored'
    assert not peer.fresh(12.)
    assert peer.observe('live', 5, 12.) == 'update'
    assert peer.fresh(12.)


def test_future_or_nonfinite_heartbeat_time_is_not_fresh(protocol):
    peer = protocol.PeerSession()
    peer.observe('live', 1, 10.)
    assert not peer.fresh(9.)
    assert not peer.fresh(float('nan'))


def test_protocol_preserves_public_heartbeat_aliases(protocol):
    from usv_navigation import heartbeat
    assert protocol.PeerSession is heartbeat.PeerSession
    assert protocol.HEARTBEAT_PERIOD == heartbeat.HEARTBEAT_PERIOD
    assert protocol.PEER_TIMEOUT == heartbeat.PEER_TIMEOUT
