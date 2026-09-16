# 建图适配器期限回归：使用真实消息类型和精确模拟墙钟，检查 TF 返回时刻对地图修改的准入。
"""Use the real ROS adapter with exact synthetic clocks; no ROS graph needed."""

from collections import deque
from types import SimpleNamespace

import numpy as np
import pytest

pytest.importorskip('rclpy')
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import LaserScan

import usv_perception.mapping_node as mapping_module
from perception.mapping import GridConfig, GridMapper
from usv_perception.mapping_node import MappingNode
from perception.scan_queue import ScanQueue


def adapter_with_tf(monkeypatch, returned_at, *, invalid_rotation=False):
    clock = [0.19999]
    monkeypatch.setattr(mapping_module, 'time', SimpleNamespace(monotonic=lambda: clock[0]))
    transform = TransformStamped()
    transform.transform.rotation.w = 0.0 if invalid_rotation else 1.0

    def lookup(target, source, stamp, *, timeout):
        assert (target, source) == ('map', 'lidar')
        assert stamp.nanoseconds == 1_000_000_000
        assert timeout.nanoseconds == 0
        clock[0] = returned_at
        return transform

    scan = LaserScan()
    scan.header.stamp.sec = 1
    scan.ranges = [2.0]
    scan.range_min, scan.range_max, scan.scan_time = 0.1, 20.0, 0.1
    queue = ScanQueue()
    queue.enqueue(scan, frame_id='lidar', stamp_ns=1_000_000_000, received_at=0.0)
    adapter = SimpleNamespace(
        tf_buffer=SimpleNamespace(lookup_transform=lookup), map_frame='map', pending=queue,
        mapper=GridMapper(GridConfig(width=8, resolution=1.0)), invalid_scans=0,
        processing_ms=deque(), waiting_ms=deque(), scan_period_ms=deque(),
        scan_overruns=0, integrated_scans=0, invalid_beams=0)
    return adapter, clock


# 验证：TF 虽查询成功但达到或超过期限时不得修改地图。
@pytest.mark.parametrize('returned_at', [0.2, 0.20004235])
def test_successful_tf_at_or_after_deadline_cannot_modify_map(monkeypatch, returned_at):
    adapter, clock = adapter_with_tf(monkeypatch, returned_at)
    before = adapter.mapper.data.copy()
    assert adapter.pending.drain(lambda: clock[0], lambda pending: MappingNode._try_process(adapter, pending)) == 0
    np.testing.assert_array_equal(adapter.mapper.data, before)
    assert adapter.integrated_scans == adapter.pending.counters.processed == 0
    assert adapter.pending.counters.expired == 1
    assert adapter.pending.counters.invalid_consumed == 0
    assert not adapter.waiting_ms


# 验证：非法旋转扫描被消费但不计入成功建图数量。
def test_invalid_consumed_scan_is_not_reported_as_processed(monkeypatch):
    adapter, clock = adapter_with_tf(monkeypatch, 0.19999, invalid_rotation=True)
    before = adapter.mapper.data.copy()
    assert adapter.pending.drain(lambda: clock[0], lambda pending: MappingNode._try_process(adapter, pending)) == 0
    np.testing.assert_array_equal(adapter.mapper.data, before)
    assert adapter.invalid_scans == adapter.pending.counters.invalid_consumed == 1
    assert adapter.integrated_scans == adapter.pending.counters.processed == 0
    assert adapter.pending.counters.expired == 0


# 验证：及时 TF 仍保持原整数证据和准确排队毫秒值。
def test_timely_tf_keeps_original_integer_evidence(monkeypatch):
    adapter, clock = adapter_with_tf(monkeypatch, 0.19999)
    assert adapter.pending.drain(lambda: clock[0], lambda pending: MappingNode._try_process(adapter, pending)) == 1
    assert adapter.mapper.data[4, 6] == 54
    assert adapter.integrated_scans == adapter.pending.counters.processed == 1
    assert adapter.waiting_ms[-1] == pytest.approx(199.99)


# 验证：节点最终过期清理与诊断观察使用完全相同的检查点。
def test_node_final_expiry_and_observation_use_one_checkpoint(monkeypatch):
    times = iter([0.19999, 0.19999, 0.20004235])
    monkeypatch.setattr(mapping_module, 'time', SimpleNamespace(monotonic=lambda: next(times)))
    queue = ScanQueue()
    queue.enqueue(object(), frame_id='lidar', stamp_ns=1, received_at=0.0)
    adapter = SimpleNamespace(pending=queue, _observe_clock=lambda: None,
                              _try_process=lambda pending: False)
    MappingNode._drain(adapter)
    report = queue.observation_summary(0.20004235)
    assert queue.counters.expired == 1
    assert report['last_observed_monotonic_s'] == 0.20004235
    assert report['latest_queue_size'] == 0
    assert report['observation_count'] == report['empty_observations'] == 1
    assert report['oldest_pending_age_ms'] == {'count': 0}
