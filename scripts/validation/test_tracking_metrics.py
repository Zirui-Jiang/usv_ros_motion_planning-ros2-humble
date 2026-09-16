# 验证源时间误差积分、重复时间戳去重与覆盖边界；观测不完整时必须返回未知。
"""The source-time protocol is fixed before the next navigation matrix."""
import importlib.util
import math
from pathlib import Path

import pytest

_spec = importlib.util.spec_from_file_location("tracking_metrics", Path(__file__).with_name("tracking_metrics.py"))
tracking = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(tracking)


def source_events():
    # Nonuniform source spacing: the explicit discrete e^2 sum is 1.375.
    return [
        {"event": "odom", "wall_monotonic_s": 9.9, "sim_time_s": 0., "pose": [0., 0., 0.], "frame_id": "otter/odom", "child_frame_id": "otter/base_link"},
        {"event": "goal_published", "wall_monotonic_s": 10., "sim_time_s": 0.},
        {"event": "reference_path", "wall_monotonic_s": 10.1, "sim_time_s": -100., "frame_id": "otter/map", "points": [[0., 0., 0.], [10., 0., 0.]]},
        {"event": "odom", "wall_monotonic_s": 10.2, "sim_time_s": .25, "pose": [1., 1., 0.], "frame_id": "otter/odom", "child_frame_id": "otter/base_link"},
        {"event": "odom", "wall_monotonic_s": 10.7, "sim_time_s": .75, "pose": [2., 2., 0.], "frame_id": "otter/odom", "child_frame_id": "otter/base_link"},
        {"event": "acquisition_finished", "wall_monotonic_s": 10.8, "sim_time_s": .8},
    ]


def measure(events=None, contract="otter_fake_localization"):
    return tracking.source_time_trapezoid(source_events() if events is None else events, frame_contract=contract)


# 非均匀源采样应按时间积分平方误差，不能对样本简单等权求均方根。
def test_nonuniform_squared_error_trapezoids_not_equal_sample_rms():
    observed = measure()
    assert observed["status"] == "valid"
    assert observed["integrated_squared_error_m2_s"] == pytest.approx(1.375)
    assert observed["rmse_m"] == pytest.approx(math.sqrt(1.375 / .75))
    assert observed["max_source_gap_s"] == .5
    assert observed["source_span_s"] == .75
    assert observed["end_coverage_gap_s"] == pytest.approx(.05)
    assert observed["sample_count"] == 3
    assert observed["error_p95_m"] == observed["error_max_m"] == 2.


# 重复源时间去重，墙钟暂停不能凭空增加同一状态的积分权重。
def test_same_pose_stamp_deduplicated_and_wall_pause_cannot_change_integral():
    events = source_events()
    events[4:4] = [dict(events[3], wall_monotonic_s=10.3), dict(events[3], wall_monotonic_s=10.5)]
    answer = measure(events)
    assert answer["rmse_m"] == measure()["rmse_m"]
    assert answer["duplicate_stamp_count"] == 2
    assert answer["raw_odom_count"] == 5


# 首条路径按接收顺序选取，同时保留路径到来前已经观测到的运动。
def test_first_path_uses_receipt_order_and_all_prepath_motion_is_retained():
    events = source_events()
    events.insert(4, dict(events[2], wall_monotonic_s=10.4, sim_time_s=-200., points=[[0., 99.], [10., 99.]]))
    answer = measure(events)
    assert answer["rmse_m"] == measure()["rmse_m"]
    assert answer["first_path_event_line"] == 3
    assert answer["source_start_s"] == 0.


# 无效或覆盖不足的证据返回未知，不补零或外推。
@pytest.mark.parametrize("kind", ["conflicting_stamp", "rollback", "nonfinite_pose", "missing_stamp", "missing_pose", "missing_frame", "missing_child", "source_gap", "no_pregoal", "no_end", "no_goal_stamp", "start_coverage", "end_coverage", "no_span", "unknown_frame", "multiple_goal"])
def test_bad_or_incomplete_evidence_is_unknown(kind):
    events = source_events()
    if kind == "conflicting_stamp": events[4]["sim_time_s"] = .25
    elif kind == "rollback": events[4]["sim_time_s"] = .2
    elif kind == "nonfinite_pose": events[3]["pose"][0] = float("nan")
    elif kind == "missing_stamp": del events[3]["sim_time_s"]
    elif kind == "missing_pose": del events[3]["pose"]
    elif kind == "missing_frame": del events[3]["frame_id"]
    elif kind == "missing_child": del events[3]["child_frame_id"]
    elif kind == "source_gap": events[4]["sim_time_s"] = .750001
    elif kind == "no_pregoal": events.pop(0)
    elif kind == "no_end": events[-1]["event"] = "trial_finished"
    elif kind == "no_goal_stamp": del events[1]["sim_time_s"]
    elif kind == "start_coverage": events[1]["sim_time_s"] = .500001
    elif kind == "end_coverage": events[-1]["sim_time_s"] = 1.250001
    elif kind == "no_span":
        for i in (3, 4): events[i].update(sim_time_s=0., pose=[0., 0., 0.])
    elif kind == "unknown_frame": events[3]["frame_id"] = "another/odom"
    elif kind == "multiple_goal": events.insert(4, dict(events[1], wall_monotonic_s=10.4))
    answer = measure(events)
    assert answer["status"] == "unknown", (kind, answer)
    assert answer["rmse_m"] is None
    assert answer["reasons"]


# 未显式选择坐标系契约时，不能自动假定 map 与 odom 相同。
def test_contract_is_never_implicitly_assumed():
    assert measure(contract=None)["status"] == "unknown"
    events = source_events()
    for event in events:
        if event.get("frame_id"): event["frame_id"] = "map"
    assert measure(events)["status"] == "unknown"


# 恰好半秒的覆盖边界按协议接纳，积分区间仍不超出实际观测。
def test_boundary_half_second_is_inclusive_and_no_extrapolation():
    events = source_events()
    events[1]["sim_time_s"] = .5
    events[-1]["sim_time_s"] = 1.25
    answer = measure(events)
    assert answer["status"] == "valid"
    assert answer["source_span_s"] == .75
    assert answer["start_coverage_gap_s"] == answer["end_coverage_gap_s"] == .5


# 剔除采集结束后的观测和不属于本次目标窗口的旧样本。
def test_observations_after_acquisition_and_older_pregoal_are_excluded():
    events = source_events()
    events.insert(0, dict(events[0], wall_monotonic_s=8., sim_time_s=-10., pose=[0., 100., 0.]))
    events.append(dict(events[4], wall_monotonic_s=11., pose=[0., 100., 0.]))
    assert measure(events)["rmse_m"] == measure()["rmse_m"]


# 在线段延长线上的点应计算到端点的距离，而不是错误地得到零。
def test_polyline_endpoint_distance_is_not_infinite_line_distance():
    events = source_events()
    for event in events:
        if event["event"] == "odom": event["pose"] = [-3., 4., 0.]
    assert measure(events)["rmse_m"] == 5.


# 用有解析积分的平方误差函数检验非均匀梯形积分数值。
def test_linear_squared_error_has_known_integral_on_nonuniform_grid():
    events = source_events()
    for event in events:
        if event["event"] == "odom":
            event["pose"] = [1., math.sqrt(2. * event["sim_time_s"] + 1.), 0.]
    # Integral_0^.75 (2t+1) dt = .75^2 + .75, for e^2 linear.
    answer = measure(events)
    assert answer["integrated_squared_error_m2_s"] == pytest.approx(.75 ** 2 + .75)
    assert answer["rmse_m"] == pytest.approx(math.sqrt(1.75))


# 坐标系契约要同时保留历史源码和运行日志的身份哈希。
def test_declared_frame_contract_contains_both_audited_source_and_log_hashes():
    evidence = tracking.frame_contract_description("otter_fake_localization")["verified_sources"]
    assert {entry["mode"] for entry in evidence} == {"cpp", "python"}
    for entry in evidence:
        assert len(entry["source_sha256"]) == len(entry["launch_log_sha256"]) == 64
        assert entry["logged_frame_chain"] == ["otter/map", "otter/odom", "otter/base_link"]
