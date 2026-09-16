# 按真实源时间积分平方横向误差；不外推、不平滑，也不以重复墙钟采样代替新观测。
"""Finite-observation tracking scores; no ROS, clock extrapolation or smoothing.

The 0.5 s source-gap and boundary tolerances are protocol constants, fixed before
the next matrix from the existing navigation state-freshness requirement.
"""
import hashlib
import importlib.util
import json
import math
from pathlib import Path

_spec = importlib.util.spec_from_file_location("acceptance_geometry", Path(__file__).with_name("metrics.py"))
_geometry = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(_geometry)

RMSE_METRICS = ("legacy_wall", "source_time_trapezoid")
MAX_SOURCE_GAP_S = .5
MAX_BOUNDARY_GAP_S = .5
FRAME_CONTRACTS = ("otter_fake_localization",)
# Audited immutable source/log evidence, not a claim that arbitrary frames with
# these names are identical. CLI selection is the caller's project assertion.
# 以下哈希和行号属于列出的历史提交与日志，不应随当前文件新增注释而改写。
FRAME_EVIDENCE = [
    {"mode": "cpp", "git_commit": "632734ba31478bdb833e15949287b53ae9334cf3",
     "source": "src/sim_env/src/fake_localization_node.cpp", "source_lines": [89, 94],
     "source_sha256": "b3f0e7a47b1328787c9fba138b0ec04701b679e6951828ed4a653df176789a4e",
     "identity_evidence": "Default zero translation and rotation.w=1; map_frame to odom_frame; odom header frame at line 114.",
     "launch_log": "/home/usv/usv_migration/results/baseline-grid-interior-001/cpp_pid_straight_01/launch.log",
     "launch_log_line": 10, "launch_log_sha256": "113d5b195135cd9366886196b22a4b8dc4ab7c30b5d7a967c57eab94d4d2bcc1",
     "logged_frame_chain": ["otter/map", "otter/odom", "otter/base_link"]},
    {"mode": "python", "git_commit": "e38183218fce7044e686a0f697ed13f456f49039",
     "source": "src/sim_env/src/fake_localization_node.cpp", "source_lines": [91, 96],
     "source_sha256": "5f9f2bf08715742c1384c22c7c932ec2f3d0630ffbd16ad15103f48d54aaac5e",
     "identity_evidence": "Default zero translation and rotation.w=1; map_frame to odom_frame; odom header frame at line 116.",
     "launch_log": "/home/usv/usv_migration/results/python-release-grid-001/python_pid_straight_01/launch.log",
     "launch_log_line": 20, "launch_log_sha256": "b028adf5e279fbc28e68571b33e657608d3d57042923ffdb1f1bade31ec5b08f",
     "logged_frame_chain": ["otter/map", "otter/odom", "otter/base_link"]},
]


# 区分有效有限数值与缺失、NaN、无穷和布尔值，避免污染统计结果。
def finite(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


# 明确选择经过审计的 map/odom 恒等变换前提；仅帧名相同不足以证明坐标一致。
def frame_contract_description(name):
    if name is None:
        return {"name": None, "status": "not_selected", "scope": "No map/odom identity is assumed."}
    if name not in FRAME_CONTRACTS:
        raise ValueError("unsupported frame contract: " + str(name))
    return {"name": name, "status": "explicit_project_assertion",
            "scope": "Only the audited otter fake-localization project: map->odom is identity; this does not establish identity for arbitrary recorded data.",
            "path_frame": "otter/map", "odom_frames": ["otter/odom", "otter/map"],
            "child_frame": "otter/base_link", "verified_sources": FRAME_EVIDENCE,
            "version_scope": "These are the audited baseline/e381 implementations and logs. For later source versions the explicit caller assertion requires confirming this unchanged transform contract; matching frame strings alone is not proof."}


# 先按事件接收顺序选取观测，再按唯一源时间积分误差平方，并检查首尾与相邻覆盖。
def source_time_trapezoid(events, *, frame_contract=None):
    """Integrate squared distances between consecutive unique source observations.

    Select by event receipt order, including the latest pre-goal odometry. Use
    the actual first/last odometry source timestamps as integration endpoints.
    Boundary events provide coverage checks only; never extrapolate to them.
    P95/max describe errors at unique observed poses, not continuous extrema.
    """
    contract = frame_contract_description(frame_contract)
    result = {"metric": "source_time_trapezoid", "status": "unknown", "reasons": [],
              "rmse_m": None, "integrated_squared_error_m2_s": None,
              "sample_count": 0, "raw_odom_count": 0, "duplicate_stamp_count": 0,
              "source_start_s": None, "source_end_s": None, "source_span_s": None,
              "goal_source_s": None, "acquisition_source_s": None,
              "start_offset_s": None, "end_offset_s": None,
              "start_coverage_gap_s": None, "end_coverage_gap_s": None,
              "max_source_gap_s": None, "max_allowed_source_gap_s": MAX_SOURCE_GAP_S,
              "max_allowed_boundary_gap_s": MAX_BOUNDARY_GAP_S,
              "error_p95_m": None, "error_max_m": None,
              "first_path_event_line": None, "first_path_sha256": None,
              "frame_contract": contract["name"], "path_frame": None, "odom_frames": [],
              "end_event": "acquisition_finished", "boundary_extrapolation": False,
              "percentile_population": "nearest-rank P95 of unique observed source-pose errors; not time-weighted or continuous extrema"}

    def unknown(reason):
        result["reasons"].append(reason)
        return result

    if frame_contract is None:
        return unknown("explicit_frame_contract_required")
    if not isinstance(events, list) or any(not isinstance(e, dict) for e in events):
        return unknown("events_must_be_objects_in_receipt_order")
    goals = [i for i, event in enumerate(events) if event.get("event") == "goal_published"]
    if not goals:
        return unknown("goal_event_missing")
    start = goals[0]
    ends = [i for i in range(start + 1, len(events)) if events[i].get("event") == "acquisition_finished"]
    if not ends:
        return unknown("acquisition_finished_missing_no_source_time_fallback")
    end = ends[0]
    if any(start < i < end for i in goals):
        return unknown("multiple_goals_in_tracking_window")
    goal, finish = events[start], events[end]
    if any(not finite(e.get(key)) for e in (goal, finish) for key in ("sim_time_s", "wall_monotonic_s")):
        return unknown("boundary_source_or_receipt_timestamp_missing_or_nonfinite")
    result.update(goal_source_s=goal["sim_time_s"], acquisition_source_s=finish["sim_time_s"])
    if finish["sim_time_s"] <= goal["sim_time_s"] or finish["wall_monotonic_s"] <= goal["wall_monotonic_s"]:
        return unknown("boundary_clock_rollback_or_insufficient_span")
    path_indices = [i for i in range(start + 1, end) if events[i].get("event") == "reference_path" and events[i].get("points")]
    if not path_indices:
        return unknown("first_nonempty_path_missing")
    path_index = path_indices[0]
    path_event, path = events[path_index], events[path_index]["points"]
    result.update(first_path_event_line=path_index + 1, path_frame=path_event.get("frame_id") if isinstance(path_event.get("frame_id"), str) else None)
    if not isinstance(path, list) or any(not isinstance(p, (list, tuple)) or len(p) < 2 or any(not finite(v) for v in p) for p in path):
        return unknown("first_path_coordinates_invalid")
    if path_event.get("frame_id") != contract["path_frame"]:
        return unknown("path_frame_outside_explicit_contract")
    if not finite(path_event.get("wall_monotonic_s")) or not goal["wall_monotonic_s"] <= path_event["wall_monotonic_s"] <= finish["wall_monotonic_s"]:
        return unknown("first_path_receipt_time_outside_window")
    result["first_path_sha256"] = hashlib.sha256(json.dumps(path, separators=(",", ":"), allow_nan=False).encode()).hexdigest()
    preceding = [i for i in range(start) if events[i].get("event") == "odom"]
    if not preceding:
        return unknown("latest_pregoal_odometry_missing")
    indices = preceding[-1:] + [i for i in range(start + 1, end) if events[i].get("event") == "odom"]
    result["raw_odom_count"] = len(indices)
    samples, previous_wall = [], None
    for i in indices:
        event = events[i]
        pose, stamp, wall = event.get("pose"), event.get("sim_time_s"), event.get("wall_monotonic_s")
        if not (isinstance(pose, (list, tuple)) and len(pose) >= 3 and all(finite(v) for v in pose)
                and finite(stamp) and finite(wall)):
            return unknown(f"invalid_odometry_pose_or_timestamp_at_line_{i + 1}")
        if event.get("frame_id") not in contract["odom_frames"] or event.get("child_frame_id") != contract["child_frame"]:
            return unknown(f"odometry_frame_outside_explicit_contract_at_line_{i + 1}")
        if event["frame_id"] not in result["odom_frames"]:
            result["odom_frames"].append(event["frame_id"])
        if previous_wall is not None and wall < previous_wall:
            return unknown(f"odometry_receipt_clock_rollback_at_line_{i + 1}")
        previous_wall = wall
        if i < start and wall > goal["wall_monotonic_s"] or i > start and not goal["wall_monotonic_s"] <= wall <= finish["wall_monotonic_s"]:
            return unknown(f"odometry_receipt_time_conflicts_with_event_order_at_line_{i + 1}")
        if samples and stamp < samples[-1][0]:
            return unknown(f"source_clock_rollback_at_line_{i + 1}")
        if samples and stamp == samples[-1][0]:
            if list(pose) != samples[-1][1]:
                return unknown(f"same_source_stamp_conflicting_pose_at_line_{i + 1}")
            result["duplicate_stamp_count"] += 1
            continue
        samples.append((stamp, list(pose)))
    result["sample_count"] = len(samples)
    if len(samples) < 2 or samples[-1][0] <= samples[0][0]:
        return unknown("insufficient_distinct_source_span")
    first, last = samples[0][0], samples[-1][0]
    start_offset, end_offset = first - goal["sim_time_s"], last - finish["sim_time_s"]
    gaps = [b[0] - a[0] for a, b in zip(samples, samples[1:])]
    result.update(source_start_s=first, source_end_s=last, source_span_s=last - first,
                  start_offset_s=start_offset, end_offset_s=end_offset,
                  start_coverage_gap_s=abs(start_offset), end_coverage_gap_s=abs(end_offset),
                  max_source_gap_s=max(gaps))
    errors = [_geometry.point_to_polyline_distance(pose, path) for _, pose in samples]
    if not all(finite(value) for value in errors):
        return unknown("distance_computation_nonfinite")
    result.update(error_p95_m=sorted(errors)[math.ceil(.95 * len(errors)) - 1], error_max_m=max(errors))
    # 1e-12 absorbs binary representation at exactly 0.5, not data-scale slack.
    if max(gaps) > MAX_SOURCE_GAP_S + 1e-12:
        return unknown("source_interval_exceeds_frozen_0.5s_limit")
    if max(abs(start_offset), abs(end_offset)) > MAX_BOUNDARY_GAP_S + 1e-12:
        return unknown("boundary_coverage_exceeds_frozen_0.5s_limit")
    energy = sum(dt * (a * a + b * b) / 2 for dt, a, b in zip(gaps, errors, errors[1:]))
    if not finite(energy):
        return unknown("squared_error_integral_nonfinite")
    result.update(status="valid", rmse_m=math.sqrt(energy / (last - first)),
                  integrated_squared_error_m2_s=energy)
    return result


# 并列保留旧墙钟和源时间误差结果，避免修改统计定义后覆盖历史值。
def tracking_scores(events, summary, *, frame_contract=None):
    legacy = summary.get("cross_track_initial_m") or {}
    value = legacy.get("rms")
    return {"legacy_wall": {"metric": "legacy_wall", "status": "valid" if finite(value) and value >= 0 else "unknown",
                            "rmse_m": value if finite(value) and value >= 0 else None,
                            "sample_count": legacy.get("count"), "error_p95_m": legacy.get("p95"), "error_max_m": legacy.get("max"),
                            "definition": "Unchanged summary cross_track_initial_m.rms of wall-loop latest-pose samples, including repeated source stamps."},
            "source_time_trapezoid": source_time_trapezoid(events, frame_contract=frame_contract)}
