"""Acceptance metric regressions; synthetic inputs here are unit tests only."""
import importlib.util
import json
import math
from pathlib import Path
from types import SimpleNamespace

import pytest

_spec = importlib.util.spec_from_file_location("mission_acceptance", Path(__file__).with_name("mission_acceptance.py"))
acceptance = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(acceptance)


def sample(t, position=0.1, heading=1.0, speed=0.01):
    return {"sim_time_s": t, "position_error_m": position,
            "heading_error_deg": heading, "speed_mps": speed}


def test_fixed_window_cannot_restart_after_an_excursion():
    rows = [sample(i/10) for i in range(1201)]
    for i in range(400, 441):
        rows[i]["position_error_m"] = 0.6
    score = acceptance.fixed_window_metrics(rows, 0)
    assert score["complete"]
    assert score["sample_count"] == 601
    assert score["joint_fraction"] < 0.95
    assert not score["passed"]
    assert acceptance.fixed_window_metrics(rows, 60)["passed"]


def test_joint_position_heading_threshold_is_not_two_marginal_rates():
    rows = [sample(i/10) for i in range(601)]
    for row in rows[10:30]:
        row["position_error_m"] = 0.6
    for row in rows[40:60]:
        row["heading_error_deg"] = 6
    result = acceptance.fixed_window_metrics(rows, 0)
    assert result["joint_fraction"] == pytest.approx(561/601)
    assert not result["passed"]


def test_missing_odom_interval_invalidates_fixed_window():
    rows = [sample(i/10) for i in range(601) if not 20 <= i <= 30]
    result = acceptance.fixed_window_metrics(rows, 0)
    assert result["joint_fraction"] == 1
    assert not result["complete"] and not result["passed"]


def test_settle_requires_continuous_five_seconds_and_low_speed():
    detector = acceptance.SettleDetector(0)
    for i in range(50):
        detector.add(sample(i/10))
    assert detector.reached is None
    detector.add(sample(5, speed=0.05))
    for i in range(51, 101):
        detector.add(sample(i/10))
    assert detector.reached is None
    detector.add(sample(10.1))
    assert detector.reached == pytest.approx(10.1)


def test_settle_does_not_count_missing_observations():
    detector = acceptance.SettleDetector(0)
    detector.add(sample(0))
    detector.add(sample(5))
    assert detector.reached is None


def test_standby_label_alone_cannot_satisfy_measured_centre_return():
    detector = acceptance.DwellDetector(lambda row: acceptance.at_centre(row, 0.5))
    for i in range(20):
        detector.add({**sample(i/10, position=0.8), "phase": "STANDBY"})
    assert detector.reached is None
    for i in range(20, 31):
        detector.add({**sample(i/10, position=0.1, speed=0.2), "phase": "STANDBY"})
    assert detector.reached is None
    for i in range(31, 42):
        detector.add({**sample(i/10), "phase": "STANDBY"})
    assert detector.reached == pytest.approx(4.1)


def test_safe_wait_requires_selected_goal_low_speed_and_full_hull_gap():
    row = {**sample(0), "phase": "WAITING_CLEARANCE", "pose": [3, 4, 0],
           "temporary_goal": [3, 4, 0], "controlled_obstacle_clearance_m": 0.3}
    assert acceptance.at_safe_waiting_point(row, 0.5)
    assert not acceptance.at_safe_waiting_point(dict(row, temporary_goal=[4, 4, 0]), 0.5)
    assert not acceptance.at_safe_waiting_point(dict(row, speed_mps=0.11), 0.5)
    assert not acceptance.at_safe_waiting_point(dict(row, phase="AVOIDING"), 0.5)
    assert not acceptance.at_safe_waiting_point(dict(row, controlled_obstacle_clearance_m=0.19), 0.5)


def orbit_fixture(total_angle=2*math.pi, clockwise=True, count=1200):
    config = acceptance.scenario_config("orbit_cw" if clockwise else "orbit_ccw")
    direction = -1 if clockwise else 1
    rows = []
    for i in range(count+1):
        theta = direction*total_angle*i/count
        pose = [23+8*math.cos(theta), -10+8*math.sin(theta), theta+direction*math.pi/2]
        rows.append({"sim_time_s": i/10, "pose": pose, "phase": "ORBITING", "radial_error_m": 0.0,
                     "speed_mps": 0.5, "body_velocity": [0.5, 0, 0]})
    statuses = [{"phase": "ENTERING_ORBIT", "sim_time_s": -1.0}]
    statuses += [{"phase": "ORBITING", "sim_time_s": float(i)} for i in range(math.ceil(count/10)+1)]
    statuses.append({"phase": "RETURN_CENTER", "sim_time_s": math.ceil(count/10)+1.0})
    return rows, config, statuses


@pytest.mark.parametrize("clockwise", [True, False])
def test_independent_raw_odometry_verifies_full_signed_circle(clockwise):
    rows, config, statuses = orbit_fixture(clockwise=clockwise)
    result = acceptance.independent_orbit_metrics(rows, config, statuses, 1.0)
    assert result["passed"]
    assert result["net_laps"] == pytest.approx(1.0)
    assert result["boundary_angle_tolerance_rad"] == pytest.approx(0.15)
    assert result["maximum_boundary_interval_s"] == 1.0


def test_same_phase_same_stamp_status_repeats_do_not_change_circle_score_or_tolerance():
    rows, config, statuses = orbit_fixture()
    baseline = acceptance.independent_orbit_metrics(rows, config, statuses, 1.0)
    repeated = []
    for row in statuses:
        repeated.append(dict(row))
        if row["phase"] == "ORBITING" and row["sim_time_s"] in (54., 89., 110.):
            repeated.extend([dict(row), dict(row)])
    result = acceptance.independent_orbit_metrics(rows, config, repeated, 1.0)
    assert result["passed"]
    assert result["duplicate_status_count"] == 6
    assert result["status_observation_count"] == len(statuses)
    assert not result["invalid_status_intervals"]
    assert result["net_laps"] == baseline["net_laps"]
    assert result["boundary_angle_tolerance_rad"] == baseline["boundary_angle_tolerance_rad"] == 0.15
    assert result["boundary_intervals"] == baseline["boundary_intervals"]


def test_duplicate_status_snapshots_cannot_make_stationary_vessel_complete_a_lap():
    rows, config, statuses = orbit_fixture(total_angle=0.)
    statuses = [dict(row) for row in statuses for _ in range(3)]
    result = acceptance.independent_orbit_metrics(rows, config, statuses, 1.0)
    assert result["duplicate_status_count"] > 0
    assert result["net_laps"] == 0.
    assert not result["invalid_status_intervals"]
    assert not result["passed"]


@pytest.mark.parametrize('change', ['same_stamp_phase_change', 'backwards_stamp', 'large_gap'])
def test_status_duplicate_folding_preserves_invalid_time_or_phase_transitions(change):
    rows, config, statuses = orbit_fixture()
    statuses.insert(21, dict(statuses[20]))
    if change == 'same_stamp_phase_change':
        statuses.insert(22, {"phase": "AVOIDING", "sim_time_s": statuses[20]["sim_time_s"]})
    elif change == 'backwards_stamp':
        statuses.insert(22, {"phase": "ORBITING", "sim_time_s": statuses[20]["sim_time_s"]-.1})
    else:
        del statuses[22:27]
    result = acceptance.independent_orbit_metrics(rows, config, statuses, 1.0)
    assert result["duplicate_status_count"] == 1
    assert result["invalid_status_intervals"]
    assert not result["passed"]
    assert result["boundary_angle_tolerance_rad"] <= 0.15


@pytest.mark.parametrize("travel", [0.0, math.pi, 2*math.pi-0.2, -2*math.pi])
def test_orbit_early_return_stationary_and_wrong_way_never_pass(travel):
    rows, config, statuses = orbit_fixture(travel)
    # Even a plausible phase progression ending in RETURN_CENTER is insufficient.
    result = acceptance.independent_orbit_metrics(rows, config, statuses, 1.0)
    assert not result["passed"]


def test_orbit_missing_odometry_does_not_inflate_boundary_allowance():
    rows, config, statuses = orbit_fixture(2*math.pi+0.2)
    del rows[100:170]
    result = acceptance.independent_orbit_metrics(rows, config, statuses, 1.0)
    assert result["invalid_odom_segments"]
    assert not result["passed"]
    assert result["boundary_angle_tolerance_rad"] <= 0.15


def test_orbit_position_teleport_and_status_dropout_fail_closed():
    rows, config, statuses = orbit_fixture(2*math.pi+0.2)
    rows[100]["pose"][0] += 2
    result = acceptance.independent_orbit_metrics(rows, config, statuses, 1.0)
    assert not result["passed"] and result["invalid_odom_segments"]
    rows, config, statuses = orbit_fixture()
    del statuses[30:40]
    result = acceptance.independent_orbit_metrics(rows, config, statuses, 1.0)
    assert not result["passed"] and result["invalid_status_intervals"]


def test_orbit_avoidance_and_radial_excursions_are_excluded_from_net_laps():
    rows, config, statuses = orbit_fixture()
    for row in rows[100:250]:
        row["phase"] = "AVOIDING"
    for row in rows[300:450]:
        row["radial_error_m"] = 0.6
    result = acceptance.independent_orbit_metrics(rows, config, statuses, 1.0)
    assert result["net_laps"] < 0.8
    assert not result["passed"]


def test_orbit_obstacle_cannot_be_deleted_after_only_one_metre_motion():
    rows, config, _ = orbit_fixture(total_angle=0.125, count=20)
    spawn = rows[0]["pose"]
    obstacle_theta = -math.radians(22)
    obstacle = {"x": 23+8*math.cos(obstacle_theta), "y": -10+8*math.sin(obstacle_theta), "radius": 0.5}
    attach_obstacle_clearances(rows, obstacle, spawn)
    assert not acceptance.orbit_rejoined_after_obstacle(rows, config, obstacle, spawn)["passed"]
    rows, config, obstacle, spawn = interior_detour()
    evidence = acceptance.orbit_rejoined_after_obstacle(rows, config, obstacle, spawn)
    assert evidence["passed"]
    rows[-1]["phase"] = "AVOIDING"
    assert not acceptance.orbit_rejoined_after_obstacle(rows, config, obstacle, spawn)["passed"]


def attach_obstacle_clearances(rows, obstacle, spawn):
    previous = spawn
    for row in rows:
        row['controlled_obstacle_clearance_m'] = acceptance.swept_clearance(previous, row['pose'], obstacle)
        previous = row['pose']


def interior_detour(clockwise=True, end_degrees=175.):
    config = acceptance.scenario_config("orbit_cw" if clockwise else "orbit_ccw")
    direction = -1 if clockwise else 1
    cx, cy = config["target"][:2]
    obstacle_angle = direction*math.radians(22)
    obstacle = {"x": cx+8*math.cos(obstacle_angle), "y": cy+8*math.sin(obstacle_angle), "radius": .5}
    waypoints = []
    for degrees, radius in ((0., 8.), (70., .02), (120., .02), (end_degrees, 8.)):
        theta = -direction*math.radians(degrees)
        waypoints.append((cx+radius*math.cos(theta), cy+radius*math.sin(theta)))
    rows = []
    for start, finish in zip(waypoints, waypoints[1:]):
        dx, dy = finish[0]-start[0], finish[1]-start[1]
        count = max(1, math.ceil(math.hypot(dx, dy)/.05))
        for i in range(count):
            x, y = start[0]+dx*i/count, start[1]+dy*i/count
            rows.append({'sim_time_s': len(rows)/10, 'pose': [x, y, math.atan2(dy, dx)],
                         'phase': 'AVOIDING', 'radial_error_m': abs(math.hypot(x-cx, y-cy)-8), 'speed_mps': .5})
    theta = -direction*math.radians(end_degrees)
    rows.append({'sim_time_s': len(rows)/10, 'pose': [*waypoints[-1], theta+direction*math.pi/2],
                 'phase': 'ORBITING', 'radial_error_m': 0., 'speed_mps': .5})
    attach_obstacle_clearances(rows, obstacle, rows[0]['pose'])
    return rows, config, obstacle, rows[0]["pose"]


@pytest.mark.parametrize('clockwise', [True, False])
def test_rejoin_accepts_downstream_circle_after_opposite_polar_motion_near_centre(clockwise):
    rows, config, obstacle, spawn = interior_detour(clockwise)
    evidence = acceptance.orbit_rejoined_after_obstacle(rows, config, obstacle, spawn)
    assert evidence['directed_progress_rad'] < 0
    assert evidence['passed']
    assert evidence['minimum_footprint_downstream_projection_m'] > 0


@pytest.mark.parametrize('backwards_degrees', [0., 1., 5.])
def test_rejoin_rejects_spawn_and_small_upstream_backtrack(backwards_degrees):
    _, _, obstacle, _ = interior_detour()
    rows, config, _ = orbit_fixture(total_angle=-math.radians(backwards_degrees), count=20)
    spawn = rows[0]['pose']
    attach_obstacle_clearances(rows, obstacle, spawn)
    evidence = acceptance.orbit_rejoined_after_obstacle(rows, config, obstacle, spawn)
    assert not evidence['passed']
    assert evidence['downstream_projection_m'] < 0


@pytest.mark.parametrize('invalid', ['phase', 'radial', 'heading', 'speed'])
def test_downstream_rejoin_keeps_original_on_circle_motion_requirements(invalid):
    rows, config, obstacle, spawn = interior_detour()
    current = rows[-1]
    if invalid == 'phase':
        current['phase'] = 'AVOIDING'
    elif invalid == 'radial':
        current['radial_error_m'] = .51
    elif invalid == 'heading':
        current['pose'][2] += math.radians(16)
    else:
        current['speed_mps'] = .099
    assert not acceptance.orbit_rejoined_after_obstacle(rows, config, obstacle, spawn)['passed']


def test_downstream_centre_does_not_suffice_when_hull_straddles_passage_plane():
    rows, config, obstacle, spawn = interior_detour(end_degrees=159.)
    evidence = acceptance.orbit_rejoined_after_obstacle(rows, config, obstacle, spawn)
    assert evidence['downstream_projection_m'] > 0
    assert evidence['minimum_footprint_downstream_projection_m'] < 0
    assert not evidence['passed']


def test_downstream_rejoin_still_needs_one_second_of_fresh_measured_dwell():
    rows, config, obstacle, spawn = interior_detour()
    history = rows[:-1]
    detector = acceptance.DwellDetector(lambda _: acceptance.orbit_rejoined_after_obstacle(
        history, config, obstacle, spawn)['passed'])
    start = rows[-1]['sim_time_s']
    for i in range(11):
        current = dict(rows[-1], sim_time_s=start+i/10)
        history.append(current)
        detector.add(current)
        if i < 10:
            assert detector.reached is None
    assert detector.reached == pytest.approx(start+1.)
    current = dict(rows[-1], sim_time_s=start+1.6)
    history.append(current)
    detector.add(current)
    assert detector.reached is None


def test_observed_passage_is_not_revoked_when_rejoin_continues_past_opposite_half_circle():
    rows, config, obstacle, spawn = interior_detour()
    first_time = rows[-1]['sim_time_s']
    for i in range(1, 201):
        theta = math.radians(175)-.5/8*i/10
        rows.append({'sim_time_s': first_time+i/10,
                     'pose': [23+8*math.cos(theta), -10+8*math.sin(theta), theta-math.pi/2],
                     'phase': 'ORBITING', 'radial_error_m': 0., 'speed_mps': .5})
    attach_obstacle_clearances(rows, obstacle, spawn)
    evidence = acceptance.orbit_rejoined_after_obstacle(rows, config, obstacle, spawn)
    assert evidence['downstream_projection_m'] < 0
    assert evidence['first_downstream_passage']['sim_time_s'] < first_time
    assert evidence['passed']


@pytest.mark.parametrize('invalid', ['gap', 'missing_gap', 'missing_odom', 'duplicate_odom'])
def test_rejoin_does_not_discard_unsafe_or_unobserved_earlier_detour(invalid):
    rows, config, obstacle, spawn = interior_detour()
    if invalid == 'gap':
        rows[10]['controlled_obstacle_clearance_m'] = .19
    elif invalid == 'missing_gap':
        del rows[10]['controlled_obstacle_clearance_m']
    elif invalid == 'missing_odom':
        del rows[10:20]
    else:
        rows[10]['sim_time_s'] = rows[9]['sim_time_s']
    assert not acceptance.orbit_rejoined_after_obstacle(rows, config, obstacle, spawn)['passed']


def test_lateral_impulse_requires_fresh_observed_velocity_not_displacement():
    rows = [{**sample(10.1), "body_velocity": [0.0, 0.35, 0.0], "pose": [25, -8, 1.57]}]
    assert acceptance.observed_lateral_impulse(rows, 10.0)["observed"]
    assert not acceptance.observed_lateral_impulse(rows, 11.0)["observed"]
    rows[0]["body_velocity"] = [0.0, 0.0, 0.0]
    rows[0]["pose"] = [24.6, -8, 1.57]
    assert not acceptance.observed_lateral_impulse(rows, 10.0)["observed"]


def test_delayed_impulse_window_covers_service_interval_and_post_response_samples():
    rows = [{**sample(t), "body_velocity": [0.0, speed, 0.0]} for t, speed in
            [(9.9, 0.6), (10.7, 0.3), (11.1, 0.4), (11.4, 0.6)]]
    evidence = acceptance.observed_lateral_impulse(rows, 10.0, response_time=10.8)
    assert evidence["observed"]
    assert evidence["observation_interval_sim_s"] == pytest.approx([10.0, 11.3])
    assert evidence["observed_sample"]["sim_time_s"] == 11.1


def delayed_impulse_probe(monkeypatch, delay, observed_speed):
    class Log:
        def __init__(self):
            self.samples = [{**sample(9.9), "body_velocity": [0.0, 0.4, 0.0]}]
            self.events = []
        def write(self, event, **values):
            self.events.append({"event": event, **values})

    def request():
        return SimpleNamespace(state=SimpleNamespace(
            pose=SimpleNamespace(position=SimpleNamespace(), orientation=SimpleNamespace()),
            twist=SimpleNamespace(linear=SimpleNamespace())))

    log = Log()
    probe = SimpleNamespace(pose=(25.0, -8.0, math.pi/2), now=10.0, log=log,
                            types={"SetEntityState": SimpleNamespace(Request=request)})
    probe.sim_time = lambda: probe.now

    def delayed_service(*args, **kwargs):
        # A stale observation received during the call must not qualify.
        log.samples.append({**sample(9.95), "body_velocity": [0.0, 0.4, 0.0]})
        probe.now = 10.0+delay
        log.samples.append({**sample(probe.now-.05), "body_velocity": [0.0, observed_speed, 0.0]})
        return SimpleNamespace(success=True)

    monkeypatch.setattr(acceptance, 'call_service', delayed_service)
    monkeypatch.setattr(acceptance, 'advance', lambda probe, process, seconds, predicate: predicate())
    return probe, log


def test_lateral_impulse_accepts_observation_during_delayed_service_and_logs_timing(monkeypatch):
    probe, log = delayed_impulse_probe(monkeypatch, .8, .35)
    evidence = acceptance.inject_lateral_impulse(probe, None, log)
    assert evidence['observed']
    assert evidence['service_request_sim_s'] == 10.0
    assert evidence['service_response_sim_s'] == pytest.approx(10.8)
    assert evidence['service_delay_sim_s'] == pytest.approx(.8)
    response = next(row for row in log.events if row['event'] == 'lateral_velocity_impulse_0_4mps')
    assert response['service_request_sim_s'] == 10.0
    assert response['service_response_sim_s'] == pytest.approx(10.8)


def test_delayed_service_without_pulse_does_not_reuse_earlier_high_velocity(monkeypatch):
    probe, log = delayed_impulse_probe(monkeypatch, .8, 0.0)
    with pytest.raises(acceptance.TrialBlocked, match='not observed'):
        acceptance.inject_lateral_impulse(probe, None, log)
    evidence = next(row for row in log.events if row['event'] == 'lateral_velocity_impulse_observation')
    assert not evidence['observed']
    assert evidence['peak_observed_body_lateral_mps'] == 0.0
    assert evidence['observation_interval_sim_s'] == pytest.approx([10.0, 11.3])


def test_lateral_impulse_excessive_service_delay_is_independently_blocked(monkeypatch):
    probe, log = delayed_impulse_probe(monkeypatch, 2.1, .35)
    with pytest.raises(acceptance.TrialBlocked, match='service delay'):
        acceptance.inject_lateral_impulse(probe, None, log)
    evidence = next(row for row in log.events if row['event'] == 'lateral_velocity_impulse_observation')
    assert not evidence['service_delay_within_limit']
    assert evidence['service_delay_sim_s'] == pytest.approx(2.1)


def test_heading_error_wrap_and_scenario_targets():
    values = acceptance.sample_errors((0, 0, math.radians(179)), (0, 0, 0), (0, 0, math.radians(-179)))
    assert values["heading_error_deg"] == pytest.approx(2)
    assert acceptance.scenario_config("station")["target"] == (25, -8, math.pi/2)
    assert acceptance.scenario_config("orbit_cw")["clockwise"] is True
    assert acceptance.scenario_config("orbit_ccw")["clockwise"] is False


def test_preflight_rejects_existing_lethal_unknown_and_unobserved_envelope():
    config = acceptance.scenario_config("station")
    data = [0]*10000
    costmap = (100, 100, 1.0, -50, -50, data, "raw")
    assert not acceptance.preflight_failures(costmap, config)
    data[42*100+75] = 254  # (25.5, -7.5), within station test region.
    assert acceptance.preflight_failures(costmap, config)
    data[42*100+75] = 255
    assert acceptance.preflight_failures(costmap, config)
    assert acceptance.preflight_failures((10, 10, 1.0, 20, -12, [0]*100, "raw"), config)


def test_invalid_command_is_recorded_then_stops_acquisition(tmp_path):
    recorder = acceptance.MissionLog(tmp_path/"events.jsonl", acceptance.scenario_config("station"))
    try:
        with pytest.raises(acceptance.TrialFailed, match="out_of_range"):
            recorder.write("thruster_command", normalized_command=1.1, axis="left")
        assert recorder.invalid_command
    finally:
        recorder.close()
    assert json.loads((tmp_path/"events.jsonl").read_text())["normalized_command"] == 1.1


def test_raw_logs_keep_each_orbit_sample_and_phase(tmp_path):
    recorder = acceptance.MissionLog(tmp_path/"events.jsonl", acceptance.scenario_config("orbit_cw"))
    recorder.active = True
    try:
        recorder.write("navigation_status", payload={"mission": {"phase": "ORBITING"}})
        recorder.write("odom", sim_time_s=1, pose=(31, -10, 0), body_velocity=(0.5, 0, 0))
        assert recorder.samples[0]["radial_error_m"] == 0
        assert recorder.samples[0]["phase"] == "ORBITING"
    finally:
        recorder.close()


def test_controlled_obstacle_requires_lethal_observation_not_unknown():
    obstacle = {"x": 3, "y": 3, "radius": 0.5}
    data = [0]*100
    costmap = (10, 10, 1.0, 0.0, 0.0, data, "raw")
    assert not acceptance.obstacle_observed(costmap, obstacle)
    data[33] = 255
    assert not acceptance.obstacle_observed(costmap, obstacle)
    data[33] = 254
    assert acceptance.obstacle_observed(costmap, obstacle)


def test_dry_run_never_reports_an_experiment_pass(tmp_path):
    output = tmp_path/"dry"
    assert acceptance.main(["--workspace", str(tmp_path), "--output", str(output),
                            "--scenarios", "station", "--repeats", "1", "--dry-run"]) == 0
    summary = json.loads((output/"summary.json").read_text())
    assert summary["result"] == "not_run"
    assert summary["all_passed"] is False and summary["results"] == []


def test_plotter_does_not_create_figures_for_blocked_trials_without_measurements(tmp_path):
    spec = importlib.util.spec_from_file_location("plot_missions", Path(__file__).with_name("plot_missions.py"))
    plotter = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(plotter)
    (tmp_path/"metrics.json").write_text(json.dumps({"trial": "blocked_unit_fixture", "result": "blocked"}))
    (tmp_path/"events.jsonl").write_text("")
    result = plotter.plot_trial(tmp_path)
    assert result["plots"] == []
    assert not list(tmp_path.glob("*.png"))


def test_plotter_exports_only_logged_samples_with_separate_command_timebase(tmp_path):
    spec = importlib.util.spec_from_file_location("plot_missions", Path(__file__).with_name("plot_missions.py"))
    plotter = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(plotter)
    config = acceptance.scenario_config("station")
    metrics = {"trial": "synthetic_unit_fixture_not_an_experiment", "result": "not_run", "config": config}
    (tmp_path/"metrics.json").write_text(json.dumps(metrics))
    rows = [{"event": "mission_sample", "sim_time_s": float(i), "pose": [25+i/100, -8, 1.57],
             "phase": "STATION_KEEP", "position_error_m": i/100, "heading_error_deg": 0.1} for i in range(3)]
    rows += [{"event": "thruster_command", "axis": "left", "wall_monotonic_s": 40+i,
              "normalized_command": 0.01} for i in range(3)]
    (tmp_path/"events.jsonl").write_text("\n".join(json.dumps(row) for row in rows))
    result = plotter.plot_trial(tmp_path)
    assert len(result["plots"]) == 2
    assert (tmp_path/"mission_plots.png").read_bytes().startswith(b"\x89PNG")
    assert (tmp_path/"mission_plots.pdf").read_bytes().startswith(b"%PDF")
