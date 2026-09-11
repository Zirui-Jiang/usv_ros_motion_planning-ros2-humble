#!/usr/bin/env python3
"""Read-only straight-trial diagnostics. These values never replace main scores.

Requires Python 3.10+ and NumPy for the original NPZ map metadata. No ROS,
production algorithm imports, data rewriting, plotting, or simulation runs.
"""
import argparse
import bisect
from collections import defaultdict
import hashlib
import json
import math
from pathlib import Path

import numpy as np


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def rms(values):
    return math.sqrt(sum(v*v for v in values)/len(values)) if values else None


def distance(point, path):
    """Same finite-segment Euclidean distance definition as the primary score."""
    if len(path) == 1:
        return math.dist(point[:2], path[0][:2])
    answer = math.inf
    for a, b in zip(path, path[1:]):
        dx, dy = b[0]-a[0], b[1]-a[1]
        squared = dx*dx+dy*dy
        f = 0. if squared == 0 else max(0., min(1., ((point[0]-a[0])*dx+(point[1]-a[1])*dy)/squared))
        answer = min(answer, math.hypot(point[0]-a[0]-f*dx, point[1]-a[1]-f*dy))
    return answer


def describe(rows, path):
    if not rows:
        return {"count": 0}
    speeds = [math.hypot(*r['body_velocity'][:2]) for r in rows]
    return {"count": len(rows), "rmse_m": rms([distance(r['pose'], path) for r in rows]),
            "mean_y_m": sum(r['pose'][1] for r in rows)/len(rows),
            "yaw_min_max_rad": [min(r['pose'][2] for r in rows), max(r['pose'][2] for r in rows)],
            "mean_body_u_mps": sum(r['body_velocity'][0] for r in rows)/len(rows),
            "mean_speed_mps": sum(speeds)/len(speeds), "max_speed_mps": max(speeds)}


def analyse(directory):
    summary_path, events_path = directory/'summary.json', directory/'events.jsonl'
    summary = json.loads(summary_path.read_text())
    events = [dict(json.loads(line), event_line=i) for i, line in enumerate(events_path.open(), 1)]
    goal = next(e for e in events if e['event'] == 'goal_published')
    end = next(e for e in events if e['event'] == 'acquisition_finished')
    begun, finished = goal['wall_monotonic_s'], end['wall_monotonic_s']
    active = [e for e in events if begun <= e['wall_monotonic_s'] <= finished]
    first_path = next(e for e in active if e['event'] == 'reference_path' and e.get('points'))
    first_trajectory = next(e for e in active if e['event'] == 'local_trajectory' and e.get('poses'))
    path = first_path['points']
    samples = [e for e in active if e['event'] == 'sample']
    valid = [e for e in samples if e.get('cross_track_initial_m') is not None]
    commands = [e for e in active if e['event'] == 'thruster_command']
    first_nonzero = next(e for e in commands if abs(e['normalized_command']) > 1e-6)

    # Associate by callback order, not the harness sample's own /clock value.
    last_odom, groups, mismatches = None, defaultdict(list), 0
    for e in events:
        if e['event'] == 'odom':
            last_odom = e
        elif (e['event'] == 'sample' and e.get('cross_track_initial_m') is not None
              and begun <= e['wall_monotonic_s'] <= finished):
            if last_odom is None:
                raise ValueError('sample has no preceding recorded odometry')
            mismatches += int(e['pose'] != last_odom['pose'])
            groups[last_odom['sim_time_s']].append(e)
    if mismatches:
        raise ValueError('sample/latest odom association is not exact')
    once = [rows[-1]['cross_track_initial_m'] for rows in groups.values()]
    before_command = [s['cross_track_initial_m'] for s in valid
                      if s['wall_monotonic_s'] < first_nonzero['wall_monotonic_s']]
    after_command = [s['cross_track_initial_m'] for s in valid
                     if s['wall_monotonic_s'] >= first_nonzero['wall_monotonic_s']]

    all_odom = [e for e in events if e['event'] == 'odom']
    preceding = [e for e in all_odom if e['wall_monotonic_s'] <= begun]
    source = preceding[-1:] + [e for e in all_odom if begun < e['wall_monotonic_s'] <= finished]
    # Include the latest observed state at goal publication; never extrapolate
    # its source timestamp to the wall-clock boundary or acquisition end.
    source = list({e['sim_time_s']: e for e in source}.values())
    bounds = [(23., 26.), (26., 32.), (32., 36.)]
    bins = [{"x_interval_m": [lo, hi], **describe([e for e in source if lo <= e['pose'][0] < hi], path),
             "source_duration_s": 0., "integrated_squared_error_m2_s": 0.} for lo, hi in bounds]
    energy, duration = 0., 0.
    for a, b in zip(source, source[1:]):
        dt = b['sim_time_s']-a['sim_time_s']
        if dt <= 0:
            raise ValueError('source odometry does not advance monotonically')
        value = dt*(distance(a['pose'], path)**2+distance(b['pose'], path)**2)/2
        energy += value
        duration += dt
        x = (a['pose'][0]+b['pose'][0])/2
        for region, (lo, hi) in zip(bins, bounds):
            if lo <= x < hi:
                region['source_duration_s'] += dt
                region['integrated_squared_error_m2_s'] += value
    for region in bins:
        region['time_weighted_rmse_m'] = (math.sqrt(region['integrated_squared_error_m2_s']/region['source_duration_s'])
                                           if region['source_duration_s'] else None)
        region['contribution_to_full_time_weighted_mse_m2'] = region['integrated_squared_error_m2_s']/duration

    # Pair already-published left/right observations within 20 ms; distinguish
    # unpaired messages explicitly instead of inventing a command cycle.
    rights = [e for e in commands if e['axis'] == 'right']
    right_times = [e['wall_monotonic_s'] for e in rights]
    odom_times = [e['wall_monotonic_s'] for e in all_odom]
    paired, unpaired = [], 0
    for e in commands:
        if e['axis'] != 'left':
            continue
        t = e['wall_monotonic_s']
        j = bisect.bisect_left(right_times, t-.005)
        candidates = rights[max(0, j-1):j+3]
        other = min(candidates, key=lambda r: abs(r['wall_monotonic_s']-t)) if candidates else None
        if other is None or abs(other['wall_monotonic_s']-t) > .02:
            unpaired += 1
            continue
        od = all_odom[bisect.bisect_right(odom_times, t)-1]
        paired.append({"wall_after_goal_s": t-begun, "left": e['normalized_command'],
                       "right": other['normalized_command'], "odom_source_s": od['sim_time_s'],
                       "latest_odom_age_wall_ms": 1000*(t-od['wall_monotonic_s'])})
    arrival_wall = next((s['wall_monotonic_s'] for s in samples if s['goal_distance_m'] <= 1.5), finished)
    first_pair = next(e for e in paired if abs(e['left']) > 1e-6 or abs(e['right']) > 1e-6)
    moving = [e for e in paired if first_pair['wall_after_goal_s'] <= e['wall_after_goal_s'] < arrival_wall-begun]
    zero = [e for e in moving if abs(e['left']) < 1e-9 and abs(e['right']) < 1e-9]
    gaps = [{"from_source_s": a['sim_time_s'], "to_source_s": b['sim_time_s'],
             "wall_after_goal_s": a['wall_monotonic_s']-begun,
             "wall_gap_s": b['wall_monotonic_s']-a['wall_monotonic_s']}
            for a, b in zip(all_odom, all_odom[1:])
            if b['wall_monotonic_s'] >= begun and a['wall_monotonic_s'] <= finished
            and b['wall_monotonic_s']-a['wall_monotonic_s'] > .3]

    snap = next(e for e in events if e['event'] == 'goal_input_snapshot')
    state_path, map_path = directory/snap['state_path'], directory/snap['costmap_path']
    state = json.loads(state_path.read_text())
    with np.load(map_path) as grid:
        ox, oy, resolution = map(float, (grid['origin_x'], grid['origin_y'], grid['resolution']))
        ix, iy = math.floor((state['pose'][0]-ox)/resolution), math.floor((state['pose'][1]-oy)/resolution)
        metadata = {"origin_xy": [ox, oy], "resolution": resolution, "shape": list(grid['data'].shape),
                    "stamp": float(grid['stamp']), "state": state, "cell_xy": [ix, iy],
                    "cell_center_xy": [ox+(ix+.5)*resolution, oy+(iy+.5)*resolution],
                    "source_cost": int(grid['data'][iy, ix])}
    return {"trial": directory.name, "source_directory": str(directory),
            "inputs_sha256": {str(p): digest(p) for p in (summary_path, events_path, map_path, state_path)},
            "official": {k: summary.get(k) for k in ('result', 'passed', 'duration_sim_s', 'duration_wall_s', 'cross_track_initial_m')},
            "raw_goal_snapshot": metadata, "goal": goal, "acquisition_end": end,
            "controller_verified": next(e for e in events if e['event'] == 'controller_verified'),
            "first_path": first_path, "first_path_segment_lengths_m": [math.dist(a, b) for a, b in zip(path, path[1:])],
            "first_trajectory": first_trajectory,
            "delays_wall_s": {"first_path": first_path['wall_monotonic_s']-begun,
                              "first_trajectory": first_trajectory['wall_monotonic_s']-begun,
                              "first_nonzero_command": first_nonzero['wall_monotonic_s']-begun},
            "source_repeat_diagnostic": {"valid_wall_samples": len(valid), "unique_source_stamps": len(groups),
                "extra_repeated_source_samples": len(valid)-len(groups), "pose_mismatches": mismatches,
                "new_source_only_rmse_m": rms(once),
                "multiplicities": [{"stamp": stamp, "count": len(rows), "error_m": rows[0]['cross_track_initial_m']}
                                   for stamp, rows in groups.items() if len(rows) > 1]},
            "pre_command_diagnostic": {"count": len(before_command), "rmse_m": rms(before_command),
                "fraction_of_official_squared_error": sum(v*v for v in before_command)/sum(s['cross_track_initial_m']**2 for s in valid),
                "post_command_rmse_m": rms(after_command)},
            "source_time_diagnostic": {"count": len(source), "start": source[0]['sim_time_s'],
                "end": source[-1]['sim_time_s'], "span_s": duration, "rmse_m": math.sqrt(energy/duration),
                "x_bins": bins},
            "motion_commands": {"first_pair": first_pair, "moving_pairs": len(moving),
                                "unpaired_left_messages": unpaired, "zero_pairs": zero},
            "odom_wall_gaps_over_300ms": gaps}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--results-root', type=Path, default=Path('/home/usv/usv_migration/results'))
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        parser.error('output must not already exist')
    locations = [('python-release-grid-001', 'python_pid_straight_01'),
                 ('baseline-grid-interior-001', 'cpp_pid_straight_01'),
                 ('python-release-grid-001', 'python_pid_straight_03')]
    trials = [analyse(args.results_root/run/trial) for run, trial in locations]
    py, cpp = trials[:2]
    report = {"classification": "diagnostic only; primary acceptance scores/verdicts unchanged",
              "definitions": ["No initial sample, stopped interval, or repeated trial replaces a formal result.",
                "New-source-only uses the last wall sample per preceding recorded odometry source timestamp; poses must match exactly.",
                "Source-time RMSE trapezoid-integrates squared finite-polyline distance at actual odometry source times.",
                "Source-time window includes the last observed state at goal publication and all observations through acquisition end; no endpoint extrapolation.",
                "x bins use distinct source odometry; interval energy goes to the bin of the midpoint x coordinate.",
                "Command pairing uses received left/right events within 20ms; motion window is first nonzero until first observed distance<=1.5m.",
                "Python repeat03 is context only and is never paired with baseline repeat01."],
              "script_sha256": digest(Path(__file__)), "trials": trials,
              "repeat01_ratios": {"official": py['official']['cross_track_initial_m']['rms']/cpp['official']['cross_track_initial_m']['rms'],
                "new_source_only_diagnostic": py['source_repeat_diagnostic']['new_source_only_rmse_m']/cpp['source_repeat_diagnostic']['new_source_only_rmse_m'],
                "source_time_diagnostic": py['source_time_diagnostic']['rmse_m']/cpp['source_time_diagnostic']['rmse_m']}}
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2, allow_nan=False)+'\n')
    print(json.dumps(report['repeat01_ratios']))


if __name__ == '__main__':
    main()
