#!/usr/bin/env python3
"""Read-only turn01 diagnostic; retains all source observations and raw scores."""
import argparse
import bisect
import importlib.util
import json
import math
from pathlib import Path


def module(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    value = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(value)
    return value


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--repo-root', type=Path, default=Path(__file__).resolve().parents[4])
    parser.add_argument('--results-root', type=Path, default=Path('/home/usv/usv_migration/results'))
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    repo, root, output = args.repo_root.resolve(), args.results_root.resolve(), args.output.resolve()
    if output.exists() or root == output or root in output.parents:
        parser.error('output must be new and outside the raw-results tree')
    previous = repo / 'docs/validation/source-time-candidate-diagnostics/reproduce.py'
    d = module('previous_diagnostic', previous)
    geometry = module('geometry', repo / 'scripts/validation/metrics.py')
    runs = {
        'formal': 'python-grid-endpoint-final-matrix-001/python_pid_turn_01',
        'candidate': 'python-grid-endpoint-turn-candidate-001/python_pid_turn_01',
        'cpp': 'baseline-grid-interior-001/cpp_pid_turn_01',
    }
    result = {
        'purpose': 'Post-hoc localization of a retained formal failure, not an acceptance rerun or a new score.',
        'decomposition_definition': 'Reuse complete source-time trapezoids; bin by midpoint progress -y-10.125m: <3, 3..9, >=9. No interval or startup/tail observation removed.',
        'reference_definition': 'Latest received published trajectory, not an atomic record of the control callback. Reconstructed preview cannot prove which reference generated a received command.',
        'zero_definition': 'Consecutive wall samples with all three known normalized commands within 1e-6; split at receipt gap >0.2s. Counts are observations, not control computations.',
        'pause_definition': 'Record odom receipt gaps >0.3s and received non-NAVIGATING statuses; absent clock/per-scan TF records cannot prove pause cause.',
        'source_files': {str(p): d.digest(p) for p in [Path(__file__), previous, repo / 'scripts/validation/metrics.py']},
        'trials': {},
    }
    loaded = {}
    for label, relative in runs.items():
        directory = root / relative
        trial = d.load_trial(directory)
        loaded[label] = trial
        dec = d.decomposition(trial, geometry, preview=3.0 if label == 'cpp' else 2.6)
        active, odom = trial['active'], trial['odom']
        start = active[0]
        first_refs = {}
        for event_name, field in [('reference_path', 'points'), ('local_trajectory', 'poses')]:
            event = next(e for e in active if e['event'] == event_name and e.get(field))
            first_refs[event_name] = {'event_line': event['event_line'], 'source_stamp_s': event['sim_time_s'],
                'wall_delay_from_goal_s': event['wall_monotonic_s'] - start['wall_monotonic_s'],
                'first7': event[field][:7], 'last3': event[field][-3:]}
        command = next(e for e in active if e['event'] == 'thruster_command' and abs(e['normalized_command']) > 1e-6)
        zero_groups = []
        for event in active:
            if event['event'] != 'sample' or len(event.get('command', {})) != 3:
                continue
            if not all(abs(v) <= 1e-6 for v in event['command'].values()):
                continue
            if not zero_groups or event['wall_monotonic_s'] - zero_groups[-1][-1]['wall_monotonic_s'] > .2:
                zero_groups.append([])
            zero_groups[-1].append(event)
        gaps = []
        for a, b in zip(odom, odom[1:]):
            wall_gap = b['wall_monotonic_s'] - a['wall_monotonic_s']
            if wall_gap > .3:
                gaps.append({'source_before_s': a['sim_time_s'], 'source_after_s': b['sim_time_s'],
                    'wall_gap_s': wall_gap, 'before_pose': a['pose'], 'after_pose': b['pose'],
                    'before_body_velocity': a['body_velocity'], 'after_body_velocity': b['body_velocity'],
                    'before_event_line': a['event_line'], 'after_event_line': b['event_line']})
        all_events = [json.loads(line) for line in (directory / 'events.jsonl').open() if line.strip()]
        verified = next(e for e in all_events if e['event'] == 'controller_verified')
        dec.update({'goal_event': start, 'initial_odom': odom[0], 'first_references': first_refs,
            'first_nonzero_command': {'event_line': command['event_line'], 'axis': command['axis'],
                'wall_delay_from_goal_s': command['wall_monotonic_s'] - start['wall_monotonic_s']},
            'parameter_verification_wall_before_goal_s': start['wall_monotonic_s'] - verified['wall_monotonic_s'],
            'verified_parameters': verified['parameters'], 'odom_receipt_gaps': gaps,
            'non_navigating_statuses': [e for e in active if e['event'] == 'navigation_status' and e['payload']['state'] != 'NAVIGATING'],
            'zero_observation_groups': [{'count': len(group), 'first': group[0], 'last': group[-1]} for group in zero_groups]})
        result['trials'][label] = dec
    result['first_paths_identical'] = loaded['formal']['path'] == loaded['candidate']['path'] == loaded['cpp']['path']
    result['python_verified_parameters_identical'] = result['trials']['formal']['verified_parameters'] == result['trials']['candidate']['verified_parameters']
    result['differences'] = {}
    for other in ('candidate', 'cpp'):
        formal, contrast = result['trials']['formal'], result['trials'][other]
        delta = [a['contribution_to_full_mse_m2'] - b['contribution_to_full_mse_m2'] for a, b in zip(formal['bins'], contrast['bins'])]
        result['differences']['formal_minus_' + other] = {
            'full_mse_delta_by_bin': delta,
            'fraction_of_net_mse_delta_by_bin': [v / sum(delta) for v in delta],
            'source_rmse_ratio': formal['source_rmse_m'] / contrast['source_rmse_m'],
            'initial_pose_delta': [a - b for a, b in zip(formal['initial_odom']['pose'], contrast['initial_odom']['pose'])],
        }
    report_path = repo / 'docs/validation/python-grid-endpoint-final-comparison/report.json'
    report = json.loads(report_path.read_text())
    result['formal_turn_pairs'] = [p for p in report['comparisons'] if p['controller'] == 'pid' and p['scenario'] == 'turn']
    result['formal_report'] = {'path': str(report_path), 'sha256': d.digest(report_path)}
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False) + '\n')
    print(json.dumps({'output': str(output), 'sha256': d.digest(output), 'differences': result['differences']}))


if __name__ == '__main__':
    main()
