#!/usr/bin/env python3
"""Make readable tables from completed measured trials, retaining every row."""
import argparse
from collections import Counter
import csv
import hashlib
import json
from pathlib import Path


def compact(task):
    measured = task.get('measurement') or {}
    row = {name: task.get(name) for name in ('trial_id', 'scenario', 'matrix_repeat', 'result')}
    row['reason'] = task.get('reason') or measured.get('reason')
    row['duration_sim_s'] = measured.get('duration_sim_s')
    for label in ('initial', 'after_disturbance'):
        window = measured.get('windows', {}).get(label, {})
        for name in ('joint_fraction', 'sample_count', 'settle_elapsed_sim_s'):
            row[label + '_' + name] = window.get(name)
        row[label + '_max_position_m'] = window.get('position_error_m', {}).get('max')
        row[label + '_max_heading_deg'] = window.get('heading_error_deg', {}).get('max')
    row['observed_lateral_impulse_mps'] = measured.get('lateral_velocity_impulse', {}).get(
        'peak_observed_body_lateral_mps')
    radial = measured.get('normal_orbit_radial_error_m', {})
    row['orbit_radial_p95_m'] = radial.get('p95')
    row['orbit_radial_max_m'] = radial.get('max')
    row['independent_net_laps'] = measured.get('independent_orbit_progress', {}).get('net_laps')
    row['minimum_obstacle_hull_gap_m'] = measured.get('minimum_controlled_obstacle_footprint_gap_m')
    row['return_position_error_m'] = measured.get('returned_centre_dwell', {}).get(
        'last_observed_sample', {}).get('position_error_m')
    row['controlled_avoidance_passed'] = measured.get('controlled_avoidance', {}).get('passed')
    row['backing_escape_observed'] = any(
        item.get('phase') == 'BACKING_ESCAPE' for item in measured.get('phase_transitions', []))
    return row


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('summary', type=Path)
    parser.add_argument('output', type=Path)
    args = parser.parse_args()
    source = args.summary.read_bytes()
    summary = json.loads(source)
    if summary.get('counts', {}).get('running') != 0 or summary.get('running_trials'):
        raise ValueError('A completed matrix summary with running=0 is required')
    rows = [compact(task) for task in summary['results']]
    groups = {}
    for scenario in dict.fromkeys(row['scenario'] for row in rows):
        selected = [row for row in rows if row['scenario'] == scenario]
        ranges = {}
        for key in rows[0]:
            values = [row[key] for row in selected if type(row[key]) in (int, float)]
            if values and key != 'matrix_repeat':
                ranges[key] = {'count': len(values), 'min': min(values), 'max': max(values)}
        groups[scenario] = {'counts': dict(Counter(row['result'] for row in selected)),
                            'measured_ranges_all_recorded_results': ranges}
    args.output.mkdir(parents=True, exist_ok=True)
    with (args.output / 'metrics-summary.csv').open('w', encoding='utf-8-sig', newline='') as stream:
        writer = csv.DictWriter(stream, list(rows[0]) if rows else [])
        writer.writeheader()
        writer.writerows(rows)
    report = {'classification': 'Descriptive statistics of recorded measurements; no rescoring',
              'summary_sha256': hashlib.sha256(source).hexdigest(),
              'all_passed': summary.get('all_passed'), 'counts': summary['counts'],
              'source_unchanged': summary.get('source_unchanged'),
              'installation_unchanged': summary.get('installation_unchanged'),
              'groups': groups, 'trials': rows}
    (args.output / 'metrics-summary.json').write_text(
        json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False) + '\n', encoding='utf-8')
    print(json.dumps({'rows': len(rows), 'counts': summary['counts'], 'output': str(args.output)}))


if __name__ == '__main__':
    main()
