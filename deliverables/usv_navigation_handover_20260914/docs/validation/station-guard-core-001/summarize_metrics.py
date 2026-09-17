"""Build compact tables from the archived measured trial results."""
import argparse
import csv
import json
from pathlib import Path
import shutil


def write_csv(path, rows):
    if not rows:
        raise ValueError(f'No measured rows for {path}')
    with path.open('w', encoding='utf-8-sig', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('archive', type=Path)
    args = parser.parse_args()
    folder = args.archive
    matrix = folder / 'raw/results/mission-matrix-001'
    summary = json.loads((matrix / 'summary.json').read_text())
    trials, windows, graphs, gaps, radial = [], [], [], [], []
    for task in summary['results']:
        m = task.get('measurement') or {}
        graph = m.get('control_ownership_graph') or {}
        normal = m.get('normal_orbit_radial_error_m') or {}
        gap = m.get('minimum_controlled_obstacle_footprint_gap_m')
        trials.append(dict(trial_id=task['trial_id'], scenario=task['scenario'],
            repeat=task['matrix_repeat'], domain_id=task['domain_id'], result=task['result'],
            passed=task['passed'], duration_sim_s=m.get('duration_sim_s'),
            sample_count=m.get('sample_count'), normal_orbit_radial_p95_m=normal.get('p95'),
            minimum_obstacle_footprint_gap_m=gap,
            independent_laps_passed=(m.get('independent_orbit_progress') or {}).get('passed'),
            original_mission_preserved=m.get('original_mission_preserved'),
            graph_observation_count=graph.get('observation_count'), graph_passed=graph.get('passed')))
        graphs.append(graph)
        if gap is not None:
            gaps.append(gap)
        if normal.get('p95') is not None:
            radial.append(normal['p95'])
        for name, window in (m.get('windows') or {}).items():
            position, heading = window['position_error_m'], window['heading_error_deg']
            windows.append(dict(trial_id=task['trial_id'], window=name,
                start_sim_s=window['start_sim_s'], end_sim_s=window['end_sim_s'],
                sample_count=window['sample_count'], complete=window['complete'],
                joint_fraction=window['joint_fraction'], position_max_m=position['max'],
                position_p95_m=position['p95'], heading_max_deg=heading['max'],
                heading_p95_deg=heading['p95'], passed=window['passed']))
        if task['matrix_repeat'] == 1:
            source = matrix / task['trial_id'] / m['trial']
            target = folder / 'figures' / task['scenario']
            target.mkdir(parents=True, exist_ok=True)
            for stem in ('mission_plots', 'independent_orbit_progress'):
                for ext in ('png', 'pdf'):
                    path = source / f'{stem}.{ext}'
                    if path.is_file():
                        shutil.copy2(path, target / path.name)
    write_csv(folder / 'trial-metrics.csv', trials)
    write_csv(folder / 'station-windows.csv', windows)
    measured = dict(
        matrix_all_passed=summary['all_passed'], counts=summary['counts'],
        station_window_count=len(windows), station_windows_all_passed=all(w['passed'] for w in windows),
        station_window_sample_counts=sorted({w['sample_count'] for w in windows}),
        minimum_joint_fraction=min(w['joint_fraction'] for w in windows),
        maximum_window_position_error_m=max(w['position_max_m'] for w in windows),
        maximum_window_heading_error_deg=max(w['heading_max_deg'] for w in windows),
        maximum_normal_orbit_radial_p95_m=max(radial) if radial else None,
        minimum_controlled_obstacle_footprint_gap_m=min(gaps) if gaps else None,
        graph_observation_count=sum(g.get('observation_count', 0) for g in graphs),
        graph_all_passed=all(g.get('passed') is True for g in graphs),
        figure_selection='The first repeat of every scenario, selected by repeat number only.',
    )
    (folder / 'measured-summary.json').write_text(json.dumps(measured, indent=2), encoding='utf-8')
    print(json.dumps(measured))


if __name__ == '__main__':
    main()
