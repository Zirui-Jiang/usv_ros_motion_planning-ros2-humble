#!/usr/bin/env python3
"""Read-only diagnostics of completed e381/preview-2.5 records; no ROS needed.

Writes one new JSON outside the raw-results tree. Does not alter acceptance
thresholds, original scores, archived verdicts, observations or control inputs.
"""
import argparse
import bisect
import hashlib
import importlib.util
import json
import math
from pathlib import Path


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def module(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    result = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(result)
    return result


def compact_pair(pair, report):
    key = pair['controller'], pair['scenario'], pair['repeat']
    members = [record for record in report['trials'] if not record['result'].get('fault')
               and tuple(record['result'].get(k) for k in ('controller', 'scenario', 'repeat')) == key]
    return {'controller': key[0], 'scenario': key[1], 'repeat': key[2],
            'comparable': pair['comparable'], 'passed': pair['passed'], 'reasons': pair['reasons'],
            'source_rmse': pair['metrics'].get('cross_track_rmse_m'),
            'legacy_wall_rmse': pair.get('rmse_alternatives', {}).get('legacy_wall'),
            'duration_sim_s': pair['metrics'].get('duration_sim_s'),
            'observations': {r['result']['mode']: {'trial': r['result']['trial'], 'source': r['source'],
                'events_path': r['observations']['events_path'], 'events_sha256': r['observations']['events_sha256'],
                'source_score': r['observations']['tracking']['source_time_trapezoid'],
                'legacy_score': r['observations']['tracking']['legacy_wall']} for r in members}}


def load_trial(directory):
    path = directory / 'events.jsonl'
    events = [dict(json.loads(line), event_line=i) for i, line in enumerate(path.open(), 1) if line.strip()]
    start = next(i for i, e in enumerate(events) if e['event'] == 'goal_published')
    end = next(i for i, e in enumerate(events) if i > start and e['event'] == 'acquisition_finished')
    active = events[start:end + 1]
    preceding = [e for e in events[:start] if e['event'] == 'odom']
    odom = preceding[-1:] + [e for e in active if e['event'] == 'odom']
    unique = []
    for event in odom:
        if unique and event['sim_time_s'] == unique[-1]['sim_time_s']:
            if event['pose'] != unique[-1]['pose']:
                raise ValueError('same source stamp has conflicting poses')
            continue
        if unique and event['sim_time_s'] < unique[-1]['sim_time_s']:
            raise ValueError('source clock rollback')
        unique.append(event)
    path_event = next(e for e in active if e['event'] == 'reference_path' and e.get('points'))
    return {'trial': directory.name, 'directory': str(directory), 'events_sha256': digest(path),
            'summary_sha256': digest(directory / 'summary.json'), 'active': active, 'odom': unique,
            'path': path_event['points'], 'first_path_event_line': path_event['event_line'],
            'goal': events[start]['goal']}


def decomposition(trial, geometry, *, preview, counterfactual_path=None):
    odom, path = trial['odom'], trial['path']
    span = odom[-1]['sim_time_s'] - odom[0]['sim_time_s']
    error = [geometry.point_to_polyline_distance(e['pose'], path) for e in odom]
    alternative = path if counterfactual_path is None else counterfactual_path
    cf_error = [geometry.point_to_polyline_distance(e['pose'], alternative) for e in odom]
    turning = 'turn' in trial['trial']

    def progress(event):
        return -event['pose'][1] - 10.125 if turning else event['pose'][0] - 23.125

    bins = [{'along_route_interval_m': interval, 'source_duration_s': 0., 'error_energy_m2_s': 0.,
             'source_intervals': 0, 'speed_integral': 0., 'heading_integral': 0., 'lateral_integral': 0.}
            for interval in ([None, 3.], [3., 9.], [9., None])]
    energy, cf_energy = 0., 0.
    for i, (a, b) in enumerate(zip(odom, odom[1:])):
        dt = b['sim_time_s'] - a['sim_time_s']
        value = dt * (error[i] ** 2 + error[i + 1] ** 2) / 2
        energy += value
        cf_energy += dt * (cf_error[i] ** 2 + cf_error[i + 1] ** 2) / 2
        position = (progress(a) + progress(b)) / 2
        item = bins[0 if position < 3 else 1 if position < 9 else 2]
        item['source_duration_s'] += dt
        item['error_energy_m2_s'] += value
        item['source_intervals'] += 1
        item['speed_integral'] += dt * (math.hypot(*a['body_velocity'][:2]) + math.hypot(*b['body_velocity'][:2])) / 2
        item['heading_integral'] += dt * (a['pose'][2] + b['pose'][2]) / 2
        axis = 0 if turning else 1
        item['lateral_integral'] += dt * (a['pose'][axis] + b['pose'][axis]) / 2
    for item in bins:
        duration = item['source_duration_s']
        item['local_rmse_m'] = math.sqrt(item['error_energy_m2_s'] / duration) if duration else None
        item['contribution_to_full_mse_m2'] = item['error_energy_m2_s'] / span
        for field in ('speed', 'heading', 'lateral'):
            item['mean_' + field] = item.pop(field + '_integral') / duration if duration else None
    refs = [e for e in trial['active'] if e['event'] == 'local_trajectory' and e.get('poses')]
    ref_wall = [e['wall_monotonic_s'] for e in refs]
    checkpoints = []
    for target in (1., 3., 6., 9., 10., 11.):
        event = min(odom, key=lambda e: abs(progress(e) - target))
        index = bisect.bisect_right(ref_wall, event['wall_monotonic_s']) - 1
        ref = refs[index] if index >= 0 else None
        ref_pose, query = None, None
        if ref:
            query = preview + (max(0., event['sim_time_s'] - ref['sim_time_s']) if trial['trial'].startswith('python') else 0.)
            lower = min(int(query / .5), len(ref['poses']) - 1)
            upper = min(lower + 1, len(ref['poses']) - 1)
            fraction = min(1., max(0., query / .5 - lower))
            a, b = ref['poses'][lower], ref['poses'][upper]
            ref_pose = [a[0] + fraction * (b[0] - a[0]), a[1] + fraction * (b[1] - a[1]),
                        geometry.wrap_angle(a[2] + fraction * geometry.wrap_angle(b[2] - a[2]))]
        checkpoints.append({'target_along_route_m': target, 'actual_along_route_m': progress(event),
                            'source_time_s': event['sim_time_s'], 'pose': event['pose'],
                            'speed_m_s': math.hypot(*event['body_velocity'][:2]),
                            'error_m': geometry.point_to_polyline_distance(event['pose'], path),
                            'last_received_reference_query_s': query, 'last_received_reference_preview_pose': ref_pose,
                            'reference_receipt_age_s': event['wall_monotonic_s'] - ref['wall_monotonic_s'] if ref else None})
    first_disk = next((e for e in odom if math.dist(e['pose'][:2], trial['goal'][:2]) <= 1.), None)
    last = odom[-1]
    return {'trial': trial['trial'], 'directory': trial['directory'], 'events_sha256': trial['events_sha256'],
            'summary_sha256': trial['summary_sha256'], 'goal': trial['goal'], 'preview_s': preview,
            'first_path_event_line': trial['first_path_event_line'], 'first_path_last3': path[-3:],
            'source_duration_s': span, 'integrated_error_m2_s': energy, 'source_rmse_m': math.sqrt(energy / span),
            'bins': bins, 'checkpoints': checkpoints,
            'counterfactual': {'definition': 'Only rescore these exact observed poses after replacing the scoring path endpoint; not a controller/LOS rollout or simulation.',
                'path_last3': alternative[-3:], 'source_rmse_m': math.sqrt(cf_energy / span),
                'integrated_error_m2_s': cf_energy, 'maximum_observed_error_change_m': max(abs(a - b) for a, b in zip(error, cf_error))},
            'first_requested_goal_disk_observation': None if first_disk is None else {
                'source_time_s': first_disk['sim_time_s'], 'pose': first_disk['pose'], 'speed_m_s': math.hypot(*first_disk['body_velocity'][:2])},
            'last_observation': {'source_time_s': last['sim_time_s'], 'pose': last['pose'],
                'requested_goal_distance_m': math.dist(last['pose'][:2], trial['goal'][:2]),
                'speed_m_s': math.hypot(*last['body_velocity'][:2])}}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--results-root', type=Path, default=Path('/home/usv/usv_migration/results'))
    parser.add_argument('--repo-root', type=Path, default=Path(__file__).resolve().parents[3])
    parser.add_argument('--output', required=True, type=Path)
    args = parser.parse_args()
    output, root, repo = args.output.resolve(), args.results_root.resolve(), args.repo_root.resolve()
    if output.exists() or output == root or root in output.parents:
        parser.error('output must be new and outside raw results')
    compare = module('diagnostic_compare', repo / 'scripts/validation/compare_results.py')
    geometry = module('diagnostic_geometry', repo / 'scripts/validation/metrics.py')
    baseline = ('cpp', root / 'baseline-grid-interior-001')
    old = compare.build_report([('python', root / 'python-release-grid-001'), baseline],
                               rmse_metric='source_time_trapezoid', frame_contract='otter_fake_localization')
    candidate_dirs = ['python-preview25-straight-candidate-001', 'python-preview25-turn-candidate-001', 'python-preview25-obstacle-candidate-001']
    candidate = compare.build_report([('python', root / name) for name in candidate_dirs] + [baseline],
                                     rmse_metric='source_time_trapezoid', frame_contract='otter_fake_localization')
    results = {'purpose': 'Read-only retrospective diagnostic; not a replacement for frozen historical acceptance reports or a new physical rerun.',
               'method': old['method'], 'e381_source_diagnostic_summary': old['summary'],
               'e381_pairs': [compact_pair(p, old) for p in old['comparisons']],
               'preview25_pairs': [compact_pair(p, candidate) for p in candidate['comparisons'] if p['python_source'] is not None],
               'decomposition_definition': 'Along-route progress is x-23.125 for straight, -y-10.125 for turn. Assign each complete observed source-time trapezoid to the bin of its midpoint progress (<3,3..9,>=9m). All intervals are retained, including tiny negative initial progress; energies sum exactly. Spatial boundary assignment is a diagnostic discretization, not a new acceptance score.',
               'reference_definition': 'Reconstruct preview pose from latest received published trajectory and its 0.5s pose spacing. Python query is state-source age plus configured preview; C++ uses fixed relative preview and ignores its erroneous published absolute header. This is not proof of which asynchronous callback/reference actually produced each command.',
               'source_code': {name: {'path': str(repo / name), 'sha256': digest(repo / name)} for name in (
                   'scripts/validation/compare_results.py', 'scripts/validation/tracking_metrics.py', 'scripts/validation/metrics.py')},
               'script_sha256': digest(Path(__file__)), 'decompositions': {}}
    for scenario, repeat, directory in [('straight', 2, candidate_dirs[0]), ('turn', 1, candidate_dirs[1])]:
        py = load_trial(root / directory / f'python_pid_{scenario}_{repeat:02d}')
        cpp = load_trial(root / 'baseline-grid-interior-001' / f'cpp_pid_{scenario}_{repeat:02d}')
        p = decomposition(py, geometry, preview=2.5, counterfactual_path=py['path'][:-1] + [cpp['path'][-1]])
        c = decomposition(cpp, geometry, preview=3.)
        delta = [a['contribution_to_full_mse_m2'] - b['contribution_to_full_mse_m2'] for a, b in zip(p['bins'], c['bins'])]
        results['decompositions'][scenario] = {'prefix_paths_identical': py['path'][:-1] == cpp['path'][:-1],
            'preview25': p, 'cpp': c, 'full_mse_delta_by_bin': delta,
            'tail_fraction_of_net_mse_delta': delta[-1] / sum(delta)}
        if scenario == 'turn':
            previous = load_trial(root / 'python-release-grid-001' / 'python_pid_turn_01')
            previous_score = decomposition(previous, geometry, preview=2.6)
            results['decompositions'][scenario]['e381_preview26'] = previous_score
            results['decompositions'][scenario]['preview25_minus_preview26_mse_by_bin'] = [
                a['contribution_to_full_mse_m2'] - b['contribution_to_full_mse_m2'] for a, b in zip(p['bins'], previous_score['bins'])]
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(results, ensure_ascii=False, indent=2, allow_nan=False) + '\n', encoding='utf-8')
    print(json.dumps({'output': str(output), 'sha256': digest(output),
                      'candidate_source_passed': sum(p['passed'] is True for p in results['preview25_pairs']),
                      'candidate_count': len(results['preview25_pairs'])}))


if __name__ == '__main__':
    main()
