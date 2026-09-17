#!/usr/bin/env python3
"""Read-only damping comparison and four actual-state LOS query experiments."""
import argparse
import bisect
import importlib.util
import json
import math
from pathlib import Path
import sys

import numpy as np


def module(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    value = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(value)
    return value


def early_thrust_observations(trial):
    odom=trial['odom'];times=[e['wall_monotonic_s'] for e in odom]
    commands=[];pending={}
    for e in trial['active']:
        if e['event']!='thruster_command':continue
        pending[e['axis']]=e
        if not all(k in pending for k in ('left','right','lateral')):continue
        entries=[pending[k] for k in ('left','right','lateral')]
        if max(x['wall_monotonic_s'] for x in entries)-min(x['wall_monotonic_s'] for x in entries)>.01:continue
        wall=max(x['wall_monotonic_s'] for x in entries);idx=bisect.bisect_right(times,wall)-1
        if idx>=0:
            state=odom[idx];left,right=[pending[k]['normalized_command'] for k in ('left','right')]
            commands.append({'command_lines':[x['event_line'] for x in entries],'wall_s':wall,
                'source_state_time_s':state['sim_time_s'],'state_receipt_age_s':wall-state['wall_monotonic_s'],
                'pose':state['pose'],'body_velocity':state['body_velocity'],
                'left':left,'right':right,'sum':left+right,'right_minus_left':right-left})
        pending={}
    moving=[x for x in commands if max(abs(x['left']),abs(x['right']))>1e-6]
    aligned=next((e for e in odom if e['pose'][2]<=-math.pi/2),None)
    return {'interpretation':'Received commands paired with preceding odom receipt, not atomic control input. Sum >.1/.3 are diagnostic markers, not acceptance thresholds.',
        'first_nonzero':moving[0] if moving else None,
        'first_sum_above_010':next((x for x in moving if x['sum']>.1),None),
        'first_sum_above_030':next((x for x in moving if x['sum']>.3),None),
        'first_heading_below_minus_pi_over_2':aligned,
        'initial_command_sequence':moving[:20]}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--repo-root', type=Path, default=Path(__file__).resolve().parents[3])
    parser.add_argument('--results-root', type=Path, default=Path('/home/usv/usv_migration/results'))
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    repo, root, output = args.repo_root.resolve(), args.results_root.resolve(), args.output.resolve()
    if output.exists() or output == root or root in output.parents:
        parser.error('output must be new and outside raw results')
    for relative in ('src/core/common','src/core/path_planner/path_planner','src/core/trajectory_planner'):
        sys.path.insert(0,str(repo/relative))
    from common.models import USVModel, VesselState
    from path_planner.path_planner import prune
    from trajectory_planner.los_trajectory_planner import LosTrajectoryPlanner
    source=repo/'docs/validation/source-time-candidate-diagnostics/reproduce.py'
    d=module('previous_diagnostic',source)
    geometry=module('geometry',repo/'scripts/validation/metrics.py')
    originals={};result={'scope':'Actual kd0 vs kd010 completed turn traces; current Python LOS executed at only four reconstructed cold planning inputs. No Gazebo, no modified parameters or acceptance score.',
        'segment_definition':'All source-time e-squared trapezoids, midpoint progress -y-10.125m: <3,3..9,>=9. r RMS uses the same interval-duration weights.',
        'planning_input_limit':'First received global path pruned afresh at selected recorded odom; map->odom identity is the explicitly verified otter_fake_localization contract. Actual worker route/state/reference and timing were asynchronous and are not reconstructed atomically. No map/collision checks or future physical-state rollout here.',
        'cpp_limit':'C++ formula is checked from preserved source only. Analytic straight-line values below are not C++ executable outputs or recorded reference velocities.',
        'source_files':{str(p):d.digest(p) for p in [Path(__file__),source,repo/'scripts/validation/metrics.py',
          repo/'src/core/trajectory_planner/src/los_trajectory_planner.cpp',repo/'src/core/trajectory_planner/src/trajectory.cpp',
          repo/'src/core/trajectory_planner/trajectory_planner/los_trajectory_planner.py',repo/'src/core/trajectory_planner/trajectory_planner/trajectory.py',
          repo/'src/core/path_planner/path_planner/path_planner/path_planner.py']},'paired_turns':[],'los_inputs':[]}
    for repeat in (1,2,3):
        pair={}
        for label,directory in [('kd0','python-grid-endpoint-final-matrix-001'),('kd010','python-pid-course-d010-turn-candidate-001')]:
            t=d.load_trial(root/directory/f'python_pid_turn_{repeat:02d}')
            dec=d.decomposition(t,geometry,preview=2.6)
            bins=[{'r2_integral':0.,'r_max_abs':0.,'heading_min':math.inf,'heading_max':-math.inf} for _ in range(3)]
            for a,b in zip(t['odom'],t['odom'][1:]):
                dt=b['sim_time_s']-a['sim_time_s'];progress=-(a['pose'][1]+b['pose'][1])/2-10.125
                entry=bins[0 if progress<3 else 1 if progress<9 else 2]
                entry['r2_integral']+=dt*(a['body_velocity'][2]**2+b['body_velocity'][2]**2)/2
                entry['r_max_abs']=max(entry['r_max_abs'],abs(a['body_velocity'][2]),abs(b['body_velocity'][2]))
                entry['heading_min']=min(entry['heading_min'],a['pose'][2],b['pose'][2])
                entry['heading_max']=max(entry['heading_max'],a['pose'][2],b['pose'][2])
            for entry,errors in zip(bins,dec['bins']):
                entry['r_rms_radps']=math.sqrt(entry['r2_integral']/errors['source_duration_s']) if errors['source_duration_s'] else None
            pair[label]={'directory':t['directory'],'events_sha256':t['events_sha256'],'summary_sha256':t['summary_sha256'],
                'source_rmse_m':dec['source_rmse_m'],'source_span_s':dec['source_duration_s'],'error_bins':dec['bins'],'yaw_bins':bins,
                'early_thrust':early_thrust_observations(t)}
            if label=='kd0':originals[repeat]=t
        pair['repeat']=repeat
        pair['source_rmse_kd010_divided_by_kd0']=pair['kd010']['source_rmse_m']/pair['kd0']['source_rmse_m']
        pair['mse_contribution_delta_by_bin']=[b['contribution_to_full_mse_m2']-a['contribution_to_full_mse_m2'] for a,b in zip(pair['kd0']['error_bins'],pair['kd010']['error_bins'])]
        result['paired_turns'].append(pair)
    t=originals[1]
    events=[json.loads(line) for line in (Path(t['directory'])/'events.jsonl').open() if line.strip()]
    params=next(e['parameters'] for e in events if e['event']=='controller_verified')
    model=USVModel.from_mapping({k[len('dynamics.'):]:v for k,v in params.items() if k.startswith('dynamics.')})
    prefix='usv_simulation.trajectory_planner.los.'
    los_config={k[len(prefix):]:v for k,v in params.items() if k.startswith(prefix)}
    los_config.setdefault('slowdown_distance',3.)
    route0=np.column_stack((np.asarray(t['path']),np.zeros(len(t['path']))))
    for progress in (0.,3.,6.,9.):
        event=t['odom'][0] if progress==0 else min(t['odom'],key=lambda e:abs(-e['pose'][1]-10.125-progress))
        state=VesselState(*event['pose'],*event['body_velocity'],event['sim_time_s'],'otter/map')
        route=prune(route0,state)
        cfg=dict(los_config)
        endpoint_offset=float(np.linalg.norm(route[-1,:2]-t['goal'][:2]))
        cfg['goal_tolerance']=min(cfg['goal_tolerance'],params['usv_simulation.controller.pid_controller.goal_distance_tolerance']-endpoint_offset)
        planner=LosTrajectoryPlanner(model,cfg);generated=planner.plan(route,state)
        trajectory=generated.trajectory
        if trajectory.empty():raise RuntimeError(generated.message)
        initial_distance=float(np.linalg.norm(route[1,:2]-np.array(state.pose[:2]))+np.linalg.norm(np.diff(route[1:,:2],axis=0),axis=1).sum()) if len(route)>1 else float(np.linalg.norm(route[0,:2]-state.pose[:2]))
        euclidean=float(np.linalg.norm(route[-1,:2]-state.pose[:2]))
        refs=[]
        for preview in (2.6,3.0):
            reference=trajectory.reference_at_time(preview)
            error=geometry.wrap_angle(reference.theta-state.psi);yaw_raw=2.*error;yaw_wrapped=geometry.wrap_angle(yaw_raw)
            refs.append({'preview_s':preview,'query_age_s':0.,'reference':reference._asdict(),'course_error_rad':error,
                'yaw_unwrapped_kd0':yaw_raw,'yaw_wrapped_kd0':yaw_wrapped,'wrap_crossing':bool(abs(yaw_raw)>=math.pi),
                'raw_differential_before_thruster_clipping':yaw_wrapped/model.D,
                'speed_error_mps':reference.velocity-math.hypot(state.u,state.v)})
        result['los_inputs'].append({'target_progress_m':progress,'actual_progress_m':-state.y-10.125,'odom_event':event,
            'route_pruned':route.tolist(),'goal':t['goal'],'los_config':cfg,'model':model.__dict__,'initial_route_remaining_m':initial_distance,
            'initial_euclidean_goal_m':euclidean,'status':generated.status.value,'trajectory_rows':trajectory.points.tolist(),
            'references':refs,'theta_delta_30_minus_26':geometry.wrap_angle(refs[1]['reference']['theta']-refs[0]['reference']['theta']),
            'velocity_delta_30_minus_26':refs[1]['reference']['velocity']-refs[0]['reference']['velocity'],
            'analytic_straight_only':{'python_continuous_v_at_26':model.cruising_speed*math.exp(-model.cruising_speed*2.6/initial_distance),
              'python_continuous_v_at_30':model.cruising_speed*math.exp(-model.cruising_speed*3./initial_distance),
              'cpp_label_30_physical_26_linear_v':model.cruising_speed*(1-model.cruising_speed*2.6/euclidean)}})
    output.parent.mkdir(parents=True,exist_ok=True)
    output.write_text(json.dumps(result,ensure_ascii=False,indent=2,allow_nan=False)+'\n')
    print(json.dumps({'output':str(output),'sha256':d.digest(output),'turn_ratios':[x['source_rmse_kd010_divided_by_kd0'] for x in result['paired_turns']],
         'queries':[{'progress':x['target_progress_m'],'references':x['references']} for x in result['los_inputs']]}))


if __name__=='__main__':main()
