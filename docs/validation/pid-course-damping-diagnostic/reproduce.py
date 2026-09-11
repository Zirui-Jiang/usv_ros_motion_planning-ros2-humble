#!/usr/bin/env python3
"""Conditional open-loop course PD replay from recorded poses, not a ROS callback replay."""
import argparse
import bisect
import importlib.util
import json
import math
from pathlib import Path

import numpy as np


def module(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    out = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(out)
    return out


def distribution(values):
    if not values:
        return {'count': 0}
    a = np.asarray(values)
    return {'count': len(values), 'mean': float(a.mean()), 'rms': float(np.sqrt(np.mean(a*a))),
            'p95_abs': float(np.percentile(abs(a), 95)), 'max_abs': float(max(abs(a)))}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--repo-root', type=Path, default=Path(__file__).resolve().parents[3])
    parser.add_argument('--results-root', type=Path, default=Path('/home/usv/usv_migration/results'))
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    repo, root, output = args.repo_root.resolve(), args.results_root.resolve(), args.output.resolve()
    if output.exists() or output == root or root in output.parents:
        parser.error('output must be new and outside raw results')
    dependency = repo/'docs/validation/source-time-candidate-diagnostics/reproduce.py'
    d = module('previous_diagnostic', dependency)
    g = module('geometry', repo/'scripts/validation/metrics.py')
    cases = {'formal':'python-grid-endpoint-final-matrix-001/python_pid_turn_01',
             'candidate':'python-grid-endpoint-turn-candidate-001/python_pid_turn_01',
             'cpp':'baseline-grid-interior-001/cpp_pid_turn_01'}
    result = {'purpose':'Conditional replay of a recorded state/reference sequence with Kp=2, Ki=0, Kd=0 vs 0.10. No closed-loop or Gazebo prediction.',
        'reference_selection':'Latest nonempty published trajectory received at each unique source odom receipt; 0.5s pose interpolation. Python uses max(0,state_stamp-reference_stamp)+2.6s; CPP uses nominal3s without its invalid absolute header. Actual control callback state/reference identity is not recorded.',
        'dt':'Adjacent unique source odom interval, not an asserted actual controller dt. First usable reference uses nominal0.1s and previous_error0; initialization kick separately flagged.',
        'allocation':'Course effort yaw=wrap(2*e+Kd*wrap(e-e_previous)/dt). Raw differential=(right-left)=yaw/D, D=.395. Conditional individual thrusters are computed ONLY where the latest recorded complete three-axis command has both longitudinal axes strictly unsaturated: its sum is used as a fixed observed-surge proxy for BOTH replays. This proxy is not proven to equal the replay reference speed-controller output. Saturated observed cases lack recoverable raw surge and have null full allocation.',
        'limits':'No reference velocity, PID speed integral, atomic control inputs or per-control dt in events. No exact replay of the full actual PID, no claim to lower future RMSE.',
        'source_files':{str(p):d.digest(p) for p in [Path(__file__),dependency,repo/'scripts/validation/metrics.py',repo/'src/core/controller/controller/controller/pid_controller.py']},
        'trials':{}}
    for label, relative in cases.items():
        t=d.load_trial(root/relative);refs=[e for e in t['active'] if e['event']=='local_trajectory' and e.get('poses')]
        ref_times=[e['wall_monotonic_s'] for e in refs]
        events=[dict(json.loads(line), event_line=i) for i,line in enumerate((root/relative/'events.jsonl').open(),1) if line.strip()]
        complete=[];pending={}
        for event in events:
            if event['event']=='thruster_command':
                pending[event['axis']]=event
                if all(k in pending for k in ('left','right','lateral')):
                    axes=[pending[k] for k in ('left','right','lateral')]
                    if max(e['wall_monotonic_s'] for e in axes)-min(e['wall_monotonic_s'] for e in axes)<.01:
                        complete.append({'wall':max(e['wall_monotonic_s'] for e in axes),
                            'command':[pending[k]['normalized_command'] for k in ('left','right','lateral')],
                            'lines':[pending[k]['event_line'] for k in ('left','right','lateral')]})
                        pending={}
        command_times=[c['wall'] for c in complete]
        previous=None;rows=[]
        for state in t['odom']:
            ri=bisect.bisect_right(ref_times,state['wall_monotonic_s'])-1
            if ri<0 or math.dist(state['pose'][:2],t['goal'][:2])<=1.5:
                continue
            ref=refs[ri]
            query=3. if label=='cpp' else 2.6+max(0.,state['sim_time_s']-ref['sim_time_s'])
            lo=min(int(query/.5),len(ref['poses'])-1);hi=min(lo+1,len(ref['poses'])-1);f=min(1.,max(0.,query/.5-lo))
            theta=g.wrap_angle(ref['poses'][lo][2]+f*g.wrap_angle(ref['poses'][hi][2]-ref['poses'][lo][2]))
            e=g.wrap_angle(theta-state['pose'][2]);dt=.1 if previous is None else state['sim_time_s']-previous['source_time_s']
            de=g.wrap_angle(e-(0. if previous is None else previous['course_error_rad']))/dt
            p=2.*e;derivative=.1*de;y0=g.wrap_angle(p);y1=g.wrap_angle(p+derivative)
            row={'source_time_s':state['sim_time_s'],'source_dt_s':dt,'pose':state['pose'],'body_velocity':state['body_velocity'],
                 'progress_m':-state['pose'][1]-10.125,'cross_track_signed_m':state['pose'][0]-23.25,
                 'world_lateral_velocity_mps':state['body_velocity'][0]*math.cos(state['pose'][2])-state['body_velocity'][1]*math.sin(state['pose'][2]),
                 'reference_event_line':ref['event_line'],'odom_event_line':state['event_line'],'query_s':query,
                 'reference_theta_rad':theta,'course_error_rad':e,'error_rate_radps':de,'initialization':previous is None,
                 'reference_changed':previous is not None and ref['event_line']!=previous['reference_event_line'],
                 'p_effort':p,'d_effort':derivative,'gyro_only_diagnostic_effort':-.1*state['body_velocity'][2],
                 'yaw_unwrapped_kd0':p,'yaw_unwrapped_kd010':p+derivative,'yaw_wrapped_kd0':y0,'yaw_wrapped_kd010':y1,
                 'raw_differential_kd0':y0/.395,'raw_differential_kd010':y1/.395,'conditional_allocation':None}
            ci=bisect.bisect_right(command_times,state['wall_monotonic_s'])-1
            if ci>=0:
                c=complete[ci];row['latest_received_command']=c;row['command_receipt_age_s']=state['wall_monotonic_s']-c['wall']
                left,right,_=c['command']
                if abs(left)<1-1e-6 and abs(right)<1-1e-6 and row['command_receipt_age_s']<=.2:
                    surge=left+right
                    def allocate(yaw):
                        raw=[.5*(surge-yaw/.395),.5*(surge+yaw/.395)]
                        return {'raw_left_right':raw,'clipped_left_right':[max(-1.,min(1.,x)) for x in raw]}
                    row['conditional_allocation']={'observed_surge_proxy':surge,'observed_effective_yaw':.395*(right-left),
                        'kd0':allocate(y0),'kd010':allocate(y1),'baseline_yaw_reconstruction_residual':y0-.395*(right-left)}
            rows.append(row);previous=row
        summaries={}
        for name,subset in [('front',[x for x in rows if x['progress_m']<3]),('middle',[x for x in rows if 3<=x['progress_m']<9]),('tail',[x for x in rows if x['progress_m']>=9])]:
            regular=[x for x in subset if not x['initialization']]
            rotating=[x for x in regular if abs(x['body_velocity'][2])>.1]
            conditional=[x for x in regular if x['conditional_allocation'] is not None]
            summary={'observations':len(subset),'course_error_rad':distribution([x['course_error_rad'] for x in subset]),
                'yaw_rate_radps':distribution([x['body_velocity'][2] for x in subset]),
                'derivative_effort':distribution([x['d_effort'] for x in regular]),
                'd_when_reference_changed':distribution([x['d_effort'] for x in regular if x['reference_changed']]),
                'd_when_reference_held':distribution([x['d_effort'] for x in regular if not x['reference_changed']]),
                'rotating_observations_abs_r_gt_010':len(rotating),
                'd_opposes_measured_rotation':sum(x['d_effort']*x['body_velocity'][2]<0 for x in rotating),
                'd_reinforces_measured_rotation':sum(x['d_effort']*x['body_velocity'][2]>0 for x in rotating),
                'conditional_allocation_count':len(conditional),
                'conditional_baseline_reconstruction_residual':distribution([x['conditional_allocation']['baseline_yaw_reconstruction_residual'] for x in conditional]),
                'conditional_max_clipped_change':max([max(abs(a-b) for a,b in zip(x['conditional_allocation']['kd0']['clipped_left_right'],x['conditional_allocation']['kd010']['clipped_left_right'])) for x in conditional],default=None)}
            summaries[name]=summary
        middle=[x for x in rows if 3<=x['progress_m']<9]
        observed=[x for x in middle if x.get('latest_received_command') and x['command_receipt_age_s']<=.2]
        wrap_check={name:{'maximum_absolute_unwrapped':max(abs(x[name]) for x in rows),
                          'crossed_pi_count':sum(abs(x[name])>=math.pi for x in rows)}
                    for name in ('yaw_unwrapped_kd0','yaw_unwrapped_kd010')}
        result['trials'][label]={'directory':t['directory'],'events_sha256':t['events_sha256'],'summary_sha256':t['summary_sha256'],
            'summaries':summaries,'yaw_wrap_check':wrap_check,
            'observed_middle_differential':distribution([x['latest_received_command']['command'][1]-x['latest_received_command']['command'][0] for x in observed]),
            'observed_middle_yaw_range_rad':[min(x['pose'][2] for x in middle),max(x['pose'][2] for x in middle)],'rows':rows}
    output.parent.mkdir(parents=True,exist_ok=True)
    output.write_text(json.dumps(result,ensure_ascii=False,indent=2,allow_nan=False)+'\n')
    print(json.dumps({'output':str(output),'sha256':d.digest(output),'summaries':{name:value['summaries'] for name,value in result['trials'].items()}}))


if __name__=='__main__':
    main()
