"""Replay measured pose progression with a cached global route on a frozen map."""
import argparse
from collections import Counter
import json
from pathlib import Path
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
for package in ('shared/common','shared/usv_navigation','control_module/controller',
                'decision_module/path_planner','decision_module/path_processor',
                'decision_module/trajectory_planner','decision_module/usv_decision'):
    sys.path.insert(0, str(ROOT/'src/core'/package))

from common.models import GridSnapshot, VesselState
from usv_navigation.configuration import resolve_parameters
from usv_decision.runtime import PlanningRequest, plan_navigation


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('capture',type=Path)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--sample-interval',type=float,default=.5)
    args=parser.parse_args()
    saved=json.loads((args.capture/'snapshot.json').read_text(encoding='utf-8'))
    events=json.loads((args.capture/'events.json').read_text(encoding='utf-8'))
    with np.load(args.capture/'map.npz') as data:
        grid=GridSnapshot(data['data'],float(data['resolution']),float(data['origin_x']),
                          float(data['origin_y']),float(data['stamp']),str(data['frame_id']))
    cfg=resolve_parameters(saved['parameters'])
    odom=[e for e in events if e['topic']=='odom']
    if not odom:
        raise ValueError('capture contains no odometry')
    # This replay is valid only for the verified identity fake-localization transform.
    if (not np.allclose([odom[-1]['x'],odom[-1]['y'],odom[-1]['yaw']],
                        [saved['state']['x'],saved['state']['y'],saved['state']['psi']],atol=1e-8)):
        raise ValueError('cannot infer identity map/odom alignment from this capture')
    route=np.empty((0,3)); results=[]; last=-float('inf')
    for event in odom:
        if event['stamp']-last < args.sample_interval-1e-6:
            continue
        last=event['stamp']
        # Historical probe stored speed magnitude. LOS uses position/yaw here;
        # placing it in surge is only a replay convention, not measured surge.
        state=VesselState(event['x'],event['y'],event['yaw'],event['speed'],0.,event['r'],
                          event['stamp'],grid.frame_id)
        response=plan_navigation(PlanningRequest(saved['version'],tuple(saved['goal']),state,grid,route,cfg))
        route=response.route
        results.append(dict(stamp=state.stamp,status=response.status.value,reason=response.message,
                            searched=response.searched,expanded=response.expanded,
                            route_points=len(route),trajectory_points=len(response.trajectory.points),
                            trajectory_stamp=response.trajectory.stamp,elapsed=response.elapsed))
    summary=dict(cycles=len(results),global_searches=sum(r['searched'] for r in results),
                 statuses=dict(Counter(r['status'] for r in results)),
                 elapsed_percentiles=np.percentile([r['elapsed'] for r in results],[50,95,100]).tolist(),
                 assumptions=['frozen final costmap for all historical poses',
                              'verified map/odom identity in this simulation only',
                              'speed magnitude placed in surge for reference-only replay'],
                 results=results)
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(json.dumps(summary,ensure_ascii=False,indent=2),encoding='utf-8')
    print(json.dumps({k:v for k,v in summary.items() if k!='results'}))


if __name__=='__main__':
    main()
