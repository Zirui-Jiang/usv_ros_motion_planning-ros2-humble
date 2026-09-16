"""Replay captured Hybrid inputs offline; never creates ROS nodes or commands."""
import argparse
import cProfile
import io
import json
from pathlib import Path
import pstats
import sys
import time

import numpy as np

ROOT=Path(__file__).resolve().parents[2]
for package in ('shared/common','shared/usv_navigation','control_module/controller',
                'decision_module/path_planner','decision_module/trajectory_planner','decision_module/usv_decision'):
    sys.path.insert(0,str(ROOT/'src/core'/package))
from common.models import GridSnapshot, VesselState
from path_planner import PathPlannerCore
from usv_navigation.configuration import section
from usv_decision.runtime import PlanningRequest, plan_navigation


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('capture',type=Path)
    parser.add_argument('--timeout',type=float,default=1.)
    parser.add_argument('--profile',action='store_true')
    parser.add_argument('--dubins-heuristic',action='store_true',help='offline-only candidate heuristic experiment')
    parser.add_argument('--heuristic-weight',type=float,help='override search weight for this offline replay')
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    values=json.loads((args.capture/'snapshot.json').read_text(encoding='utf-8'))
    with np.load(args.capture/'map.npz') as f:
        grid=GridSnapshot(f['data'],float(f['resolution']),float(f['origin_x']),float(f['origin_y']),
                          float(f['stamp']),str(f['frame_id']))
    state=VesselState(**values['state']); cfg=values['parameters'].copy()
    cfg['navigation.planning_timeout']=args.timeout
    if args.heuristic_weight is not None:
        cfg['path_planner.heuristic_weight']=args.heuristic_weight
    planner_cfg=section(cfg,'path_planner.')
    planner_cfg.update(planner_name=cfg['planner_name'],obstacle_factor=cfg['obstacle_factor'],
                       timeout=args.timeout,max_expansions=cfg['navigation.max_expansions'],
                       footprint_length=cfg['navigation.footprint_length'],footprint_width=cfg['navigation.footprint_width'])
    planner=PathPlannerCore(planner_cfg)
    if args.dubins_heuristic:
        from path_planner.graph_planner.dubins import dubins_paths
        type(planner.planner)._heuristic=lambda self,pose,goal: min(
            curve.length for curve in dubins_paths(pose,goal,self.minimum_turning_radius))
    profiler=cProfile.Profile()
    if args.profile:
        profiler.enable()
    found=planner.make_plan(state.pose,values['goal'],grid)
    if args.profile:
        profiler.disable()
    request=PlanningRequest(values['version'],tuple(values['goal']),state,grid,np.empty((0,3)),cfg)
    response=plan_navigation(request)
    cells={key:grid.world_to_map(*point[:2]) for key,point in [('start',state.pose),('goal',values['goal'])]}
    report=dict(timeout=args.timeout,dubins_heuristic=args.dubins_heuristic,heuristic_weight=planner.planner.heuristic_weight,start=state.pose,goal=values['goal'],
                costs={key:int(grid.data[cell[1],cell[0]]) if cell else None for key,cell in cells.items()},
                planner=dict(status=found.status.value,message=found.message,elapsed=found.elapsed,
                             expanded=found.expanded,points=len(found.path)),
                runtime=dict(status=response.status.value,message=response.message,elapsed=response.elapsed,
                             expanded=response.expanded,points=len(response.route),trajectory_points=len(response.trajectory.points)))
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(json.dumps(report,indent=2),encoding='utf-8')
    if len(found.path):
        np.save(args.output.with_suffix('.path.npy'),found.path)
    if args.profile:
        stream=io.StringIO()
        pstats.Stats(profiler,stream=stream).sort_stats('cumtime').print_stats(25)
        args.output.with_suffix('.profile.txt').write_text(stream.getvalue(),encoding='utf-8')
    print(json.dumps(report))


if __name__=='__main__':
    main()
