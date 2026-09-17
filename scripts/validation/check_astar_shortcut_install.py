"""Verify independent installed imports and A* shortcut behavior, without ROS nodes."""

import importlib
from pathlib import Path
import sys

import numpy as np
from ament_index_python.packages import get_package_prefix


def main():
    if not __debug__:
        raise RuntimeError('Run without Python optimization (-O / PYTHONOPTIMIZE); assertions are required')
    install = Path(sys.argv[1]).resolve()
    for package, module in (
            ('path_processor', 'path_processor.shortcut'),
            ('path_planner', 'path_planner.path_planner_core'),
            ('usv_navigation', 'usv_navigation.configuration'),
            ('usv_decision', 'usv_decision.decision')):
        assert Path(get_package_prefix(package)).resolve() == install/package
        origin = Path(importlib.import_module(module).__file__).resolve()
        assert origin.is_relative_to(install), (module, origin)
        print(f'{module}: {origin}')

    from common.models import GridSnapshot
    from path_planner import PathPlannerCore
    from usv_navigation.configuration import resolve_parameters, section
    from usv_navigation_interfaces.srv import NavigateTo, NavigateRoute

    grid = GridSnapshot(np.zeros((8, 8), np.uint8), 1.)
    start, goal = (.5, .5, .3), (5.5, 3.5, -.6)
    raw = PathPlannerCore({'enable_shortcut': False}).make_plan(start, goal, grid)
    pruned = PathPlannerCore({'enable_shortcut': True}).make_plan(start, goal, grid)
    assert raw.success and pruned.success and len(raw.path) > len(pruned.path) == 2
    np.testing.assert_array_equal(pruned.path, [start, goal])
    assert pruned.expanded == raw.expanded
    assert section(resolve_parameters({}), 'path_planner.')['enable_shortcut'] is True
    print(f'Installed A*: {len(raw.path)} -> {len(pruned.path)} nodes; '
          f'expanded={pruned.expanded} unchanged; navigation services importable: '
          f'{NavigateTo.__name__}, {NavigateRoute.__name__}')
    print('PASS: independent install, declared dependencies and shortcut behavior')


if __name__ == '__main__':
    main()
