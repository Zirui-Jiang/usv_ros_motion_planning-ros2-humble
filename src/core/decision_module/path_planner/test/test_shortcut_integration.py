"""A* post-processing is opt-in at the core and does not change search."""

import math
import json
import os
from pathlib import Path
import subprocess
import sys
import time
import numpy as np
import pytest

from common.models import AlgorithmStatus, GridSnapshot
from path_planner import PathPlannerCore, create_planner
from path_planner.path_planner_core import path_is_clear_with_inflation_escape


def scene(data=None):
    return GridSnapshot(np.zeros((8, 8), np.uint8) if data is None else np.asarray(data, np.uint8), 1.)


def test_astar_shortcut_is_applied_after_search_without_changing_expansions():
    start, goal, grid = (.5, .5, .3), (5.5, 3.5, -.6), scene()
    original = create_planner().plan(start, goal, grid)
    result = PathPlannerCore({'enable_shortcut': True}).make_plan(start, goal, grid)
    assert result.success
    np.testing.assert_array_equal(result.path, [start, goal])
    assert result.expanded == original.expanded
    assert np.linalg.norm(np.diff(result.path[:, :2], axis=0), axis=1).sum() == pytest.approx(math.sqrt(34))


@pytest.mark.parametrize('name', ['astar', 'dijkstra', 'gbfs'])
def test_disabled_shortcut_retains_original_search_output(name):
    grid, start, goal = scene(), (.5, .5, .3), (5.5, 3.5, -.6)
    original = create_planner({'planner_name': name}).plan(start, goal, grid)
    result = PathPlannerCore({'planner_name': name, 'enable_shortcut': False}).make_plan(start, goal, grid)
    np.testing.assert_array_equal(result.path, original.path)


@pytest.mark.parametrize('name', ['dijkstra', 'gbfs'])
def test_switch_does_not_change_other_grid_algorithms(name):
    start, goal, grid = (.5, .5, 0.), (5.5, 3.5, 0.), scene()
    original = PathPlannerCore({'planner_name': name}).make_plan(start, goal, grid)
    result = PathPlannerCore({'planner_name': name, 'enable_shortcut': True}).make_plan(start, goal, grid)
    np.testing.assert_array_equal(result.path, original.path)


def test_resampling_follows_shortcut_line_instead_of_the_grid_staircase():
    start, goal = np.array([.5, .5, .3]), np.array([5.5, 3.5, -.6])
    result = PathPlannerCore({'enable_shortcut': True, 'enable_resample': True,
                              'resample_spacing': .25}).make_plan(start, goal, scene())
    assert result.success and len(result.path) > 2
    offsets = result.path[:, :2]-start[:2]
    np.testing.assert_allclose(offsets[:, 0]*3-offsets[:, 1]*5, 0., atol=1e-12)
    np.testing.assert_array_equal(result.path[[0, -1]], [start, goal])


def test_unsafe_resampling_falls_back_to_the_valid_shortcut_not_a_wall_crossing():
    data = np.zeros((7, 7), np.uint8)
    data[:5, 3] = 254
    grid, start, goal = scene(data), (.5, .5, 0.), (6.5, .5, 0.)
    simplified = PathPlannerCore({'enable_shortcut': True}).make_plan(start, goal, grid)
    result = PathPlannerCore({'enable_shortcut': True, 'enable_resample': True,
                              'resample_spacing': 100.}).make_plan(start, goal, grid)
    assert result.success and len(result.path) > 2
    np.testing.assert_array_equal(result.path, simplified.path)
    assert path_is_clear_with_inflation_escape(grid, result.path, 127)


def test_deferred_postprocessing_keeps_grid_geometry_for_exact_anchor_connection():
    core = PathPlannerCore({'enable_shortcut': True, 'enable_resample': True})
    grid, start, goal = scene(), (.17, .31, .2), (5.72, 3.69, -.4)
    original = create_planner().plan(start, goal, grid)
    result = core.make_plan(start, goal, grid, postprocess=False)
    np.testing.assert_array_equal(result.path, original.path)


@pytest.mark.parametrize('value', ['false', 1, None])
def test_shortcut_switch_requires_a_boolean(value):
    with pytest.raises(ValueError, match='enable_shortcut'):
        PathPlannerCore({'enable_shortcut': value})


def test_search_time_is_not_reset_when_shortcut_processing_starts(monkeypatch):
    core = PathPlannerCore({'enable_shortcut': True, 'timeout': 1.})
    plan = core.planner.plan
    clock = [0.]
    monkeypatch.setattr(time, 'perf_counter', lambda: clock[0])
    def search_using_budget(*args):
        result = plan(*args)
        clock[0] = 1.1
        return result
    monkeypatch.setattr(core.planner, 'plan', search_using_budget)
    result = core.make_plan((.5, .5, 0.), (5.5, 3.5, 0.), scene())
    assert result.status == AlgorithmStatus.TIMEOUT
    assert result.path.shape == (0, 3)


def test_validator_interrupts_a_long_segment_when_the_shared_deadline_expires(monkeypatch):
    ticks = iter([0., 0., 0., 2.])
    monkeypatch.setattr(time, 'perf_counter', lambda: next(ticks, 2.))
    with pytest.raises(TimeoutError):
        path_is_clear_with_inflation_escape(scene(), [[.5, .5], [7.5, 7.5]], deadline=1.)


def test_hybrid_curve_is_unchanged_even_with_shortcuts_enabled():
    grid = GridSnapshot(np.zeros((100, 100), np.uint8), .25, -5., -5.)
    config = dict(planner_name='hybrid_astar', minimum_turning_radius=2., motion_step=.75,
                  collision_step=.2, analytic_expansion_max_length=16.,
                  footprint_length=.4, footprint_width=.3, timeout=4., enable_resample=True)
    start, goal = (0., 0., 0.), (2., 2., math.pi/2)
    original = PathPlannerCore(config).make_plan(start, goal, grid)
    result = PathPlannerCore(dict(config, enable_shortcut=True)).make_plan(start, goal, grid)
    assert original.success and result.success and len(result.path) > 10
    np.testing.assert_array_equal(result.path, original.path)


@pytest.mark.parametrize('relative', [
    'src/core/decision_module/path_planner/examples/hybrid_astar_demo.py',
    'scripts/validation/replay_hybrid_snapshot.py',
    'scripts/validation/replay_hybrid_route_reuse.py',
])
def test_source_entrypoints_resolve_processor_dependency_without_an_overlay(relative, tmp_path):
    root = Path(__file__).resolve().parents[5]
    env = {key: value for key, value in os.environ.items() if key != 'PYTHONPATH'}
    result = subprocess.run([sys.executable, '-I', '-B', str(root/relative), '--help'], cwd=tmp_path,
                            env=env, capture_output=True, text=True, timeout=20)
    assert result.returncode == 0, result.stderr
    assert 'usage:' in result.stdout.lower()


@pytest.mark.parametrize('suffix', ['c0a7f065', '78bb0382'])
@pytest.mark.parametrize('resampling', [False, True])
def test_captured_soft_start_maps_remain_clear_with_shortcuts_enabled(suffix, resampling):
    directory = Path(__file__).parent/'fixtures/factor025_soft_start'
    state = json.loads((directory/f'state_input_{suffix}.json').read_text())
    with np.load(directory/f'costmap_input_{suffix}.npz') as saved:
        grid = GridSnapshot(saved['data'], float(saved['resolution']),
                            float(saved['origin_x']), float(saved['origin_y']))
    raw = PathPlannerCore({'obstacle_factor': .25, 'timeout': 5.}).make_plan(state['pose'], state['goal'], grid)
    result = PathPlannerCore({'obstacle_factor': .25, 'timeout': 5., 'enable_shortcut': True,
                              'enable_resample': resampling}).make_plan(state['pose'], state['goal'], grid)
    assert raw.success and result.success, result.message
    assert path_is_clear_with_inflation_escape(grid, result.path, 63.5)
    np.testing.assert_array_equal(result.path[[0, -1]], raw.path[[0, -1]])
    assert result.expanded == raw.expanded
    assert np.linalg.norm(np.diff(result.path[:, :2], axis=0), axis=1).sum() <= (
        np.linalg.norm(np.diff(raw.path[:, :2], axis=0), axis=1).sum()+1e-9)
