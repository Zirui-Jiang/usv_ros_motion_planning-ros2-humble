"""Hybrid A* configuration must reach the runtime with validated units/types."""
import pytest
from pathlib import Path

from usv_navigation.configuration import resolve_parameters, section, validate_update


def test_hybrid_parameters_are_accepted_and_forwarded_as_flat_planner_keys():
    values = resolve_parameters({
        'planner_name': 'hybrid_astar',
        'path_planner.minimum_turning_radius': 6,
        'path_planner.angle_bins': 96,
        'path_planner.heuristic_weight': 1.4,
        'path_planner.motion_step': .8,
        'path_planner.collision_step': .1,
        'path_planner.analytic_expansion_max_length': 24.,
        'path_planner.non_straight_penalty': 1.1,
        'path_planner.change_penalty': .3,
        'path_planner.cost_penalty': .2,
        'path_planner.footprint_padding': .25,
        'path_planner.turning_radius_multipliers': [1, 2],
    })
    planner = section(values, 'path_planner.')
    assert values['planner_name'] == 'hybrid_astar'
    assert planner['minimum_turning_radius'] == 6.
    assert type(planner['minimum_turning_radius']) is float
    assert planner['angle_bins'] == 96
    assert planner['heuristic_weight'] == 1.4
    assert planner['footprint_padding'] == .25
    assert planner['turning_radius_multipliers'] == [1., 2.]
    with pytest.raises(ValueError, match='Restart'):
        validate_update(values, {'path_planner.minimum_turning_radius': 5.})


@pytest.mark.parametrize('name, value', [
    ('minimum_turning_radius', 0.), ('minimum_turning_radius', float('nan')),
    ('motion_step', -1.), ('collision_step', 0.),
    ('analytic_expansion_max_length', 0.),
    ('angle_bins', 7), ('angle_bins', 72.), ('angle_bins', True),
    ('non_straight_penalty', .99), ('change_penalty', -.1),
    ('cost_penalty', -.1), ('footprint_padding', -.1),
    ('default_tolerance', -.1),
    ('heuristic_weight', .99), ('heuristic_weight', float('inf')),
    ('turning_radius_multipliers', []), ('turning_radius_multipliers', [0.5]),
])
def test_hybrid_invalid_values_are_rejected(name, value):
    with pytest.raises(ValueError):
        resolve_parameters({'planner_name': 'hybrid_astar', 'path_planner.'+name: value})


@pytest.mark.parametrize('name', ['minimum_turning_radius', 'turning_radius_multipliers', 'angle_bins', 'motion_step',
    'collision_step', 'analytic_expansion_max_length', 'non_straight_penalty',
    'change_penalty', 'cost_penalty', 'footprint_padding'])
def test_hybrid_defaults_can_construct_a_valid_startup_configuration(name):
    values = resolve_parameters({'planner_name': 'hybrid_astar'})
    assert 'path_planner.'+name in values


def test_shared_launch_accepts_hybrid_override(monkeypatch):
    pytest.importorskip('launch')
    pytest.importorskip('ament_index_python')
    import ament_index_python.packages
    from launch import LaunchContext
    from usv_navigation.launch_support import navigation_arguments

    monkeypatch.setattr(ament_index_python.packages, 'get_package_share_directory',
                        lambda package: '/unused/' + package)
    context = LaunchContext()
    context.launch_configurations['planner_name'] = 'hybrid_astar'
    for argument in navigation_arguments():
        argument.execute(context)
    assert context.launch_configurations['planner_name'] == 'hybrid_astar'


def test_common_planner_yaml_remains_usable_by_legacy_mission_app():
    from usv_mission.configuration import flatten, load_yaml, resolve_parameters as mission_parameters
    root = Path(__file__).resolve().parents[3]
    path = root / 'src/core/shared/usv_navigation/config/path_planner/common_planner_params.yaml'
    values = mission_parameters(flatten(load_yaml(path)))
    assert values['planner_name'] == 'astar'


def test_dedicated_hybrid_yaml_resolves_with_runtime_defaults():
    from usv_navigation.configuration import flatten, load_yaml
    root = Path(__file__).resolve().parents[3]
    path = root / 'src/core/shared/usv_navigation/config/path_planner/hybrid_astar_planner_params.yaml'
    assert path.is_file(), 'Hybrid configuration example is missing'
    values = resolve_parameters(flatten(load_yaml(path)))
    assert values['planner_name'] == 'hybrid_astar'
    assert values['path_planner.minimum_turning_radius'] == 4.


def test_main_launch_passes_hybrid_override_to_decision_node(monkeypatch):
    pytest.importorskip('launch_ros')
    from test_launch_modules import _launch_graph, _nodes, _parameters
    context, actions = _launch_graph(monkeypatch, 'main.launch.py',
                                     {'planner_name': 'hybrid_astar', 'headless': 'true'})
    node = _nodes(context, actions)[('usv_decision', 'decision_node')]
    values = _parameters(context, node)
    assert values['planner_name'] == 'hybrid_astar'
    assert values['path_planner.minimum_turning_radius'] == 4.
    assert values['path_planner.angle_bins'] == 72
