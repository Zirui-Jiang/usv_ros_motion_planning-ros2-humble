"""The shortcut switch reaches planning while legacy configs still parse it."""

import pytest
from usv_navigation.configuration import resolve_parameters, section, validate_update
from usv_mission.configuration import resolve_parameters as mission_parameters


@pytest.mark.parametrize('enabled', [False, True])
def test_shortcut_boolean_reaches_the_planner_section(enabled):
    values = resolve_parameters({'path_planner.enable_shortcut': enabled})
    assert section(values, 'path_planner.')['enable_shortcut'] is enabled


def test_normal_navigation_enables_astar_shortcuts_by_default():
    assert section(resolve_parameters({}), 'path_planner.')['enable_shortcut'] is True


@pytest.mark.parametrize('value', ['true', 0, 1])
def test_nonboolean_shortcut_switch_is_rejected(value):
    with pytest.raises(ValueError):
        resolve_parameters({'path_planner.enable_shortcut': value})


def test_legacy_mission_accepts_switch_without_changing_its_default():
    assert mission_parameters({})['path_planner.enable_shortcut'] is False
    assert mission_parameters({'path_planner.enable_shortcut': True})['path_planner.enable_shortcut'] is True


def test_changing_shortcut_policy_requires_restart():
    values = resolve_parameters({'path_planner.enable_shortcut': True})
    with pytest.raises(ValueError, match='Restart required'):
        validate_update(values, {'path_planner.enable_shortcut': False})
