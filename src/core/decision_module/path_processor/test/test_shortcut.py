"""Obstacle-aware shortcuts keep endpoints and the caller's safety policy."""

import numpy as np
import pytest

from path_processor import ShortcutPathProcessor, shortcut_path as shortcut
from common.models import GridSnapshot
from path_planner.path_planner_core import path_is_clear_with_inflation_escape


def checker(data, threshold=127, **options):
    grid = GridSnapshot(np.asarray(data, dtype=np.uint8), 1.)
    return lambda path: path_is_clear_with_inflation_escape(grid, path, threshold, **options)


def test_open_path_connects_endpoints_without_mutating_input():
    path = np.array([[.5, .5, .3], [1.5, 1.5, 0.], [2.5, 1.5, 0.], [4.5, 3.5, -.6]])
    original = path.copy()
    result = shortcut(path, checker(np.zeros((6, 6))))
    np.testing.assert_array_equal(result, original[[0, -1]])
    assert not np.shares_memory(result, path)
    np.testing.assert_array_equal(path, original)


def test_processor_class_uses_the_same_collision_aware_interface():
    result = ShortcutPathProcessor(checker(np.zeros((4, 4)))).process([[.5, .5], [1.5, .5], [2.5, 2.5]])
    np.testing.assert_array_equal(result, [[.5, .5, 0.], [2.5, 2.5, 0.]])


@pytest.mark.parametrize('path', [[], [[.5, .5, .2]], [[.5, .5, .2], [1.5, .5, -.4]]])
def test_empty_and_short_paths_keep_owned_three_column_arrays(path):
    result = shortcut(path, checker(np.zeros((4, 4))))
    np.testing.assert_array_equal(result, np.asarray(path).reshape(-1, 3))


@pytest.mark.parametrize('obstacle', [127, 254, 255])
def test_wall_detour_retains_necessary_bends(obstacle):
    data = np.zeros((7, 7), np.uint8)
    data[:5, 3] = obstacle
    path = np.array([[.5, .5, .1], [.5, 5.5, 0.], [6.5, 5.5, 0.], [6.5, .5, -.3]])
    result = shortcut(path, checker(data))
    np.testing.assert_array_equal(result, path)


def test_soft_start_can_shortcut_a_strictly_descending_escape():
    path = np.array([[i+.5, .5, 0.] for i in range(5)])
    result = shortcut(path, checker([[120, 100, 75, 50, 0]], 63.5))
    np.testing.assert_array_equal(result, path[[0, -1]])


def test_bent_escape_retains_original_edges_whose_end_is_still_inflated():
    data = np.full((3, 4), 253, np.uint8)
    data[0, :3], data[1, 2], data[2, 2:] = [120, 110, 100], 80, [0, 0]
    path = np.array([[.5, .5, 0.], [1.5, .5, 0.], [2.5, .5, 0.],
                     [2.5, 1.5, 0.], [2.5, 2.5, 0.], [3.5, 2.5, 0.]])
    valid = checker(data, 63.5, allow_corner_cutting=False)
    assert valid(path) and not valid(path[:2])
    result = shortcut(path, valid)
    assert valid(result)
    np.testing.assert_array_equal(result[[0, -1]], path[[0, -1]])
    assert len(result) > 2


def test_short_cell_intersections_cannot_be_cut_away():
    path = [[.9, .9, 0.], [.9, 1.1, 0.], [1.2, 1.1, 0.]]
    result = shortcut(path, checker([[0, 127], [0, 0]]))
    np.testing.assert_array_equal(result, path)


def test_corner_policy_is_not_relaxed_by_shortcuts():
    data = np.zeros((3, 3), np.uint8)
    data[0, 1] = 254
    path = [[.5, .5, 0.], [.5, 1.5, 0.], [1.5, 1.5, 0.]]
    np.testing.assert_array_equal(shortcut(path, checker(data, allow_corner_cutting=False)), path)
    np.testing.assert_array_equal(shortcut(path, checker(data, allow_corner_cutting=True)),
                                  np.asarray(path)[[0, -1]])


def test_complete_candidate_is_rechecked_before_returning_it():
    path = np.array([[0., 0., 0.], [1., 0., 0.], [2., 1., 0.], [3., 0., 0.]])
    # Stateful fixture simulates the caller rejecting the final aggregate check.
    calls = 0
    def valid(candidate):
        nonlocal calls
        calls += 1
        return calls < 3
    result = shortcut(path, valid)
    np.testing.assert_array_equal(result, path)


@pytest.mark.parametrize('path', [[[0, float('nan')]], [[0]], [0, 1, 2]])
def test_invalid_path_fails_instead_of_producing_a_reference(path):
    with pytest.raises(ValueError):
        shortcut(path, lambda candidate: True)


def test_unvalidated_original_path_is_rejected():
    with pytest.raises(ValueError, match='path'):
        shortcut([[.5, .5], [2.5, .5]], checker([[0, 254, 0]]))


@pytest.mark.parametrize('deadline', [float('nan'), float('inf'), True])
def test_invalid_deadline_is_rejected(deadline):
    with pytest.raises(ValueError, match='deadline'):
        shortcut([[0, 0]], lambda candidate: True, deadline=deadline)


def test_expired_budget_cannot_return_even_an_unchanged_path(monkeypatch):
    import time
    monkeypatch.setattr(time, 'perf_counter', lambda: 10.)
    with pytest.raises(TimeoutError):
        shortcut([[0, 0]], lambda candidate: True, deadline=10.)


def test_time_spent_inside_collision_callback_counts_against_budget(monkeypatch):
    import time
    clock = [0.]
    monkeypatch.setattr(time, 'perf_counter', lambda: clock[0])
    def slow_check(candidate):
        clock[0] = 2.
        return True
    with pytest.raises(TimeoutError):
        shortcut([[0, 0], [1, 0], [2, 0]], slow_check, deadline=1.)


def test_callback_is_required():
    with pytest.raises(ValueError, match='callable'):
        shortcut([[0, 0]], None)
