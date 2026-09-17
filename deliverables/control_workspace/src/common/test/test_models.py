import math
import pickle
import numpy as np
import pytest
from common.models import GridSnapshot, USVModel, VesselState
from common.math_helper import path_is_clear, wrap_angle


def test_grid_is_snapshot_and_two_dimensional_bounds():
    data = np.zeros((2, 3), dtype=np.uint8)
    grid = GridSnapshot(data, .5, -1, 2)
    data[:] = 254
    assert grid.data[0, 0] == 0
    assert grid.world_to_map(-1.01, 2.1) is None
    assert grid.world_to_map(.5, 2.1) is None
    assert grid.world_to_map(-.75, 2.25) == (0, 0)
    assert grid.map_to_world(0, 0) == (-.75, 2.25)
    with pytest.raises(ValueError):
        grid.data[0, 0] = 4


def test_frame_velocity_and_angles():
    state = VesselState(psi=math.pi/2, u=2, v=1)
    assert np.allclose(state.world_velocity, [-1, 2])
    assert abs(wrap_angle(3*math.pi) + math.pi) < 1e-12


def test_grid_remains_read_only_across_planning_process_serialization():
    grid = GridSnapshot(np.array([[0, 75, 254, 255]], dtype=np.uint8), .5, -2, 3, 17.5, 'test/map')
    restored = pickle.loads(pickle.dumps(grid))
    np.testing.assert_array_equal(restored.data, grid.data)
    assert restored.data is not grid.data and not restored.data.flags.writeable
    assert (restored.resolution, restored.origin_x, restored.origin_y, restored.stamp, restored.frame_id) == (.5, -2, 3, 17.5, 'test/map')
    with pytest.raises(ValueError):
        restored.data[0, 2] = 0  # A worker must not accidentally erase an obstacle.


def test_segment_collision_not_just_waypoints():
    data = np.zeros((3, 5), dtype=np.uint8)
    data[1, 2] = 254
    grid = GridSnapshot(data, 1)
    assert not path_is_clear(grid, [[.5, 1.5, 0], [4.5, 1.5, 0]])
    assert path_is_clear(grid, [[.5, .5, 0], [4.5, .5, 0]])


def test_invalid_metadata_and_model():
    with pytest.raises(ValueError):
        GridSnapshot(np.zeros((3, 3)), 1)
    with pytest.raises(ValueError):
        USVModel.from_mapping({"mass": 0})
    with pytest.raises(ValueError):
        USVModel.from_mapping({"mass": float("nan")})
