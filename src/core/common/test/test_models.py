import math
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
