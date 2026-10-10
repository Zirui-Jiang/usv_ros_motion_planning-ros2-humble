# 验证状态坐标约定、地图独立只读快照、进程序列化与路径线段检查。
import math
import pickle
import numpy as np
import pytest
from common.models import GridSnapshot, USVModel, VesselState
from common.math_helper import path_is_clear, wrap_angle


# 地图复制源数组且保持只读，坐标转换严格按二维边界。
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


# 船体系速度旋转及角度规整符合约定。
def test_frame_velocity_and_angles():
    state = VesselState(psi=math.pi/2, u=2, v=1)
    assert np.allclose(state.world_velocity, [-1, 2])
    assert abs(wrap_angle(3*math.pi) + math.pi) < 1e-12


# 跨规划进程序列化后地图仍为独立只读数组。
def test_grid_remains_read_only_across_planning_process_serialization():
    grid = GridSnapshot(np.array([[0, 75, 254, 255]], dtype=np.uint8), .5, -2, 3, 17.5, 'test/map')
    restored = pickle.loads(pickle.dumps(grid))
    np.testing.assert_array_equal(restored.data, grid.data)
    assert restored.data is not grid.data and not restored.data.flags.writeable
    assert (restored.resolution, restored.origin_x, restored.origin_y, restored.stamp, restored.frame_id) == (.5, -2, 3, 17.5, 'test/map')
    with pytest.raises(ValueError):
        restored.data[0, 2] = 0  # A worker must not accidentally erase an obstacle.


# 路径中间线段穿障不能因路点安全而漏检。
def test_segment_collision_not_just_waypoints():
    data = np.zeros((3, 5), dtype=np.uint8)
    data[1, 2] = 254
    grid = GridSnapshot(data, 1)
    assert not path_is_clear(grid, [[.5, 1.5, 0], [4.5, 1.5, 0]])
    assert path_is_clear(grid, [[.5, .5, 0], [4.5, .5, 0]])


# 非法地图数据类型和船舶质量参数必须拒绝。
def test_invalid_metadata_and_model():
    with pytest.raises(ValueError):
        GridSnapshot(np.zeros((3, 3)), 1)
    with pytest.raises(ValueError):
        USVModel.from_mapping({"mass": 0})
    with pytest.raises(ValueError):
        USVModel.from_mapping({"mass": float("nan")})


# 船型与推进形式：Otter 只能差动；WAM-V 可差动或带独立侧推器，侧推最大力须与推进形式一致。
def test_supported_vessels_and_lateral_thruster_consistency():
    assert USVModel.from_mapping({}).max_lateral_force == 0.
    wamv = USVModel.from_mapping({"name": "wamv", "type": "Omni", "max_lateral_force": 250.})
    assert (wamv.name, wamv.type, wamv.max_lateral_force) == ("wamv", "Omni", 250.)
    assert USVModel.from_mapping({"name": "wamv"}).type == "Differential"
    # 两台全回转桨同样需要横向力归一化基准，并带回转轴的纵向位置。
    azimuth = USVModel.from_mapping({"name": "wamv", "type": "Azimuth", "max_lateral_force": 200., "thruster_x": -2.374})
    assert (azimuth.type, azimuth.max_lateral_force, azimuth.thruster_x) == ("Azimuth", 200., -2.374)
    assert USVModel.from_mapping({}).thruster_x == 0.
    for values in ({"name": "wamv", "type": "Azimuth"}, {"name": "otter", "type": "Azimuth", "max_lateral_force": 10.},
                   {"name": "wamv", "type": "Azimuth", "max_lateral_force": 200., "thruster_x": float("nan")}):
        with pytest.raises(ValueError):
            USVModel.from_mapping(values)
    for values in ({"name": "otter", "type": "Omni", "max_lateral_force": 10.},
                   {"name": "other"}, {"type": "Ackermann"},
                   {"name": "wamv", "type": "Omni"},
                   {"name": "wamv", "type": "Omni", "max_lateral_force": -1.},
                   {"name": "wamv", "max_lateral_force": 250.},
                   {"max_lateral_force": float("inf")}):
        with pytest.raises(ValueError):
            USVModel.from_mapping(values)
