# 用已捕获的 raw 地图与轨迹检查完整船体：允许合法软膨胀区情况，仍拒绝实体障碍切角。
"""Regression geometry from recorded raw-map/trajectory captures, not new runs."""
import json
from pathlib import Path
import numpy as np
from common.collision import footprint_path_is_clear
from common.math_helper import path_is_clear
from common.models import GridSnapshot

DIAGNOSTICS = Path(__file__).with_name('diagnostics')


def read_grid(directory):
    paths = list(directory.glob('costmap_input_*.npz'))
    assert len(paths) == 1
    with np.load(paths[0], allow_pickle=False) as capture:
        assert str(capture['cost_kind']) == 'raw'
        return GridSnapshot(capture['data'], float(capture['resolution']),
                            float(capture['origin_x']), float(capture['origin_y']),
                            float(capture['stamp']), str(capture['frame_id']))


def test_soft_buffer_rejections_are_clear_for_full_footprint_on_captured_map():
    directory = DIAGNOSTICS / 'switch1_failure_001'
    grid = read_grid(directory)
    report = json.loads((directory / 'offline_combinations_immediate_map.json').read_text())
    for case in report['comparisons']:
        poses = np.asarray(case['trajectory'])[:, [2, 3, 4]]
        assert not path_is_clear(grid, poses, 127.)
        assert footprint_path_is_clear(grid, poses, 2.2, 1.2)


def test_physical_corner_cut_is_still_rejected_on_captured_map():
    directory = DIAGNOSTICS / 'obstacle_raw_002'
    grid = read_grid(directory)
    report = json.loads((directory / 'los_replay.json').read_text())
    verdicts = {case['lookahead_distance_m']: footprint_path_is_clear(
        grid, np.asarray(case['trajectory'])[:, [2, 3, 4]], 2.2, 1.2)
        for case in report['comparisons']}
    assert verdicts == {5.0: False, 1.0: True}
