"""Hybrid A* 的船体、精确圆弧扫掠和原始地图代价策略。"""

import math

import numpy as np
import pytest

from common.models import GridSnapshot


def checker(data=None, *, resolution=1.0, origin_x=0.0, origin_y=0.0, **config):
    from path_planner.graph_planner.hybrid_collision import HybridCollisionChecker

    values = np.zeros((12, 12), dtype=np.uint8) if data is None else data
    scene = GridSnapshot(values, resolution, origin_x, origin_y)
    return HybridCollisionChecker(scene, **config)


def test_bow_collision_is_detected_even_when_centre_cell_is_clear():
    data = np.zeros((12, 12), dtype=np.uint8)
    data[4, 5] = 254
    collision = checker(data, footprint_width=.6)
    assert not collision.pose_is_clear((4.5, 4.5, 0.0))


def test_rotated_long_hull_changes_clearance():
    data = np.zeros((12, 12), dtype=np.uint8)
    data[4, 5] = 254
    collision = checker(data, footprint_width=.6)
    assert collision.pose_is_clear((4.5, 4.5, math.pi / 2))
    assert not collision.pose_is_clear((4.5, 4.5, 0.0))


def test_soft_band_is_checked_at_centre_without_inflating_the_hull_again():
    data = np.zeros((12, 12), dtype=np.uint8)
    data[4, 5] = 200
    collision = checker(data, footprint_width=.6)
    assert collision.pose_is_clear((4.5, 4.5, 0.0))
    assert not collision.pose_is_clear((5.5, 4.5, 0.0))
    assert checker(data, factor=1.0).pose_is_clear((5.5, 4.5, 0.0))


@pytest.mark.parametrize("unknown_is_obstacle", [True, False])
@pytest.mark.parametrize("pose", [(4.5, 4.5, 0.0), (5.5, 4.5, 0.0)])
def test_unknown_policy_applies_to_both_hull_and_centre(unknown_is_obstacle, pose):
    data = np.zeros((12, 12), dtype=np.uint8)
    data[4, 5] = 255
    collision = checker(data, footprint_width=.6, unknown_is_obstacle=unknown_is_obstacle)
    assert collision.pose_is_clear(pose) is not unknown_is_obstacle


def test_map_outline_blocks_the_hull_even_with_centre_interior():
    assert checker().pose_is_clear((1.5, 4.5, 0.0))
    assert not checker(outline_map=True).pose_is_clear((1.5, 4.5, 0.0))
    assert checker(outline_map=True).pose_is_clear((2.5, 4.5, 0.0))


@pytest.mark.parametrize("pose", [(.9, 4.5, 0.0), (11.1, 4.5, 0.0),
                                 (4.5, .9, math.pi / 2), (4.5, 11.1, math.pi / 2)])
def test_hull_extending_outside_map_is_blocked(pose):
    assert not checker().pose_is_clear(pose)


def test_checker_keeps_grid_snapshot_read_only_and_unchanged():
    from path_planner.graph_planner.hybrid_collision import HybridCollisionChecker

    scene = GridSnapshot(np.zeros((12, 12), dtype=np.uint8), 1.0)
    original = scene.data.copy()
    collision = HybridCollisionChecker(scene, outline_map=True)
    assert collision.motion_is_clear((3.5, 4.5, 0.0), 0.0, 2.0)
    np.testing.assert_array_equal(scene.data, original)
    assert not scene.data.flags.writeable


def test_nonzero_origin_and_fractional_resolution_preserve_cell_geometry():
    data = np.zeros((40, 40), dtype=np.uint8)
    data[20, 20] = 254  # World square x=[0,.25], y=[15,15.25].
    collision = checker(data, resolution=.25, origin_x=-5.0, origin_y=10.0,
                        footprint_length=.6, footprint_width=.1)
    assert not collision.pose_is_clear((-.2, 15.125, 0.0))
    assert collision.pose_is_clear((-.2, 15.125, math.pi / 2))
    assert collision.pose_is_clear((-.7, 15.125, 0.0))


def test_configured_padding_increases_the_physical_hull():
    data = np.zeros((12, 12), dtype=np.uint8)
    data[4, 6] = 254
    config = dict(footprint_length=2.0, footprint_width=.2)
    assert checker(data, **config).pose_is_clear((4.9, 4.5, 0.0))
    assert not checker(data, footprint_padding=.11, **config).pose_is_clear((4.9, 4.5, 0.0))


def test_straight_motion_checks_obstacle_between_clear_endpoints():
    data = np.zeros((12, 12), dtype=np.uint8)
    data[3, 5] = 254
    collision = checker(data, footprint_length=.4, footprint_width=.2)
    assert collision.pose_is_clear((2.5, 3.5, 0.0))
    assert collision.pose_is_clear((8.5, 3.5, 0.0))
    assert not collision.motion_is_clear((2.5, 3.5, 0.0), 0.0, 6.0)


@pytest.mark.parametrize("curvature,start_y,obstacle_row", [(.25, 3.0, 41), (-.25, 10.0, 87)])
def test_curved_motion_checks_obstacle_on_the_arc(curvature, start_y, obstacle_row):
    data = np.zeros((140, 140), dtype=np.uint8)
    data[obstacle_row:obstacle_row + 3, 68:70] = 254
    collision = checker(data, resolution=.1, footprint_length=.2, footprint_width=.1)
    assert collision.pose_is_clear((4.0, start_y, 0.0))
    end_y = 7.0 if curvature > 0 else 6.0
    assert collision.pose_is_clear((8.0, end_y, math.copysign(math.pi / 2, curvature)))
    assert not collision.motion_is_clear((4.0, start_y, 0.0), curvature, 2 * math.pi)


def test_arc_is_not_replaced_by_a_straight_chord_for_collision():
    data = np.zeros((140, 140), dtype=np.uint8)
    data[49:51, 59:61] = 254  # The chord hits (6,5), the quarter-circle does not.
    collision = checker(data, resolution=.1, footprint_length=.2, footprint_width=.1)
    assert collision.motion_is_clear((4.0, 3.0, 0.0), .25, 2 * math.pi)


@pytest.mark.parametrize("cost", [127, 254])
def test_sampling_gap_cannot_skip_a_tiny_cell_corner_visit(cost):
    data = np.zeros((12, 12), dtype=np.uint8)
    data[4, 5] = cost
    collision = checker(data, footprint_length=.005, footprint_width=.005, collision_step=10.0)
    yaw = math.atan2(.2, .3)
    assert collision.pose_is_clear((4.9, 4.9, yaw))
    assert collision.pose_is_clear((5.2, 5.1, yaw))
    assert not collision.motion_is_clear((4.9, 4.9, yaw), 0.0, math.hypot(.3, .2))


def test_zero_distance_still_checks_the_pose():
    data = np.zeros((12, 12), dtype=np.uint8)
    data[4, 5] = 254
    collision = checker(data, footprint_width=.6)
    assert collision.motion_is_clear((4.5, 4.5, math.pi / 2), 0.0, 0.0)
    assert not collision.motion_is_clear((4.5, 4.5, 0.0), 0.0, 0.0)


@pytest.mark.parametrize("distance", [-1.0, math.nan, math.inf, -math.inf])
def test_invalid_motion_distance_is_rejected(distance):
    assert not checker().motion_is_clear((4.5, 4.5, 0.0), 0.0, distance)


@pytest.mark.parametrize("pose", [(math.nan, 4.5, 0.0), (4.5, 4.5, math.inf), (4.5, 4.5)])
def test_nonfinite_or_wrong_sized_pose_is_rejected(pose):
    assert not checker().pose_is_clear(pose)


def test_expired_motion_deadline_raises_timeout():
    with pytest.raises(TimeoutError):
        checker().motion_is_clear((4.5, 4.5, 0.0), .1, 1.0, deadline=0.0)


def test_deadline_is_rechecked_during_motion(monkeypatch):
    from path_planner.graph_planner import hybrid_collision

    collision = checker(collision_step=.01)
    ticks = iter([0.0, 0.0, 0.0, 2.0])
    monkeypatch.setattr(hybrid_collision.time, "perf_counter", lambda: next(ticks, 2.0))
    with pytest.raises(TimeoutError):
        collision.motion_is_clear((4.5, 4.5, 0.0), .1, 1.0, deadline=1.0)


@pytest.mark.parametrize("config", [{"factor": 0.0}, {"factor": 1.1}, {"factor": math.nan},
                                     {"footprint_length": 0.0}, {"footprint_width": -1.0},
                                     {"footprint_padding": -1.0}, {"footprint_padding": math.inf},
                                     {"collision_step": 0.0}])
def test_invalid_configuration_is_rejected_before_search(config):
    with pytest.raises(ValueError):
        checker(**config)


def count_fine_checks(monkeypatch, collision):
    """Count actual centre/hull geometry work without replacing its result."""
    calls = {"centre": 0, "hull": 0}
    centre = collision._centre_is_clear
    hull = collision._hull_is_clear

    def checked_centre(*args):
        calls["centre"] += 1
        return centre(*args)

    def checked_hull(*args):
        calls["hull"] += 1
        return hull(*args)

    monkeypatch.setattr(collision, "_centre_is_clear", checked_centre)
    monkeypatch.setattr(collision, "_hull_is_clear", checked_hull)
    return calls


@pytest.mark.parametrize("curvature,distance", [(0.0, 1.0), (0.0, 40.0),
                                                (.08, 20.0), (-.08, 20.0)])
def test_clear_water_motion_needs_no_per_pose_geometry_checks(monkeypatch, curvature, distance):
    collision = checker(np.zeros((400, 400), dtype=np.uint8), resolution=.25,
                        collision_step=.02)
    calls = count_fine_checks(monkeypatch, collision)

    assert collision.motion_is_clear((30.0, 50.0, .3), curvature, distance)
    assert calls == {"centre": 0, "hull": 0}


def test_fast_clear_keeps_soft_band_beneath_hull_outside_centre_region(monkeypatch):
    data = np.zeros((160, 160), dtype=np.uint8)
    data[55, 52] = 200  # Inside the hull, outside the padded centre trajectory.
    collision = checker(data, resolution=.1)
    calls = count_fine_checks(monkeypatch, collision)

    assert collision.motion_is_clear((5.0, 5.0, 0.0), 0.0, .5)
    assert calls == {"centre": 0, "hull": 0}


@pytest.mark.parametrize("start_y,expected", [(1.0, True), (.5, False)])
def test_broad_region_crossing_map_boundary_falls_back_to_geometry(monkeypatch, start_y, expected):
    collision = checker()
    calls = count_fine_checks(monkeypatch, collision)

    # The enclosing square crosses y=0 even when the actual straight hull fits.
    assert collision.motion_is_clear((2.0, start_y, 0.0), 0.0, 6.0) is expected
    assert calls["centre"] > 0
    assert calls["hull"] > 0


def test_fast_clear_does_not_skip_rotating_bow_between_clear_endpoints():
    data = np.zeros((120, 120), dtype=np.uint8)
    data[63, 67] = 254
    collision = checker(data, resolution=.1, footprint_length=3.0, footprint_width=.1)

    assert collision.pose_is_clear((5.0, 5.0, 0.0))
    assert collision.pose_is_clear((6.0, 6.0, math.pi / 2))
    assert not collision.motion_is_clear((5.0, 5.0, 0.0), 1.0, math.pi / 2)


@pytest.mark.parametrize("unknown_is_obstacle", [True, False])
def test_fast_clear_motion_preserves_unknown_policy(unknown_is_obstacle):
    data = np.zeros((12, 12), dtype=np.uint8)
    data[4, 5] = 255
    collision = checker(data, unknown_is_obstacle=unknown_is_obstacle,
                        footprint_length=.4, footprint_width=.2)
    assert collision.motion_is_clear((4.5, 4.5, 0.0), 0.0, 2.0) is not unknown_is_obstacle


def test_deadline_is_checked_before_region_proof(monkeypatch):
    from path_planner.graph_planner import hybrid_collision

    ticks = iter([0.0, 2.0])
    monkeypatch.setattr(hybrid_collision.time, "perf_counter", lambda: next(ticks, 2.0))
    with pytest.raises(TimeoutError):
        checker().motion_is_clear((4.5, 4.5, 0.0), .1, 1.0, deadline=1.0)


def test_deadline_expires_during_successful_region_proof(monkeypatch):
    from path_planner.graph_planner import hybrid_collision

    collision = checker()
    proof = collision._motion_region_is_clear
    clock = [0.0]

    def delayed_proof(*args):
        # Execute the real successful proof, then model its elapsed work.
        assert proof(*args)
        clock[0] = 2.0
        return True

    monkeypatch.setattr(collision, "_motion_region_is_clear", delayed_proof)
    monkeypatch.setattr(hybrid_collision.time, "perf_counter", lambda: clock[0])
    with pytest.raises(TimeoutError):
        collision.motion_is_clear((4.5, 4.5, 0.0), .1, 1.0, deadline=1.0)


@pytest.mark.parametrize("yaw,curvature,distance", [(0.0, .4, 20.0), (1e100, .1, 2.0)])
def test_large_angles_fall_back_to_fine_geometry(monkeypatch, yaw, curvature, distance):
    collision = checker(np.zeros((64, 64), dtype=np.uint8), resolution=.25,
                        footprint_length=.2, footprint_width=.1)
    calls = count_fine_checks(monkeypatch, collision)
    assert collision.motion_is_clear((8.0, 6.0, yaw), curvature, distance)
    assert calls["centre"] > 0
    assert calls["hull"] > 0


@pytest.mark.parametrize("unknown_is_obstacle", [True, False])
@pytest.mark.parametrize("outline_map", [True, False])
@pytest.mark.parametrize("factor", [.25, .5, 1.0])
def test_fast_clear_matches_fine_reference_on_seeded_scenes(
        monkeypatch, unknown_is_obstacle, outline_map, factor):
    rng = np.random.default_rng(20260916)
    results = set()
    for resolution, origin in [(.25, (-8.0, 3.0)), (.75, (2.0, -12.0))]:
        data = rng.choice([0, 50, 127, 200, 254, 255], size=(64, 72),
                          p=[.9875, .0025, .0025, .0025, .0025, .0025]).astype(np.uint8)
        config = dict(resolution=resolution, origin_x=origin[0], origin_y=origin[1],
                      unknown_is_obstacle=unknown_is_obstacle, outline_map=outline_map,
                      factor=factor, footprint_length=.7, footprint_width=.3,
                      footprint_padding=.05)
        collision = checker(data, **config)
        reference = checker(data, **config)
        # Disable only the optional proof; the unchanged precise loop is the oracle.
        monkeypatch.setattr(reference, "_motion_region_is_clear", lambda *args: False, raising=False)
        for _ in range(30):
            start = (origin[0] + rng.uniform(.1, 71.9) * resolution,
                     origin[1] + rng.uniform(.1, 63.9) * resolution,
                     rng.uniform(-math.pi, math.pi))
            curvature = rng.choice([-.9, -.2, 0.0, .2, .9])
            distance = rng.uniform(.05, 8.0)
            expected = reference.motion_is_clear(start, curvature, distance)
            assert collision.motion_is_clear(start, curvature, distance) is expected
            results.add(expected)
    assert results == {False, True}
