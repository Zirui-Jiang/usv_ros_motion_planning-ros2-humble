"""Only measured, ordered, plausible vessel motion confirms mandatory points."""
import pytest
from common.models import VesselState
from usv_decision.route_progress import RouteProgress


def state(x, y=0., stamp=1.):
    return VesselState(x=x, y=y, stamp=stamp)


def route(*xy):
    return RouteProgress(tuple((x, y, 0., 1.) for x, y in xy), max_speed=4.)


def test_order_and_final_stop_are_separate_from_intermediate_passage():
    progress = route((5., 0.), (10., 0.), (0., 0.))
    assert not progress.update(state(0.))  # Closed route does not finish at start.
    assert progress.index == 0
    assert progress.update(state(4., stamp=2.))
    assert progress.index == 1 and not progress.terminal
    assert progress.update(state(9., stamp=3.))
    assert progress.terminal and progress.passed_count == 2
    assert not progress.update(state(0., stamp=6.))
    assert progress.passed_count == 2  # Final needs low-speed controller ACK.
    progress.finish()
    assert progress.passed_count == 3 and progress.index == 3


def test_only_current_point_can_pass_and_repeated_positions_stay_ordered():
    progress = route((0., 0.), (10., 0.), (0., 0.), (15., 0.))
    assert progress.update(state(0.))
    assert progress.index == 1
    assert not progress.update(state(0., stamp=2.))
    assert progress.update(state(10., stamp=5.))
    assert progress.index == 2
    assert progress.update(state(0., stamp=8.))
    assert progress.index == 3


def test_short_measured_segment_crossing_confirms_even_if_samples_outside_disk():
    progress = route((0., 0.), (20., 0.))
    progress.update(state(-2.))
    assert progress.update(state(2., stamp=2.))
    assert progress.index == 1


def test_segment_crossing_cannot_confirm_points_in_reverse_order():
    progress = route((2., 0.), (-2., 0.), (20., 0.))
    progress.update(state(-4.))
    # A generous speed limit makes this segment plausible. P2 was passed before P1.
    progress.max_speed = 10.
    assert progress.update(state(4., stamp=2.))
    assert progress.index == 1


def test_teleport_and_long_observation_gap_cannot_fabricate_passages():
    progress = route((0., 0.), (20., 0.))
    progress.update(state(-10.))
    assert not progress.update(state(10., stamp=1.1))
    assert progress.index == 0
    progress.update(state(-10., stamp=10.))
    assert not progress.update(state(10., stamp=20.))
    assert progress.index == 0


def test_duplicate_backward_nonfinite_and_wrong_frame_samples_do_not_advance():
    progress = route((0., 0.), (20., 0.))
    progress.update(state(-2., stamp=2.))
    assert not progress.update(state(0., stamp=2.))
    assert not progress.update(state(0., stamp=1.))
    assert not progress.update(state(float('nan'), stamp=3.))
    assert not progress.update(VesselState(x=0., stamp=3., frame_id='other'))
    assert progress.index == 0


def test_consecutive_duplicate_points_are_preserved_and_confirmed_in_order():
    progress = route((0., 0.), (0., 0.), (20., 0.))
    assert len(progress.points) == 3
    assert progress.update(state(0.))
    assert progress.index == 2


def test_final_cannot_be_marked_complete_while_previous_points_unpassed():
    progress = route((0., 0.), (20., 0.))
    with pytest.raises(ValueError):
        progress.finish()


@pytest.mark.parametrize('points,radius', [([], 1.5), ([(0., 0., 0., 0.)], 1.5),
    ([(0., float('nan'), 0., 1.)], 1.5), ([(0., 0., 0., 1.)], 0.),
    ([(0., 0., 0., 1.)] * 256, 1.5)])
def test_invalid_route_is_rejected(points, radius):
    with pytest.raises(ValueError):
        RouteProgress(points, radius)
