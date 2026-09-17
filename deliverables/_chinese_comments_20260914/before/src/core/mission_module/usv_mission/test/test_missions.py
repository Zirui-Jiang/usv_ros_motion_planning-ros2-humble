from dataclasses import replace
from types import SimpleNamespace
import math
import numpy as np
import pytest

from common.models import AlgorithmStatus, GridSnapshot, VesselState
from usv_mission.configuration import resolve_parameters
from usv_mission.missions import MissionManager, MissionSpec


def grid(now=0., obstacle=None):
    data = np.zeros((400, 400), np.uint8)
    if obstacle is not None:
        x, y = obstacle
        data[int((y+50)/.25), int((x+50)/.25)] = 254
    return GridSnapshot(data, .25, -50., -50., now)


def manager(mode='standby', **kwargs):
    return MissionManager(MissionSpec(mode, -33.7242, 150.68, **kwargs), (0., 0., 0.), 0., 'test', resolve_parameters({}))


@pytest.mark.parametrize('changes', [{'latitude': 91.}, {'longitude': math.nan}, {'speed': 0.},
    {'speed': 2.}, {'position_tolerance': 0.}, {'watch_radius': .1}, {'duration': -1.},
    {'laps': -1}, {'laps': 1.5}, {'clockwise': 'yes'}, {'mode': 'unknown'}, {'orbit_radius': 1.}])
def test_spec_rejects_invalid_parameters(changes):
    spec = replace(MissionSpec('orbit', 0., 0.), **changes)
    with pytest.raises(ValueError):
        spec.validate(resolve_parameters({}))


def test_spec_respects_nonzero_model_minimum_speed():
    cfg = resolve_parameters({'dynamics.velocity_min': .6})
    with pytest.raises(ValueError, match='velocity'):
        MissionSpec('standby', 0., 0., speed=.5).validate(cfg)


def test_standby_requires_low_speed_dwell_and_returns_only_beyond_outer_radius():
    m = manager()
    assert m.tick(VesselState(u=.2), grid(), 0.).kind == 'navigate'
    m.tick(VesselState(stamp=.1), grid(.1), .1)
    assert m.tick(VesselState(stamp=.9), grid(.9), .9).kind == 'navigate'
    assert m.tick(VesselState(stamp=1.2), grid(1.2), 1.2).kind == 'stop'
    revision = m.revision
    assert m.tick(VesselState(x=8., stamp=2.), grid(2.), 2.).kind == 'stop'
    assert m.revision == revision
    returned = m.tick(VesselState(x=10.1, stamp=3.), grid(3.), 3.)
    assert returned.kind == 'navigate' and returned.goal[:2] == (0., 0.)


@pytest.mark.parametrize('clockwise', [True, False])
def test_orbit_counts_signed_progress_across_wrap_and_finishes_at_requested_laps(clockwise):
    m = manager('orbit', clockwise=clockwise, laps=1)
    sign = -1 if clockwise else 1
    for n, angle in enumerate(np.linspace(3., 3.+sign*2*math.pi, 100)):
        t = n*.2
        state = VesselState(10*math.cos(angle), 10*math.sin(angle), angle+sign*math.pi/2, stamp=t)
        intent = m.tick(state, grid(t), t)
    assert intent.kind == 'navigate' and intent.goal[:2] == (0., 0.)
    assert m.status(state, t)['completed_laps'] == pytest.approx(1.)


def test_orbit_does_not_count_reverse_jitter_or_excursions_outside_precision_band():
    m = manager('orbit', clockwise=False)
    for n, angle in enumerate([3., 3.1, 3.05, 3.1]):
        state = VesselState(10*math.cos(angle), 10*math.sin(angle), angle+math.pi/2, stamp=n*.1)
        m.tick(state, grid(n*.1), n*.1)
    assert m.status(state, .3)['completed_laps'] == pytest.approx(.1/(2*math.pi))
    outside = VesselState(12*math.cos(4.), 12*math.sin(4.), 4.+math.pi/2, stamp=.4)
    m.tick(outside, grid(.4), .4)
    rejoin = VesselState(10*math.cos(5.), 10*math.sin(5.), 5.+math.pi/2, stamp=.5)
    m.tick(rejoin, grid(.5), .5)
    assert m.status(rejoin, .5)['completed_laps'] == pytest.approx(.1/(2*math.pi))


def test_orbit_duration_includes_entry_and_clock_pause_but_not_rewind():
    m = manager('orbit', duration=5.)
    state = VesselState(x=0., stamp=2.)
    assert m.tick(state, grid(2.), 2.).kind == 'orbit_entry'
    assert m.tick(state, grid(2.), 2.).kind == 'orbit_entry'
    assert m.status(state, 2.)['elapsed_time'] == 2.
    assert m.tick(replace(state, stamp=1.), grid(1.), 1.).kind == 'stop'
    m = manager('orbit', duration=5.)
    assert m.tick(replace(state, stamp=5.), grid(5.), 5.).phase == 'RETURN_CENTER'


def test_idle_mission_detects_intrusion_without_allowing_hull_escape():
    m = manager('station_keep')
    assert m.tick(VesselState(), grid(), 0.).kind == 'hold'
    intent = m.tick(VesselState(stamp=.1), grid(.1, (3., 0.)), .1)
    assert intent.kind == 'escape' and len(intent.candidates) > 0
    m = manager('station_keep')
    assert m.tick(VesselState(), grid(0., (0., 0.)), 0.).kind == 'stop'
    assert m.status(VesselState(), 0.)['blocked_reason'] == 'HULL_COLLISION'


def test_safety_release_requires_fresh_distinct_maps_and_original_target_clear():
    m = manager('station_keep')
    intent = m.tick(VesselState(), grid(0., (3., 0.)), 0.)
    assert intent.kind == 'escape'
    m.on_plan(type('Response', (), {'status': AlgorithmStatus.SUCCESS, 'selected_goal': (-8., 0., 0.), 'message': ''})())
    state = VesselState(x=-8., stamp=.5)
    assert m.tick(state, grid(.5, (0., 0.)), .5).kind == 'stop'
    # Repeated publication of the same map is not a release dwell.
    same = grid(.5)
    m.tick(state, same, .5)
    assert m.tick(replace(state, stamp=2.7), same, 2.7).kind == 'stop'
    m.tick(replace(state, stamp=3.), grid(3.), 3.)
    m.tick(replace(state, stamp=4.), grid(4.), 4.)
    restored = m.tick(replace(state, stamp=5.1), grid(5.1), 5.1)
    assert restored.kind == 'navigate' and restored.phase == 'APPROACH_STATION'
    assert m.spec.mode == 'station_keep' and m.target_pose == (0., 0., 0.)


def test_active_mission_rejects_stale_and_mismatched_input():
    m = manager('station_keep')
    assert m.tick(VesselState(stamp=0.), grid(0.), 2.).kind == 'stop'
    assert m.tick(VesselState(frame_id='wrong'), grid(), 0.).kind == 'stop'


def test_distant_station_uses_global_navigation_then_latches_direct_holding():
    m = manager('station_keep')
    first = m.tick(VesselState(x=20.), grid(), 0.)
    assert first.kind == 'navigate' and first.phase == 'APPROACH_STATION'
    assert m.tick(VesselState(x=1., stamp=.1), grid(.1), .1).kind == 'hold'
    assert m.tick(VesselState(x=2.2, stamp=.2), grid(.2), .2).kind == 'hold'


def test_circle_entry_alignment_locks_pose_under_small_position_noise():
    m = manager('orbit')
    first = m.tick(VesselState(x=10.), grid(), 0.)
    assert first.kind == 'hold'
    second = m.tick(VesselState(x=10., y=.01, stamp=.1), grid(.1), .1)
    assert second.signature == first.signature


def test_explicit_controller_safety_preemption_requests_safe_exit():
    m = manager('station_keep')
    m.tick(VesselState(), grid(), 0.)
    m.request_escape('unsafe predicted turn')
    assert m.tick(VesselState(stamp=.1), grid(.1), .1).kind == 'escape'


@pytest.mark.parametrize('mode, state, obstacle, excluded_goal', [
    ('station_keep', VesselState(), (16.5, 0.), (8., 0.)),
    ('orbit', VesselState(y=10.), (18.5, 0.), (10., 0.)),
])
def test_escape_selection_reserves_the_position_tolerance_disk(mode, state, obstacle, excluded_goal):
    m = manager(mode, position_tolerance=.5)
    m.request_escape('candidate arrival disk regression')
    intent = m.tick(state, grid(obstacle=obstacle), 0.)
    assert intent.kind == 'escape' and intent.candidates
    assert not any(np.allclose(pose[:2], excluded_goal) for pose in intent.candidates)


def test_position_teleport_cannot_increment_laps():
    m = manager('orbit', clockwise=False)
    m.tick(VesselState(x=10., psi=math.pi/2), grid(), 0.)
    state = VesselState(x=10*math.cos(2.), y=10*math.sin(2.), psi=2.+math.pi/2, stamp=.1)
    m.tick(state, grid(.1), .1)
    assert m.status(state, .1)['completed_laps'] == 0.


def test_station_safety_recovery_returns_using_navigation_after_previously_holding():
    m = manager('station_keep')
    m.tick(VesselState(), grid(), 0.)
    m.tick(VesselState(stamp=.1), grid(.1, (3., 0.)), .1)
    m.on_plan(type('Response', (), {'status': AlgorithmStatus.SUCCESS, 'selected_goal': (-8., 0., 0.), 'message': ''})())
    state = VesselState(x=-8.)
    for now in (.5, 1.5, 2.5):
        intent = m.tick(replace(state, stamp=now), grid(now), now)
    assert intent.kind == 'navigate' and intent.phase == 'APPROACH_STATION'


def test_safety_release_does_not_count_unobserved_map_gap():
    m = manager('station_keep')
    m.tick(VesselState(), grid(0., (3., 0.)), 0.)
    m.on_plan(type('Response', (), {'status': AlgorithmStatus.SUCCESS, 'selected_goal': (-8., 0., 0.), 'message': ''})())
    m.tick(VesselState(x=-8., stamp=.5), grid(.5), .5)
    assert m.tick(VesselState(x=-8., stamp=3.), grid(3.), 3.).kind == 'stop'


def test_small_known_region_reports_no_safe_exit_instead_of_unknown_escape():
    m = manager('station_keep')
    small = GridSnapshot(np.zeros((20, 20), np.uint8), .25, -2.5, -2.5)
    intent = m.tick(VesselState(), small, 0.)
    assert intent.kind == 'stop' and intent.phase == 'AVOIDANCE_BLOCKED'
    assert m.blocked_reason == 'NO_SAFE_EXIT'


def departure_response(heading=math.pi/2, goal=(8., 0., 0.), departure_xy=(99., 99.), backoff_distance=0.):
    return SimpleNamespace(status=AlgorithmStatus.SUCCESS, selected_goal=goal, message='',
                           metadata={'departure_pose': (*departure_xy, heading), 'map_stamp': 0.,
                                     'backoff_distance': backoff_distance})


def departure_manager(heading=math.pi/2, mode='station_keep', backoff_distance=0., **kwargs):
    m = manager(mode, **kwargs)
    m.request_escape('turning departure required')
    assert m.tick(VesselState(u=.2), grid(), 0.).kind == 'escape'
    m.on_plan(departure_response(heading, backoff_distance=backoff_distance))
    return m


@pytest.mark.parametrize('velocity', [(.041, 0., 0.), (-.041, 0., 0.), (0., .041, 0.), (0., 0., .021)])
def test_escape_departure_cannot_turn_before_measured_translation_and_yaw_stop(velocity):
    m = departure_manager()
    state = VesselState(x=.2, u=velocity[0], v=velocity[1], r=velocity[2], stamp=.1)
    intent = m.tick(state, grid(.1), .1)
    assert intent.kind == 'brake' and intent.phase == 'BRAKING_ESCAPE'
    assert m._departure_pose is None
    assert m.temporary_goal == (8., 0., 0.)


def test_escape_alignment_freezes_measured_stop_xy_not_worker_geometry_and_holds_it():
    m = departure_manager()
    m.tick(VesselState(x=.2, u=.1, stamp=.1), grid(.1), .1)
    first = m.tick(VesselState(x=.3, y=.1, stamp=.2), grid(.2), .2)
    assert first.kind == 'hold' and first.phase == 'ALIGN_ESCAPE'
    assert first.goal == pytest.approx((.3, .1, math.pi/2))
    assert first.goal[:2] != (99., 99.)
    second = m.tick(VesselState(x=.32, y=.1, psi=.4, stamp=.3), grid(.3), .3)
    assert second.signature == first.signature
    assert m.temporary_goal == (8., 0., 0.)


@pytest.mark.parametrize('bad', [{'psi': math.radians(6)}, {'x': .26}, {'u': .041}, {'r': .021}])
def test_escape_alignment_does_not_release_until_all_pose_and_speed_constraints_hold(bad):
    m = departure_manager(heading=0.)
    assert m.tick(VesselState(stamp=.1), grid(.1), .1).phase == 'ALIGN_ESCAPE'
    for now in (.2, .3, .4, .5, .6, .7):
        state = VesselState(stamp=now, **bad)
        assert m.tick(state, grid(now), now).phase == 'ALIGN_ESCAPE'
    for now in (.8, .9, 1., 1.1, 1.2):
        assert m.tick(VesselState(stamp=now), grid(now), now).phase == 'ALIGN_ESCAPE'
    released = m.tick(VesselState(stamp=1.3), grid(1.3), 1.3)
    assert released.kind == 'escape' and released.goal == (8., 0., 0.)
    assert m._departure_heading is None and m._departure_pose is None


def test_escape_alignment_scales_position_threshold_with_requested_tolerance():
    m = departure_manager(heading=0., position_tolerance=.2)
    m.tick(VesselState(stamp=.1), grid(.1), .1)
    for now in (.2, .3, .4, .5, .6, .7):
        assert m.tick(VesselState(x=.11, stamp=now), grid(now), now).phase == 'ALIGN_ESCAPE'


def test_escape_alignment_requires_distinct_fresh_state_samples_not_repeated_stamp():
    m = departure_manager(heading=0.)
    fixed = VesselState(stamp=.1)
    for now in (.1, .2, .3, .4, .5):
        assert m.tick(fixed, grid(now), now).phase == 'ALIGN_ESCAPE'
    # Fresh after a long unobserved odometry gap starts a new dwell.
    assert m.tick(VesselState(stamp=.8), grid(.8), .8).phase == 'ALIGN_ESCAPE'
    for now in (.9, 1., 1.1, 1.2):
        assert m.tick(VesselState(stamp=now), grid(now), now).phase == 'ALIGN_ESCAPE'
    assert m.tick(VesselState(stamp=1.3), grid(1.3), 1.3).kind == 'escape'


def test_escape_alignment_stale_input_resets_dwell_without_changing_frozen_pose():
    m = departure_manager(heading=0.)
    m.tick(VesselState(stamp=.1), grid(.1), .1)
    m.tick(VesselState(stamp=.3), grid(.3), .3)
    pose = m._departure_pose
    assert m.tick(VesselState(stamp=.3), grid(.9), .9).phase == 'WAITING_INPUT'
    assert m._departure_since is None and m._departure_pose == pose
    for now in (1., 1.1, 1.2, 1.3, 1.4):
        assert m.tick(VesselState(stamp=now), grid(now), now).phase == 'ALIGN_ESCAPE'
    assert m.tick(VesselState(stamp=1.5), grid(1.5), 1.5).kind == 'escape'


def test_repeated_departure_metadata_does_not_restart_completed_or_active_prelude():
    m = departure_manager(heading=0.)
    response = departure_response(heading=0.)
    m.on_plan(response)  # Duplicate before the first post-plan tick.
    first = m.tick(VesselState(stamp=.1), grid(.1), .1)
    m.on_plan(response)  # Duplicate while the manager is holding.
    assert m.tick(VesselState(stamp=.2), grid(.2), .2).signature == first.signature
    for now in (.3, .4, .5, .6):
        intent = m.tick(VesselState(stamp=now), grid(now), now)
    assert intent.kind == 'escape'
    m.on_plan(response)  # Same goal and yaw cannot restart the completed prelude.
    assert m.tick(VesselState(stamp=.7), grid(.7), .7).kind == 'escape'
    assert m._departure_heading is None
    m.request_escape('new safety episode')
    m.tick(VesselState(stamp=.8), grid(.8), .8)
    m.on_plan(response)
    assert m.tick(VesselState(stamp=.9), grid(.9), .9).phase == 'ALIGN_ESCAPE'


def test_departure_prelude_preserves_original_circle_and_completed_progress():
    m = manager('orbit', clockwise=False, laps=2)
    for n, angle in enumerate([0., .05, .1]):
        now = n*.2
        state = VesselState(x=10*math.cos(angle), y=10*math.sin(angle), psi=angle+math.pi/2, stamp=now)
        m.tick(state, grid(now), now)
    progress, circle, spec = m._net_angle, m._circle, m.spec
    m.request_escape()
    m.tick(VesselState(stamp=.5), grid(.5), .5)
    m.on_plan(departure_response(heading=0., goal=(-10., 0., -math.pi/2)))
    for now in (.6, .7, .8, .9, 1., 1.1):
        intent = m.tick(VesselState(stamp=now), grid(now), now)
    assert intent.kind == 'escape' and intent.goal == (-10., 0., -math.pi/2)
    assert m._circle is circle and m.spec is spec and m._net_angle == progress
    assert m.target_pose == (0., 0., 0.)


@pytest.mark.parametrize('event', ['target_invalid', 'resume', 'request_escape', 'plan_failure'])
def test_departure_state_is_cleared_when_its_escape_reference_is_invalidated(event):
    m = departure_manager()
    m.tick(VesselState(stamp=.1), grid(.1), .1)
    if event == 'target_invalid':
        m.tick(VesselState(stamp=.2), grid(.2, (8., 0.)), .2)
    elif event == 'resume':
        m._resume_nominal()
    elif event == 'request_escape':
        m.request_escape('new obstacle')
    else:
        m._emit('escape', 'AVOIDING', m.temporary_goal)
        m.on_plan(SimpleNamespace(status=AlgorithmStatus.NO_PATH, message='failed', metadata={}))
    assert m._departure_heading is None and m._departure_pose is None
    assert m._departure_since is None and not m._departure_seen


def test_malformed_departure_pose_cannot_activate_rotation_target():
    m = manager('station_keep')
    m.request_escape()
    m.tick(VesselState(), grid(), 0.)
    m.on_plan(departure_response(heading=math.nan))
    assert m._departure_pose is None and m._departure_heading is None
    assert m.temporary_goal is None and m.blocked_reason == 'INVALID_DEPARTURE_POSE'


def test_backoff_freezes_measured_stopped_heading_and_then_aligns_at_actual_arrival():
    m = departure_manager(heading=math.pi, backoff_distance=.5)
    assert m.tick(VesselState(x=.2, u=.1, stamp=.1), grid(.1), .1).phase == 'BRAKING_ESCAPE'
    first = m.tick(VesselState(x=.3, y=.1, psi=math.pi/2, stamp=.2), grid(.2), .2)
    assert first.kind == 'hold' and first.phase == 'BACKING_ESCAPE'
    assert first.goal == pytest.approx((.3, -.4, math.pi/2))
    # Duplicate metadata must not turn a fixed rearward target into a moving one.
    m.on_plan(departure_response(heading=math.pi, backoff_distance=.5))
    same = m.tick(VesselState(x=.32, y=.05, psi=math.pi/2+.02, stamp=.3), grid(.3), .3)
    assert same.signature == first.signature
    # Being in the old .25 m arrival disk is insufficient for a .5 m backoff.
    for now in (.4, .5, .6, .7, .8, .9):
        assert m.tick(VesselState(x=.3, y=-.3, psi=math.pi/2, stamp=now), grid(now), now).phase == 'BACKING_ESCAPE'
    for now in (1., 1.1, 1.2, 1.3, 1.4):
        assert m.tick(VesselState(x=.32, y=-.39, psi=math.pi/2, stamp=now), grid(now), now).phase == 'BACKING_ESCAPE'
    aligned = m.tick(VesselState(x=.32, y=-.39, psi=math.pi/2, stamp=1.5), grid(1.5), 1.5)
    assert aligned.phase == 'ALIGN_ESCAPE'
    assert aligned.goal == pytest.approx((.32, -.39, -math.pi))
    for now in (1.6, 1.7, 1.8, 1.9, 2.):
        assert m.tick(VesselState(x=.32, y=-.39, psi=math.pi, stamp=now), grid(now), now).phase == 'ALIGN_ESCAPE'
    released = m.tick(VesselState(x=.32, y=-.39, psi=math.pi, stamp=2.1), grid(2.1), 2.1)
    assert released.kind == 'escape' and released.goal == (8., 0., 0.)
    assert m.status(VesselState(), 2.1)['backoff_pose'] is None


@pytest.mark.parametrize('bad', [{'x': -.44}, {'psi': math.radians(6.)},
                                 {'u': .041}, {'u': -.041}, {'v': .041}, {'r': .021}])
def test_backoff_requires_precise_position_heading_and_stopped_motion_for_full_dwell(bad):
    m = departure_manager(backoff_distance=.5)
    assert m.tick(VesselState(stamp=.1), grid(.1), .1).phase == 'BACKING_ESCAPE'
    for now in (.2, .3, .4, .5, .6, .7):
        state = VesselState(stamp=now, **dict({'x': -.5}, **bad))
        assert m.tick(state, grid(now), now).phase == 'BACKING_ESCAPE'
    for now in (.8, .9, 1., 1.1, 1.2):
        assert m.tick(VesselState(x=-.5, stamp=now), grid(now), now).phase == 'BACKING_ESCAPE'
    assert m.tick(VesselState(x=-.5, stamp=1.3), grid(1.3), 1.3).phase == 'ALIGN_ESCAPE'


def test_backoff_uses_requested_precision_divided_by_ten_and_distinct_odometry():
    m = departure_manager(backoff_distance=.5, position_tolerance=.2)
    m.tick(VesselState(stamp=.1), grid(.1), .1)
    for now in (.2, .3, .4, .5, .6, .7):
        assert m.tick(VesselState(x=-.47, stamp=now), grid(now), now).phase == 'BACKING_ESCAPE'
    fixed = VesselState(x=-.49, stamp=.8)
    for now in (.8, .9, 1., 1.1, 1.2):
        assert m.tick(fixed, grid(now), now).phase == 'BACKING_ESCAPE'
    # An unobserved state gap restarts the dwell instead of completing it.
    for now in (1.4, 1.5, 1.6, 1.7, 1.8):
        assert m.tick(VesselState(x=-.49, stamp=now), grid(now), now).phase == 'BACKING_ESCAPE'
    assert m.tick(VesselState(x=-.49, stamp=1.9), grid(1.9), 1.9).phase == 'ALIGN_ESCAPE'


@pytest.mark.parametrize('event', ['stale_state', 'stale_map', 'target_invalid', 'request_escape', 'plan_failure'])
def test_backoff_evidence_is_discarded_after_input_or_reference_invalidation(event):
    m = departure_manager(backoff_distance=.5)
    assert m.tick(VesselState(stamp=.1), grid(.1), .1).phase == 'BACKING_ESCAPE'
    if event == 'stale_state':
        assert m.tick(VesselState(stamp=.1), grid(.7), .7).phase == 'WAITING_INPUT'
    elif event == 'stale_map':
        assert m.tick(VesselState(stamp=2.), grid(.1), 2.).phase == 'WAITING_INPUT'
    elif event == 'target_invalid':
        m.tick(VesselState(stamp=.2), grid(.2, (8., 0.)), .2)
    elif event == 'request_escape':
        m.request_escape('new obstacle')
    else:
        m.on_plan(SimpleNamespace(status=AlgorithmStatus.NO_PATH, message='failed', metadata={'map_stamp': .1}))
    assert m.temporary_goal is None
    assert m._departure_heading is None and m._departure_pose is None
    assert m.status(VesselState(), 2.)['backoff_pose'] is None


@pytest.mark.parametrize('distance', [-.5, 3.01, math.nan, math.inf, True, '.5', None, []])
def test_backoff_rejects_malformed_or_out_of_bounds_distance(distance):
    m = departure_manager(backoff_distance=distance)
    assert m.temporary_goal is None and m._departure_heading is None
    assert m.blocked_reason == 'INVALID_BACKOFF_DISTANCE'


@pytest.mark.parametrize('distance', [.5, 1., 2., 3.])
def test_backoff_accepts_planner_distances_without_changing_forward_goal(distance):
    m = departure_manager(backoff_distance=distance)
    intent = m.tick(VesselState(x=.2, stamp=.1), grid(.1), .1)
    assert intent.phase == 'BACKING_ESCAPE'
    assert intent.goal == pytest.approx((.2-distance, 0., 0.))
    assert m.temporary_goal == (8., 0., 0.)


def test_backoff_then_alignment_completes_with_real_holding_controller_and_dynamics():
    from common.dynamics import braking_command, clear_control, step_dynamics
    from common.models import ControlCommand, USVModel
    from controller.station_keeping_controller import StationKeepingController

    m = departure_manager(backoff_distance=.5)
    model = USVModel()
    controller = StationKeepingController(model)
    state, previous = VesselState(u=.2), ControlCommand()
    signature, rearward_target, rotation_target = None, None, None
    for tick in range(1, 1201):
        now = tick*.1
        state = replace(state, stamp=now)
        current_map = grid(now)
        intent = m.tick(state, current_map, now)
        if intent.kind == 'escape':
            break
        if intent.signature != signature:
            controller.reset()
            controller.observe_command(previous)
            signature = intent.signature
        if intent.phase == 'BACKING_ESCAPE':
            rearward_target = intent.goal
        elif intent.phase == 'ALIGN_ESCAPE' and rotation_target is None:
            rotation_target = intent.goal
            assert math.hypot(state.x-rearward_target[0], state.y-rearward_target[1]) <= .05
            assert math.hypot(state.u, state.v) <= .04 and abs(state.r) <= .02
        command = (braking_command(state, model) if intent.kind == 'brake'
                   else controller.compute(state, intent.goal, .1))
        assert clear_control(state, command, current_map, model, .1)
        state = step_dynamics(state, command, model, .1)
        previous = command
    else:
        pytest.fail('Rearward departure did not finish within 120 seconds')
    assert rearward_target is not None and rotation_target is not None
    assert intent.goal == (8., 0., 0.)
    assert abs(state.psi-math.pi/2) <= math.radians(5.)
