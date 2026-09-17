"""Geometric candidates; searches and swept checks remain in the worker."""
import math
import numpy as np
from common.collision import footprint_path_is_clear
from common.math_helper import wrap_angle


def pose_is_clear(grid, pose, config, margin=0.):
    return footprint_path_is_clear(
        grid, [pose], config.get('navigation.footprint_length', 2.2)+2*margin,
        config.get('navigation.footprint_width', 1.2)+2*margin)


def turning_envelope_is_clear(grid, pose, config, margin=5., arrival_tolerance=0.):
    """Enclose the rotated release rectangle and the entire arrival disk.

    ``pose_is_clear`` adds the margin to both body axes before rotation.
    Therefore its circumradius is hypot(L/2 + margin, W/2 + margin),
    rather than the bare hull circumradius plus a circular margin.
    """
    half_diagonal = math.hypot(.5*config.get('navigation.footprint_length', 2.2)+margin,
                              .5*config.get('navigation.footprint_width', 1.2)+margin)
    side = 2*(half_diagonal+arrival_tolerance)
    return footprint_path_is_clear(grid, [(pose[0], pose[1], 0.)], side, side)


def entry_candidates(circle, state, *, forward_only=False):
    """Both external tangent points plus the 15 degree circle discretization.

    Recovery covers the remaining circle in the commanded direction, excluding
    a full revolution back to the current radial point. Sorting by straight
    distance gives a lower bound for distance-plus-turning cost.
    """
    dx, dy = state.x-circle.center[0], state.y-circle.center[1]
    theta, distance = math.atan2(dy, dx), math.hypot(dx, dy)
    if forward_only:
        angles = [theta+circle.direction*math.radians(a) for a in range(15, 360, 15)]
    else:
        angles = list(np.arange(24)*math.pi/12)
        if distance > circle.radius+1e-9:
            tangent_offset = math.acos(circle.radius/distance)
            angles = [theta-tangent_offset, theta+tangent_offset, *angles]
    poses, seen = [], set()
    for angle in angles:
        pose = circle.pose_at(angle)
        key = tuple(round(v, 8) for v in pose[:2])
        if key not in seen:
            seen.add(key)
            poses.append(pose)
    return tuple(sorted(poses, key=lambda p: math.hypot(p[0]-state.x, p[1]-state.y)))


def escape_candidates(state, grid, config, arrival_tolerance=0.):
    """Two rings, 16 points each; release envelope must be clear at the goal."""
    candidates = []
    for radius in (4., 8.):
        for n in range(16):
            angle = state.psi+n*math.pi/8
            pose = (state.x+radius*math.cos(angle), state.y+radius*math.sin(angle), wrap_angle(angle))
            if turning_envelope_is_clear(grid, pose, config, arrival_tolerance=arrival_tolerance):
                cost = abs(wrap_angle(angle-state.psi))+radius/8.
                candidates.append((cost, pose))
    return tuple(pose for _, pose in sorted(candidates, key=lambda item: item[0]))
