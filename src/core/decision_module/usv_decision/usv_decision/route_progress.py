"""Ordered mandatory-point progress from actual odometry, independent of planning."""
import math


class RouteProgress:
    def __init__(self, points, radius=1.5, max_speed=3., frame_id='otter/map'):
        self.points = tuple(tuple(float(value) for value in point) for point in points)
        self.radius, self.max_speed = float(radius), float(max_speed)
        if not 1 <= len(self.points) <= 255:
            raise ValueError('A route requires 1..255 waypoints')
        if (not math.isfinite(self.radius) or self.radius <= 0.
                or not math.isfinite(self.max_speed) or self.max_speed <= 0.):
            raise ValueError('Radius and maximum speed must be finite and positive')
        if any(len(point) != 4 or not all(map(math.isfinite, point)) or point[3] <= 0.
               for point in self.points):
            raise ValueError('Waypoints require finite x/y/heading and positive speed')
        self.frame_id = frame_id
        self.index = 0
        self._previous = None

    @property
    def remaining(self):
        return self.points[self.index:]

    @property
    def terminal(self):
        return self.index == len(self.points) - 1

    @property
    def passed_count(self):
        return self.index

    def finish(self):
        if not self.terminal:
            raise ValueError('Previous mandatory waypoints have not all been passed')
        self.index = len(self.points)

    def _crossing(self, start, end, point, after):
        """Earliest segment fraction in a waypoint disk, after the previous hit."""
        dx, dy = end.x - start.x, end.y - start.y
        ox, oy = start.x - point[0], start.y - point[1]
        a = dx * dx + dy * dy
        if a <= 1e-12:
            return after if ox * ox + oy * oy <= self.radius ** 2 else None
        b = 2. * (ox * dx + oy * dy)
        c = ox * ox + oy * oy - self.radius ** 2
        discriminant = b * b - 4. * a * c
        if discriminant < 0.:
            return None
        root = math.sqrt(discriminant)
        entry, leave = (-b - root) / (2. * a), (-b + root) / (2. * a)
        hit = max(0., after, entry)
        return hit if hit <= min(1., leave) + 1e-12 else None

    def update(self, state):
        """Advance intermediate points only; final low-speed arrival needs finish()."""
        if not state.is_finite() or state.frame_id != self.frame_id:
            return False
        previous = self._previous
        if previous is not None and state.stamp <= previous.stamp:
            return False
        self._previous = state
        interpolate = False
        if previous is not None:
            dt = state.stamp - previous.stamp
            distance = math.hypot(state.x - previous.x, state.y - previous.y)
            # A reset/teleport supplies a new baseline but cannot confirm a point.
            if distance > self.max_speed * dt + 1.:
                return False
            interpolate = dt <= 1.
        before, fraction = self.index, 0.
        while self.index < len(self.points) - 1:
            point = self.points[self.index]
            if interpolate:
                hit = self._crossing(previous, state, point, fraction)
                if hit is None:
                    break
                fraction = hit
            elif math.hypot(state.x - point[0], state.y - point[1]) > self.radius:
                break
            self.index += 1
        return self.index != before
