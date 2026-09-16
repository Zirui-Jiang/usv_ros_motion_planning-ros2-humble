"""Angle-aware linear-regression filter used by the CN heading controller."""

import math


def _normalize_angle(angle):
    value = math.fmod(float(angle), 2.0 * math.pi)
    if value > math.pi:
        value -= 2.0 * math.pi
    elif value <= -math.pi:
        value += 2.0 * math.pi
    return value


class HeadingLinearRegressionFilter:
    def __init__(self, sample_time=0.05, window_length=10, history_length=50):
        self.sample_time = float(sample_time)
        if not math.isfinite(self.sample_time) or self.sample_time <= 0.0:
            raise ValueError("sample_time must be finite and positive")
        self.window_length = max(2, int(round(float(window_length))))
        self.history_length = max(
            self.window_length, int(round(float(history_length)))
        )
        self.reset()

    def reset(self, initial_value=0.0):
        self.position_history = []
        self.rate_history = []
        self.last_unwrapped = None
        self.position = float(initial_value)
        self.rate = 0.0
        self.acceleration = 0.0

    @staticmethod
    def _fit(values):
        n = len(values)
        if n < 2:
            return (float(values[-1]) if values else 0.0), 0.0
        mean_x = 0.5 * (n - 1)
        mean_y = sum(float(v) for v in values) / n
        denominator = sum((i - mean_x) ** 2 for i in range(n))
        slope = sum(
            (i - mean_x) * (float(v) - mean_y)
            for i, v in enumerate(values)
        ) / denominator
        intercept = mean_y - slope * mean_x
        return intercept + slope * (n - 1), slope

    def _unwrap(self, measurement):
        value = float(measurement)
        if self.last_unwrapped is not None:
            delta = value - self.last_unwrapped
            if delta > math.pi:
                value -= 2.0 * math.pi
            elif delta < -math.pi:
                value += 2.0 * math.pi
        self.last_unwrapped = value
        return value

    def update(self, measurement):
        value = self._unwrap(measurement)
        self.position_history.append(value)
        self.position_history = self.position_history[-self.history_length:]

        window = self.position_history[-self.window_length:]
        self.position, slope = self._fit(window)
        self.rate = slope / self.sample_time

        self.rate_history.append(self.rate)
        self.rate_history = self.rate_history[-self.history_length:]
        rate_window = self.rate_history[-self.window_length:]
        _, rate_slope = self._fit(rate_window)
        self.acceleration = rate_slope / self.sample_time

    def output(self):
        return (
            _normalize_angle(self.position),
            float(self.rate),
            float(self.acceleration),
        )
