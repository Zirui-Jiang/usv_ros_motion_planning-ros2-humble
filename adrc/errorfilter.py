"""Heading-angle filter based on the supplied C linear-regression logic."""

import math


def _normalize_angle(angle):
    normalized = math.fmod(float(angle), 2.0 * math.pi)
    if normalized > math.pi:
        normalized -= 2.0 * math.pi
    elif normalized <= -math.pi:
        normalized += 2.0 * math.pi
    return normalized


class HeadingLinearRegressionFilter:
    def __init__(self, sample_time, window_length=10, history_length=50):
        self.sample_time = float(sample_time)
        if self.sample_time <= 0.0:
            raise ValueError("sample_time must be positive")
        self.window_length = max(2, int(round(float(window_length))))
        self.history_length = max(
            self.window_length, int(round(float(history_length)))
        )
        self.reset()

    @staticmethod
    def _fit_latest(values):
        count = len(values)
        if count < 2:
            return float(values[-1]) if values else 0.0, 0.0
        mean_x = 0.5 * (count - 1)
        mean_y = sum(float(value) for value in values) / count
        denominator = sum((index - mean_x) ** 2
                          for index in range(count))
        slope = sum(
            (index - mean_x) * (float(value) - mean_y)
            for index, value in enumerate(values)
        ) / denominator
        intercept = mean_y - slope * mean_x
        return intercept + slope * (count - 1), slope

    def reset(self, initial_value=0.0):
        self.position_history = []
        self.speed_history = []
        self.last_unwrapped_measurement = None
        self.filtered_position = float(initial_value)
        self.speed = 0.0
        self.acceleration = 0.0

    def _unwrap_measurement(self, measurement):
        measurement = float(measurement)
        if self.last_unwrapped_measurement is None:
            unwrapped = measurement
        else:
            unwrapped = measurement
            delta = unwrapped - self.last_unwrapped_measurement
            if delta > math.pi:
                unwrapped -= 2.0 * math.pi
            elif delta < -math.pi:
                unwrapped += 2.0 * math.pi
        self.last_unwrapped_measurement = unwrapped
        return unwrapped

    def update(self, measurement):
        unwrapped = self._unwrap_measurement(measurement)
        self.position_history.append(unwrapped)
        self.position_history = self.position_history[-self.history_length:]

        if len(self.position_history) > self.window_length:
            position_window = self.position_history[-self.window_length:]
            self.filtered_position, slope = self._fit_latest(position_window)
            self.speed = slope / self.sample_time
        else:
            self.filtered_position = unwrapped
            self.speed = 0.0

        self.speed_history.append(self.speed)
        self.speed_history = self.speed_history[-self.history_length:]
        if len(self.speed_history) > self.window_length:
            speed_window = self.speed_history[-self.window_length:]
            _, slope = self._fit_latest(speed_window)
            self.acceleration = slope / self.sample_time
        else:
            self.acceleration = 0.0

    def output(self):
        return (
            float(_normalize_angle(self.filtered_position)),
            float(self.speed),
            float(self.acceleration),
        )


_filter = None


def initialize_filter(sample_time, window_length=10, history_length=50):
    global _filter
    _filter = HeadingLinearRegressionFilter(sample_time, window_length, history_length)
    return 1


def reset_filter(initial_value=0.0):
    if _filter is None:
        raise RuntimeError("Call initialize_filter before reset_filter")
    _filter.reset(initial_value)
    return 1


def filter_step(measurement):
    """Compatibility call for non-feedback standalone tests."""
    if _filter is None:
        raise RuntimeError("Call initialize_filter before filter_step")
    _filter.update(measurement)
    return _filter.output()


def filter_output():
    """Return the stored state without reading the current input."""
    if _filter is None:
        raise RuntimeError("Call initialize_filter before filter_output")
    return _filter.output()


def filter_update(measurement):
    """Update the stored state after the Output phase."""
    if _filter is None:
        raise RuntimeError("Call initialize_filter before filter_update")
    _filter.update(measurement)
    return 1