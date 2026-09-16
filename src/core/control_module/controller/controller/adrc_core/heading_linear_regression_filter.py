"""Angle-aware least-squares filter with measured sample times.

Adjacent valid observations must differ by less than pi radians physically.
"""
import math


def _finite(value, name="value"):
    value = float(value)
    if not math.isfinite(value):
        raise ValueError(f"{name} must be finite")
    return value


def _normalize_angle(angle):
    value = math.fmod(_finite(angle), 2.0 * math.pi)
    if value > math.pi:
        value -= 2.0 * math.pi
    elif value <= -math.pi:
        value += 2.0 * math.pi
    return value


class HeadingLinearRegressionFilter:
    def __init__(self, sample_time=0.05, window_length=10, history_length=50):
        self.sample_time = _finite(sample_time, "sample_time")
        if self.sample_time <= 0:
            raise ValueError("sample_time must be positive")
        for name, value in (("window_length", window_length), ("history_length", history_length)):
            number = _finite(value, name)
            if isinstance(value, bool) or not number.is_integer() or number < 2:
                raise ValueError(f"{name} must be an integer at least 2")
            setattr(self, name, int(number))
        if self.history_length < self.window_length:
            raise ValueError("history_length must be at least window_length")
        self.reset()

    def reset(self, initial_value=0.0):
        initial_value = _finite(initial_value)
        self.position_history = []
        self.rate_history = []
        self.time_history = []
        self.last_raw = None
        self.last_unwrapped = None
        self.last_stamp = None
        self.position = initial_value
        self.rate = 0.0
        self.acceleration = 0.0

    @staticmethod
    def _fit(values, times=None):
        values = [_finite(v) for v in values]
        times = list(range(len(values))) if times is None else [_finite(t) for t in times]
        if len(values) != len(times):
            raise ValueError("values and times must have the same length")
        if len(values) < 2:
            return (values[-1] if values else 0.0), 0.0
        # Subtract the first timestamp before averaging to protect epoch stamps.
        times = [_finite(t - times[0]) for t in times]
        mean_t = sum(times) / len(times)
        mean_y = sum(values) / len(values)
        denominator = sum((t - mean_t) ** 2 for t in times)
        if not math.isfinite(denominator) or denominator <= 0:
            raise ValueError("sample times must have finite nonzero variance")
        slope = sum((t - mean_t) * (y - mean_y) for t, y in zip(times, values)) / denominator
        latest = mean_y + slope * (times[-1] - mean_t)
        return _finite(latest), _finite(slope)

    def update(self, measurement, *, stamp=None, dt=None):
        raw = _finite(measurement, "measurement")
        step = self.sample_time if dt is None else _finite(dt, "dt")
        if step <= 0:
            raise ValueError("dt must be positive")
        time = (0.0 if self.last_stamp is None else self.last_stamp + step) if stamp is None else _finite(stamp, "stamp")
        time = _finite(time, "stamp")
        if self.last_stamp is not None:
            if time < self.last_stamp:
                raise ValueError("sample timestamp moved backwards")
            if time == self.last_stamp:
                return
        if self.last_raw is None:
            value = raw
        else:
            delta = _finite(raw - self.last_raw)
            value = _finite(self.last_unwrapped + math.atan2(math.sin(delta), math.cos(delta)))
        # Stage all results before committing histories; invalid samples are atomic.
        positions = (self.position_history + [value])[-self.history_length:]
        times = (self.time_history + [time])[-self.history_length:]
        position, rate = self._fit(positions[-self.window_length:], times[-self.window_length:])
        rates = (self.rate_history + [rate])[-self.history_length:]
        _, acceleration = self._fit(rates[-self.window_length:], times[-self.window_length:])
        self.position_history, self.rate_history, self.time_history = positions, rates, times
        self.last_raw, self.last_unwrapped, self.last_stamp = raw, value, time
        self.position, self.rate, self.acceleration = position, rate, acceleration

    def output(self):
        return _normalize_angle(self.position), self.rate, self.acceleration
