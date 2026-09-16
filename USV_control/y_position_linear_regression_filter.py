"""Independent earth-fixed y-position linear-regression filter module."""

from linear_regression_filter import LinearRegressionFilter


class YPositionLinearRegressionFilter(LinearRegressionFilter):
    pass


_filter = None


def initialize_filter(sample_time, window_length=10, history_length=50):
    global _filter
    _filter = YPositionLinearRegressionFilter(sample_time, window_length, history_length)
    return 1


def reset_filter(initial_value=0.0):
    if _filter is None:
        raise RuntimeError("Call initialize_filter before reset_filter")
    _filter.reset(initial_value)
    return 1


def filter_step(measurement):
    if _filter is None:
        raise RuntimeError("Call initialize_filter before filter_step")
    return _filter.update(measurement)
