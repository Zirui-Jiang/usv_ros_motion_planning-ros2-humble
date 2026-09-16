"""Standalone scalar sliding-window linear-regression filter.

Input: one scalar measurement. Outputs: filtered value, first derivative,
and second derivative. Create one Python Code block instance per signal so
each signal has independent filter states and tunable parameters.
"""


class LinearRegressionFilter:
    def __init__(self, sample_time, window_length=10, history_length=50):
        if float(sample_time) <= 0.0:
            raise ValueError("sample_time must be positive")
        self.sample_time = float(sample_time)
        self.window_length = max(2, int(round(float(window_length))))
        self.history_length = max(
            self.window_length, int(round(float(history_length)))
        )
        self.measurement_history = []
        self.rate_history = []
        self.filtered_value = 0.0
        self.first_derivative = 0.0
        self.second_derivative = 0.0

    @staticmethod
    def _fit_latest(values):
        n = len(values)
        if n <= 1:
            return values[0] if n == 1 else 0.0, 0.0

        # 直接使用values，因为values[0]最老，values[n-1]最新
        # 这与C代码中Um数组的排列一致！

        avg_x = (n - 1) / 2.0
        avg_y = sum(values) / n

        sum_xy = 0
        sum_xx = 0

        for i in range(n):
            sum_xy += (values[i] - avg_y) * (i - avg_x)
            sum_xx += (i - avg_x) ** 2

            slope = sum_xy / sum_xx
            intercept = avg_y - slope * avg_x

            # 预测最新点（x = n-1）
            predicted = intercept + slope * (n - 1)

            return predicted, slope

    def reset(self, initial_value=0.0):
        self.measurement_history = [float(initial_value)]
        self.rate_history = [0.0]
        self.filtered_value = float(initial_value)
        self.first_derivative = 0.0
        self.second_derivative = 0.0

    def update(self, measurement):
        self.measurement_history.append(float(measurement))
        self.measurement_history = self.measurement_history[-self.history_length:]
        value_window = self.measurement_history[-self.window_length:]
        self.filtered_value, slope = self._fit_latest(value_window)
        self.first_derivative = slope / self.sample_time

        self.rate_history.append(self.first_derivative)
        self.rate_history = self.rate_history[-self.history_length:]
        rate_window = self.rate_history[-self.window_length:]
        _, rate_slope = self._fit_latest(rate_window)
        self.second_derivative = rate_slope / self.sample_time
        return (
            float(self.filtered_value),
            float(self.first_derivative),
            float(self.second_derivative),
        )


_filter = None


def initialize_filter(sample_time, window_length=10, history_length=50):
    global _filter
    _filter = LinearRegressionFilter(sample_time, window_length, history_length)
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
