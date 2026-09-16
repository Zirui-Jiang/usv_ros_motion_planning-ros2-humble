"""Trained neural-network gain scheduler for the Level1 USV heading loop."""

import math


def _clip(value, low, high):
    return min(max(float(value), low), high)


def _wrap_to_pi(angle):
    return math.atan2(math.sin(float(angle)), math.cos(float(angle)))


# A 4-10-3 MLP trained by train_nn_gain_scheduler.py on the validated
# Level1 gain policy. Inputs are absolute heading error, yaw rate, yaw
# acceleration, and a persistent-error state, normalized to [0, 1].
_W1 = (
    (1.39444956, -0.44429459, 0.05493165, -0.01739355),
    (-0.33168740, 0.11020730, -0.04682864, -0.05237668),
    (-0.09576253, -0.17439038, 0.02903563, 0.74344919),
    (-0.25069951, 0.50531032, -0.16596051, -0.19681923),
    (0.09445250, 1.06360020, 0.33011581, -0.01162465),
    (-0.29650286, -0.11247234, 0.15230456, 0.00626890),
    (0.00700958, -0.30501537, -0.09475510, -0.51752421),
    (0.06811599, -0.08153919, 0.23852701, 0.29227707),
    (-0.12217834, 0.28030355, -0.03921166, -0.47564485),
    (-0.58405013, -0.00937452, -0.06257649, -0.14422819),
)
_B1 = (0.20755090, 0.06166178, -0.26185629, -0.42118508, 0.52550374,
       -0.04341321, 0.39425220, 0.17167074, -0.60287145, -0.62682473)
_W2 = (
    (0.80643580, 0.58201188, 0.43314898, 0.50484097, -0.06607048,
     0.18516804, 0.08274335, 0.19620490, -0.13393284, -0.68648622),
    (0.02924882, 0.43861840, -0.01694974, -0.66693096, 0.72074765,
     -0.41983534, -0.75545332, -0.30424789, 0.26377073, 0.35671146),
    (0.02581565, -0.20718870, 0.87918072, 0.30461595, -0.01739800,
     -0.11043874, -0.28593045, -0.04965859, -0.25338697, -0.02456718),
)
_B2 = (-0.10475835, 0.20625503, 0.37351958)
_OUTPUT_MIN = (0.50, 0.08, 0.0)
_OUTPUT_SPAN = (1.50, 0.72, 0.020)
_OUTPUT_MAX = (2.00, 0.80, 0.020)


class NeuralGainScheduler:
    def __init__(self, sample_time):
        self.sample_time = max(float(sample_time), 1e-4)
        self.reset()

    def reset(self):
        self.persistent_error = 0.0

    def update(self, desired_psi, measured_psi, filtered_dot_psi,
               filtered_ddot_psi):
        error = _wrap_to_pi(float(measured_psi) - float(desired_psi))
        error_feature = _clip(abs(error) / 0.50, 0.0, 1.0)
        rate_feature = _clip(abs(float(filtered_dot_psi)) / 0.50, 0.0, 1.0)
        accel_feature = _clip(abs(float(filtered_ddot_psi)) / 1.00, 0.0, 1.0)

        # The persistent state adapts online and is the fourth NN input.
        target = _clip(0.70 * error_feature + 0.30 * rate_feature
                       + 0.20 * accel_feature, 0.0, 1.0)
        learning_rate = 0.05 + 0.20 * error_feature
        self.persistent_error += learning_rate * (target - self.persistent_error)
        self.persistent_error = _clip(self.persistent_error, 0.0, 1.0)

        features = (error_feature, rate_feature, accel_feature,
                    self.persistent_error)
        hidden = tuple(
            math.tanh(sum(weight * value for weight, value in zip(row, features)) + bias)
            for row, bias in zip(_W1, _B1)
        )
        normalized_gains = tuple(
            sum(weight * value for weight, value in zip(row, hidden)) + bias
            for row, bias in zip(_W2, _B2)
        )
        gains = tuple(base + span * normalized
                      for base, span, normalized in zip(
                          _OUTPUT_MIN, _OUTPUT_SPAN, normalized_gains))
        return tuple(_clip(gain, low, high) for gain, low, high in zip(
            gains, _OUTPUT_MIN, _OUTPUT_MAX))


_scheduler = None


def initialize_scheduler(sample_time):
    global _scheduler
    _scheduler = NeuralGainScheduler(sample_time)
    return 1


def reset_scheduler():
    if _scheduler is None:
        raise RuntimeError("Call initialize_scheduler before reset_scheduler")
    _scheduler.reset()
    return 1


def scheduler_step(desired_psi, measured_psi, filtered_dot_psi,
                   filtered_ddot_psi):
    if _scheduler is None:
        raise RuntimeError("Call initialize_scheduler before scheduler_step")
    return _scheduler.update(
        desired_psi, measured_psi, filtered_dot_psi, filtered_ddot_psi
    )
