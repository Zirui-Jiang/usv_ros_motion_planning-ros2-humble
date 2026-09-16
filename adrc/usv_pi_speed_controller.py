"""Standalone longitudinal-speed PI controller for a Simulink Python Code block."""

import math


def _saturate(value, lower, upper):
    return min(max(float(value), float(lower)), float(upper))


class USVSpeedPIController:
    def __init__(self, sample_time, proportional_gain, integral_gain,
                 output_minimum, output_maximum, integral_minimum,
                 integral_maximum):
        self.sample_time = float(sample_time)
        if self.sample_time <= 0.0:
            raise ValueError("sample_time must be positive")
        self.proportional_gain = float(proportional_gain)
        self.integral_gain = float(integral_gain)
        self.output_minimum = float(output_minimum)
        self.output_maximum = float(output_maximum)
        self.integral_minimum = float(integral_minimum)
        self.integral_maximum = float(integral_maximum)
        self.integral_state = 0.0
        self.calculated_u = 0.0
        self.desired_surge_force = 0.0

    def reset(self):
        self.integral_state = 0.0
        self.calculated_u = 0.0
        self.desired_surge_force = 0.0

    def update(self, desired_speed, measured_speed, dot_x, dot_y,
               filtered_psi, speed_source):
        # Convert earth-fixed velocity components to the body-frame surge speed.
        xy_speed = (
            math.cos(float(filtered_psi)) * float(dot_x)
            + math.sin(float(filtered_psi)) * float(dot_y)
        )
        self.calculated_u = float(xy_speed)
        if int(round(float(speed_source))) == 1:
            current_speed = float(xy_speed)
        else:
            current_speed = float(measured_speed)

        speed_error = float(desired_speed) - current_speed
        candidate_integral = _saturate(
            self.integral_state + self.sample_time * speed_error,
            self.integral_minimum,
            self.integral_maximum,
        )
        candidate_output = (
            self.proportional_gain * speed_error
            + self.integral_gain * candidate_integral
        )
        upper_saturated = candidate_output > self.output_maximum and speed_error > 0.0
        lower_saturated = candidate_output < self.output_minimum and speed_error < 0.0
        if not (upper_saturated or lower_saturated):
            self.integral_state = candidate_integral

        output = (
            self.proportional_gain * speed_error
            + self.integral_gain * self.integral_state
        )
        self.desired_surge_force = _saturate(
            output, self.output_minimum, self.output_maximum
        )
        return (float(self.desired_surge_force), float(self.calculated_u))


_controller = None


def initialize_controller(sample_time, Kp_u, Ki_u, thrust_min, thrust_max,
                          Iu_min, Iu_max):
    global _controller
    _controller = USVSpeedPIController(
        sample_time, Kp_u, Ki_u, thrust_min, thrust_max, Iu_min, Iu_max
    )
    return 1


def reset_controller():
    if _controller is None:
        raise RuntimeError("Call initialize_controller before reset_controller")
    _controller.reset()
    return 1


def controller_step(desired_speed, measured_speed, dot_x, dot_y,
                    filtered_psi, speed_source):
    if _controller is None:
        raise RuntimeError("Call initialize_controller before controller_step")
    return _controller.update(
        desired_speed, measured_speed, dot_x, dot_y, filtered_psi, speed_source
    )
