"""Heading controller used by the Simulink Python block."""

import math


def _wrap_to_pi(angle):
    return math.atan2(math.sin(float(angle)), math.cos(float(angle)))


def _saturate(value, lower, upper):
    return min(max(float(value), float(lower)), float(upper))


class USVHeadingADRCController:
    def __init__(self, sample_time, output_minimum, output_maximum,
                 acceleration_gain=0.0, fixed_Kp=10,
                 fixed_Kd=0.0, fixed_Ki=0.0):
        self.sample_time = float(sample_time)
        if self.sample_time <= 0.0:
            raise ValueError("sample_time must be positive")
        self.fixed_Kp = float(fixed_Kp)
        self.fixed_Kd = float(fixed_Kd)
        self.fixed_Ki = float(fixed_Ki)
        self.acceleration_gain = float(acceleration_gain)
        self.output_minimum = float(output_minimum)
        self.output_maximum = float(output_maximum)
        self.integral_error = 0.0

    def reset(self):
        self.integral_error = 0.0

    def update(self, desire_psi, measured_psi, measured_dot_psi,
               filtered_dot_psi, filtered_ddot_psi, filtered_dot_e,
               e_mode, Kp_psi, Kd_psi, Ki_psi, mode):
        error = _wrap_to_pi(float(measured_psi) - float(desire_psi))
        selector = int(round(float(e_mode)))
        if selector == 1:
            dot_error = float(measured_dot_psi)
        elif selector == 3:
            dot_error = float(filtered_dot_e)
        else:
            dot_error = float(filtered_dot_psi)

        if int(round(float(mode))) == 1:
            kp, kd, ki = float(Kp_psi), float(Kd_psi), float(Ki_psi)
        else:
            kp, kd, ki = self.fixed_Kp, self.fixed_Kd, self.fixed_Ki

        # The supplied acceleration is a measurement, not a desired-acceleration
        # feed-forward command.  Keep its optional gain at zero unless it has been
        # identified for this particular hull and sensor chain.
        acceleration_term = self.acceleration_gain * float(filtered_ddot_psi)
        integral_step = ki * error * self.sample_time
        candidate_integral = _saturate(
            self.integral_error + integral_step, -0.25, 0.25
        )
        candidate_control = -(
            kp * error + kd * dot_error + candidate_integral
        ) + acceleration_term

        # Conditional integration prevents wind-up when the yaw-moment actuator is
        # saturated, while still allowing the integrator to unwind.
        is_saturated_low = candidate_control < self.output_minimum
        is_saturated_high = candidate_control > self.output_maximum
        should_integrate = (
            (not is_saturated_low and not is_saturated_high)
            or (is_saturated_low and integral_step < 0.0)
            or (is_saturated_high and integral_step > 0.0)
        )
        if should_integrate:
            self.integral_error = candidate_integral

        control_moment = -(
            kp * error + kd * dot_error + self.integral_error
        ) + acceleration_term
        return _saturate(control_moment, self.output_minimum, self.output_maximum)


_controller = None


def initialize_controller(sample_time, yaw_moment_min, yaw_moment_max,
                          acceleration_gain=0.0, fixed_Kp=1.0,
                          fixed_Kd=0.6, fixed_Ki=0.01):
    global _controller
    _controller = USVHeadingADRCController(
        sample_time, yaw_moment_min, yaw_moment_max,
        acceleration_gain, fixed_Kp, fixed_Kd, fixed_Ki
    )
    return 1


def reset_controller():
    if _controller is None:
        raise RuntimeError("Call initialize_controller before reset_controller")
    _controller.reset()
    return 1


def controller_step(desire_psi, measured_psi, measured_dot_psi,
                    filtered_dot_psi, filtered_ddot_psi, filtered_dot_e,
                    e_mode, Kp_psi, Kd_psi, Ki_psi, mode):
    if _controller is None:
        raise RuntimeError("Call initialize_controller before controller_step")
    return float(_controller.update(
        desire_psi, measured_psi, measured_dot_psi,
        filtered_dot_psi, filtered_ddot_psi, filtered_dot_e,
        e_mode, Kp_psi, Kd_psi, Ki_psi, mode
    ))
