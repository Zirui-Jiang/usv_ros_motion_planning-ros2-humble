# Otter 双环 PID：以平面速度模和 ROS 航向误差计算纵向及转向需求，再分配为左右桨归一化命令。
"""Original Otter speed/course PID and differential thrust allocation.

Gains retain the legacy normalized-command calibration: the allocation is
[[1, 1], [-D, D]], without another max_force division. ``u, v`` already use
the vessel frame. The original speed feedback is hypot(u, v).
Ported from the original C++ implementation by Yang Haodong (2025).
"""

import math

from common.math_helper import wrap_angle
from common.models import AlgorithmStatus
from .controller import Controller, select_config, stop, validate_goal_distance_tolerance


# 保留原工程增益标定；积分变量已经包含积分增益，分配时不再次除以最大物理推力。
class PIDController(Controller):
    DEFAULTS = {"goal_distance_tolerance": 1.0, "goal_speed_tolerance": 0.25, "lookahead_time": 2.6,
                "k_p_speed": 0.0, "k_i_speed": 0.0, "k_d_speed": 0.0,
                "k_p_course": 0.0, "k_i_course": 0.0, "k_d_course": 0.0}

    def __init__(self, model, config=None):
        super().__init__(model)
        if not math.isfinite(model.D) or model.D <= 0:
            raise ValueError("thruster arm D must be finite and positive")
        self.config = dict(self.DEFAULTS)
        self.update_config(config or {})
        self.reset()

    # 先完整验证候选参数再替换配置；在线调参保留积分历史，失败更新不污染当前参数。
    def update_config(self, config):
        """Validate atomically and retain integral/error history for gain tuning."""
        candidate = {**self.config, **select_config(config, "pid")}
        for key in self.DEFAULTS:
            value = float(candidate[key])
            if not math.isfinite(value) or value < 0:
                raise ValueError(f"{key} must be finite and nonnegative")
            candidate[key] = value
        validate_goal_distance_tolerance(candidate["goal_distance_tolerance"])
        self.config = candidate

    # 任务切换时清除速度、航向的积分项与上一拍误差。
    def reset(self):
        self.integral_speed = 0.0
        self.integral_course = 0.0
        self.last_speed_error = 0.0
        self.last_course_error = 0.0

    # dt 为秒，u/v 为船体系带符号速度；速度环使用 hypot(u,v)，航向环使用最短弧误差。
    def compute(self, state, trajectory, dt):
        reference, failure = self._reference(state, trajectory, dt)
        if failure is not None:
            return failure
        cfg = self.config
        speed_error = reference.velocity - math.hypot(state.u, state.v)
        course_error = wrap_angle(reference.theta - state.psi)
        integral_speed = self.integral_speed + cfg["k_i_speed"] * speed_error * dt
        integral_course = self.integral_course + cfg["k_i_course"] * course_error * dt
        surge = (cfg["k_p_speed"] * speed_error + integral_speed
                 + cfg["k_d_speed"] * (speed_error - self.last_speed_error) / dt)
        yaw = (cfg["k_p_course"] * course_error + integral_course
               + cfg["k_d_course"] * wrap_angle(course_error - self.last_course_error) / dt)
        if not all(math.isfinite(v) for v in [integral_speed, integral_course, surge, yaw]):
            return stop(AlgorithmStatus.INVALID_INPUT, "PID arithmetic produced nonfinite values")
        # Preserve legacy course-command regularization for calibrated gains.
        # 保留历史航向控制量的角度规整规则，以兼容当前调好的增益。
        yaw = wrap_angle(yaw)
        # 差动分配满足左右和产生纵向需求、右减左乘力臂 D 产生转向需求。
        command = self._command(0.5 * (surge - yaw / self.model.D),
                                0.5 * (surge + yaw / self.model.D))
        # 仅成功输出才提交积分和误差历史，避免数值异常影响后续控制。
        if command.status == AlgorithmStatus.SUCCESS:
            self.integral_speed = integral_speed
            self.integral_course = integral_course
            self.last_speed_error = speed_error
            self.last_course_error = course_error
        return command


PIDControllerOtter = PIDController
