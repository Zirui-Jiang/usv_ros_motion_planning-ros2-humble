# 控制器公共契约：验证状态与轨迹，选取随轨迹年龄推进的预瞄点，统一返回归一化双桨命令。
"""Common validation and reference selection for normalized thruster control."""

import math

import numpy as np

from common.models import AlgorithmStatus, ControlCommand


# LOS discards adjacent waypoints separated by at most 1e-9 m. A smaller
# completion disk can demand motion that the reference cannot represent.
MIN_GOAL_DISTANCE_TOLERANCE_M = 1e-9


# 目标距离容差单位为米，必须大于 LOS 去重尺度，保证控制器要求的位置精度可由参考轨迹表达。
def validate_goal_distance_tolerance(value, name="goal_distance_tolerance"):
    value = float(value)
    if not math.isfinite(value) or value <= MIN_GOAL_DISTANCE_TOLERANCE_M:
        raise ValueError(f"{name} must be finite and greater than 1e-9 m "
                         "(LOS waypoint spatial precision)")
    return value


# 兼容完整 controller 配置、指定算法子配置和直接参数字典三种输入层级。
def select_config(config, name):
    values = dict(config or {})
    values = values.get("controller", values)
    return dict(values.get(name + "_controller", values))


# 将失败、等待或完成状态附在零推力命令中，调用方可同时处理执行输出与诊断原因。
def stop(status, message=""):
    return ControlCommand(left=0.0, right=0.0, lateral=0.0, status=status, message=message)


class Controller:
    def __init__(self, model):
        self.model = model

    # 新任务或确认到达时由具体算法清除积分、误差或求解诊断历史。
    def reset(self):
        """Reset persistent algorithm state when starting a new navigation goal."""

    def on_control_hold(self):
        """Notify a non-tracking tick; legacy PID/LQR deliberately retain history."""

    # 先判断真实船舶是否满足终点条件，再按状态时间与预瞄时间从轨迹插值，不能以预测到达代替真实到达。
    def _reference(self, state, trajectory, dt):
        try:
            if not math.isfinite(dt) or dt <= 0:
                return None, stop(AlgorithmStatus.INVALID_INPUT, "control dt must be finite and positive")
            if not np.isfinite([state.x, state.y, state.psi, state.u, state.v, state.r, state.stamp]).all():
                return None, stop(AlgorithmStatus.INVALID_INPUT, "vessel state must be finite")
            if trajectory.empty():
                return None, stop(AlgorithmStatus.NO_PATH, "trajectory is empty")
            if state.frame_id and trajectory.frame_id and state.frame_id != trajectory.frame_id:
                return None, stop(AlgorithmStatus.INVALID_INPUT, "state and trajectory frames differ")
            # A horizon endpoint is not necessarily the final navigation goal.
            if trajectory.goal is not None and math.hypot(state.x - trajectory.goal[0], state.y - trajectory.goal[1]) <= self.config["goal_distance_tolerance"]:
                # Entering the goal disk at speed is not a completed task.
                # The original LQR latched completion while still moving and
                # could coast outside the disk without ever resuming control.
                # Keep the task active during settling so a later drift outside
                # the tolerance can still receive a newly planned reference.
                # 进入目标圆盘但仍有较大平面速度时返回 SETTLING；漂出圆盘后仍可恢复跟踪。
                if math.hypot(state.u, state.v) > self.config["goal_speed_tolerance"]:
                    return None, stop(AlgorithmStatus.SETTLING, "settling within goal tolerance")
                self.reset()
                return None, stop(AlgorithmStatus.GOAL_REACHED)
            # 轨迹 stamp 是生成时使用的状态时间，重复传输不会使旧轨迹重新变新；预瞄时间单位为秒。
            age = max(0.0, state.stamp - trajectory.stamp)
            reference = trajectory.reference_at_time(age + self.config["lookahead_time"])
            if reference.velocity < 0:
                return None, stop(AlgorithmStatus.INVALID_INPUT, "reverse reference velocity is unsupported")
            # Looking ahead to a predicted arrival must not stop the actual
            # vessel before it enters the goal tolerance. Keep preview pose/
            # heading, but use an available positive approach speed until the
            # actual goal check above succeeds. Do not override a commanded
            # intermediate stop away from the navigation goal.
            # 只有预测终点位于最终目标盘内时才恢复接近速度，途中明确设置的停车点保持原意。
            terminal = trajectory[-1]
            if (trajectory.goal is not None and reference.velocity <= 1e-9
                    and terminal.velocity <= 1e-9
                    and math.hypot(terminal.x - trajectory.goal[0], terminal.y - trajectory.goal[1])
                    <= self.config["goal_distance_tolerance"]):
                current_speed = trajectory.reference_at_time(age).velocity
                if current_speed <= 1e-9:
                    positive = np.flatnonzero(trajectory.points[:, 6] > 1e-9)
                    current_speed = float(trajectory.points[positive[-1], 6]) if positive.size else 0.0
                if current_speed > 1e-9:
                    reference = reference._replace(velocity=current_speed)
            return reference, None
        except (AttributeError, TypeError, ValueError, OverflowError) as error:
            return None, stop(AlgorithmStatus.INVALID_INPUT, str(error))

    # 左右桨分别限幅到 [-1, 1]；任何非有限输出都转为 INVALID_INPUT 的零命令。
    @staticmethod
    def _command(left, right):
        if not np.isfinite([left, right]).all():
            return stop(AlgorithmStatus.INVALID_INPUT, "nonfinite controller output")
        return ControlCommand(left=float(np.clip(left, -1.0, 1.0)),
                              right=float(np.clip(right, -1.0, 1.0)), lateral=0.0,
                              status=AlgorithmStatus.SUCCESS)
