# 低速平面三自由度 Otter 数值预测，用于命令筛查与回放；保留当前模型阻尼与刚体速度耦合。
"""Low-speed planar Otter prediction for command screening and numeric replay.

The effective inertia is rigid-body plus configured added mass. Rigid-body
surge/sway coupling is retained; configured drag coefficients already carry
their force sign. This follows the current Gazebo plugin, where added-mass
Coriolis is constructed but not applied. It is a deterministic approximation,
not a wave/current model or a guarantee against moving, unobserved obstacles.
"""
from functools import lru_cache
import math

import numpy as np

from .collision import footprint_path_is_clear
from .math_helper import wrap_angle
from .models import AlgorithmStatus, ControlCommand, VesselState


def _clip(value, limit):
    return max(-limit, min(limit, value))


# 组装有效惯性、线性阻尼和二次阻尼，要求惯性对称正定；相同不可变模型复用缓存。
@lru_cache(maxsize=16)
def _model_matrices(model):
    inertia = np.array([
        [model.mass + model.x_dot_u, model.x_dot_v, model.x_dot_r],
        [model.y_dot_u, model.mass + model.y_dot_v, model.y_dot_r],
        [model.n_dot_u, model.n_dot_v, model.i_z + model.n_dot_r]], dtype=float)
    drag = np.array([[model.x_u, model.x_v, model.x_r],
                     [model.y_u, model.y_v, model.y_r],
                     [model.n_u, model.n_v, model.n_r]], dtype=float)
    quadratic = np.array([model.x_u_u, model.y_v_v, model.n_r_r], dtype=float)
    if (not np.isfinite(inertia).all() or not np.isfinite(drag).all()
            or not np.isfinite(quadratic).all() or model.D <= 0 or model.max_force <= 0
            or not math.isfinite(model.D) or not math.isfinite(model.max_force)
            or not np.allclose(inertia, inertia.T)
            or np.min(np.linalg.eigvalsh(inertia)) <= 0):
        raise ValueError("Prediction requires finite drag and positive symmetric effective inertia")
    return np.linalg.inv(inertia), drag, quadratic


# 输入纵向力牛顿与偏航力矩牛顿米，分配左右桨后统一缩放以保留两路比例。
def allocate_force(surge, yaw, model, command_limit=.4):
    """Allocate force in N and moment in N.m with uniform command scaling."""
    if (not all(math.isfinite(v) for v in (surge, yaw, command_limit, model.max_force, model.D))
            or not 0 < command_limit <= 1 or min(model.max_force, model.D) <= 0):
        raise ValueError("Invalid force allocation")
    left = (surge - yaw / model.D) / (2 * model.max_force)
    right = (surge + yaw / model.D) / (2 * model.max_force)
    scale = max(1., abs(left) / command_limit, abs(right) / command_limit)
    return ControlCommand(left / scale, right / scale)


# 使用最大 50 毫秒的 RK4 子步推进带符号船体系 u/v/r 与世界系 x/y/yaw，拒绝侧推输入。
def step_dynamics(state, command, model, dt):
    """Advance signed BODY u/v/r and world pose using RK4, at <= 50 ms steps.

    Lateral actuation is rejected: the current Otter has two longitudinal
    propellers. Frame and simulation timestamp are preserved/advanced.
    """
    if (not state.is_finite() or not command.is_valid() or command.lateral != 0
            or not math.isfinite(dt) or not 0 < dt <= 8):
        raise ValueError("Invalid planar dynamics input")
    inverse, drag, quadratic = _model_matrices(model)
    # 左右桨和产生纵向力，右减左乘力臂产生正偏航力矩，横向执行力固定为零。
    tau = np.array([(command.left + command.right) * model.max_force, 0.,
                    (command.right - command.left) * model.max_force * model.D])
    vector = np.array([state.x, state.y, state.psi, state.u, state.v, state.r], dtype=float)

    # 船体系动力学计算加速度，世界系运动学旋转 u/v 为位置导数；阻尼系数本身携带力的符号。
    def derivative(value):
        _, _, psi, u, v, r = value
        velocity = value[3:]
        forces = tau + drag @ velocity + quadratic * np.abs(velocity) * velocity
        forces += np.array([model.mass * v * r, -model.mass * u * r, 0.])
        acceleration = inverse @ forces
        c, s = math.cos(psi), math.sin(psi)
        return np.array([c * u - s * v, s * u + c * v, r, *acceleration])

    steps = max(1, math.ceil(dt / .05))
    h = dt / steps
    for _ in range(steps):
        k1 = derivative(vector)
        k2 = derivative(vector + .5 * h * k1)
        k3 = derivative(vector + .5 * h * k2)
        k4 = derivative(vector + h * k3)
        vector += h / 6 * (k1 + 2 * k2 + 2 * k3 + k4)
    if not np.isfinite(vector).all():
        raise ValueError("Planar prediction produced nonfinite state")
    vector[2] = wrap_angle(vector[2])
    return VesselState(*vector, stamp=state.stamp + dt, frame_id=state.frame_id)


# 对带符号纵向速度和偏航角速度施加阻尼；此处不限制变化率，发布前仍需安全检查。
def braking_command(state, model, command_limit=.4):
    """Signed PD damping; no slew limit because this is a safety intervention.

    Sway is passively damped by the hull. The caller must run clear_control
    on this command as well before publishing it as an alternative.
    """
    if not state.is_finite():
        return ControlCommand(status=AlgorithmStatus.INVALID_INPUT, message="Invalid state for braking")
    return allocate_force(_clip(-30 * state.u, 20), _clip(-10 * state.r, 8), model, command_limit)


# 依次提供温和及更强制动候选，每个候选都必须分别通过完整停止尾迹的校验。
def braking_candidates(state, model, command_limit=.4):
    """Gentle then stronger signed braking, each requiring clear_control().

    The first candidate is the unchanged 30 N/(m/s), 20 N gentle brake.
    Emergency candidates use gains 60/120 and a 40 N surge request limit;
    yaw damping stays 10 N.m/(rad/s), limited to 8 N.m. Uniform allocation
    keeps each propeller within command_limit (default .4) without side thrust.
    These are safety interventions, so the normal station 20 N request cap
    and ordinary slew limit do not apply. The caller selects the first fully
    validated candidate, or publishes zero if none is clear. Prediction still
    executes the candidate for one tick followed by the original gentle brake;
    it never assumes the stronger brake will be applied indefinitely.
    """
    gentle = braking_command(state, model, command_limit)
    if gentle.status != AlgorithmStatus.SUCCESS:
        return (gentle,)
    yaw = _clip(-10 * state.r, 8)
    return (gentle, *(allocate_force(_clip(-gain * state.u, 40), yaw, model, command_limit)
                      for gain in (60, 120)))


# 先完整执行本拍候选，再逐步重新计算温和制动，返回世界位姿序列及是否在时域内停止。
def predict_control_path(state, command, model, dt, step=.05, horizon=8.):
    """Return (Nx3 world poses, stopped) for one command tick then braking.

    Full tick execution always precedes rest detection. Braking is recomputed
    each step until speed <= .02 m/s and yaw rate <= .01 rad/s. Failure to stop
    inside the total horizon is explicit; callers must fail closed.
    """
    if (not all(math.isfinite(v) and v > 0 for v in (dt, step, horizon))
            or dt > horizon or step > .05 or horizon > 8
            or not state.is_finite() or not command.is_valid() or command.lateral != 0):
        raise ValueError("Invalid control prediction input")
    points, elapsed, predicted = [state.pose], 0., state
    while elapsed < horizon - 1e-10:
        active = elapsed < dt - 1e-10
        # 候选本拍执行完后，速度不超过 0.02 米/秒且角速度不超过 0.01 弧度/秒才判停止。
        if not active and math.hypot(predicted.u, predicted.v) <= .02 and abs(predicted.r) <= .01:
            return np.asarray(points), True
        h = min(step, (dt if active else horizon) - elapsed)
        next_command = command if active else braking_command(predicted, model)
        predicted = step_dynamics(predicted, next_command, model, h)
        points.append(predicted.pose)
        elapsed += h
    stopped = math.hypot(predicted.u, predicted.v) <= .02 and abs(predicted.r) <= .01
    return np.asarray(points), stopped


# 只有预测已停止且含两侧 margin 的完整扫掠船体无碰撞才放行；任何输入或数值异常均拒绝。
def clear_control(state, command, grid, model, dt, length=2.2, width=1.2, margin=.1):
    """Fail-closed full-hull check of execution plus the complete braking tail."""
    try:
        if (command.status != AlgorithmStatus.SUCCESS or grid.frame_id != state.frame_id
                or not all(math.isfinite(v) for v in (length, width, margin))
                or min(length, width) <= 0 or margin < 0):
            return False
        poses, stopped = predict_control_path(state, command, model, dt)
        return stopped and footprint_path_is_clear(
            grid, poses, length + 2 * margin, width + 2 * margin)
    except (AttributeError, TypeError, ValueError, OverflowError, np.linalg.LinAlgError):
        return False
