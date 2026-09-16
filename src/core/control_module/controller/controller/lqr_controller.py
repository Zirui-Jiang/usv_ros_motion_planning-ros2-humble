# 保留六状态 Otter 离散 LQR 模型，通过有界 Riccati 迭代求双桨反馈增益。
"""Original six-state Otter discrete LQR, solved by Riccati iteration.

State is (x, y, psi, u, v, r), with signed vessel-frame velocities. The
legacy linearization and forward-Euler discretization are retained. NumPy
linear solves replace explicit matrix inverses without changing the equation.
Ported from the original C++ implementation by Yang Haodong (2025).
"""

import math
import time

import numpy as np

from common.math_helper import wrap_angle
from common.models import AlgorithmStatus
from .controller import Controller, select_config, stop, validate_goal_distance_tolerance


# 状态顺序为世界坐标 x/y、航向 psi、船体系 u/v/r；Q 对应六状态，R 对应左右桨输入。
class LQRController(Controller):
    DEFAULTS = {"goal_distance_tolerance": 1.0, "goal_speed_tolerance": 0.25, "lookahead_time": 2.0,
                "discrete_dt": 0.1, "solve_eps": 0.1, "max_iterations": 5000,
                "Q_matrix_diag": [1, 1, 100, 10, 10, 10], "R_matrix_diag": [20, 20],
                "solve_timeout": 0.0}

    def __init__(self, model, config=None):
        super().__init__(model)
        self.config = {**self.DEFAULTS, **select_config(config, "lqr")}
        for key in ["goal_distance_tolerance", "goal_speed_tolerance", "lookahead_time", "discrete_dt", "solve_eps", "solve_timeout"]:
            value = float(self.config[key])
            if not math.isfinite(value) or value < 0 or (key in ["discrete_dt", "solve_eps"] and value == 0):
                raise ValueError(f"invalid {key}")
            self.config[key] = value
        validate_goal_distance_tolerance(self.config["goal_distance_tolerance"])
        count = self.config["max_iterations"]
        if isinstance(count, bool) or not isinstance(count, (int, np.integer)) or count <= 0:
            raise ValueError("max_iterations must be a positive integer")
        q = np.asarray(self.config["Q_matrix_diag"], dtype=float)
        r = np.asarray(self.config["R_matrix_diag"], dtype=float)
        if q.shape != (6,) or r.shape != (2,) or not np.isfinite(q).all() or not np.isfinite(r).all():
            raise ValueError("Q and R diagonals must contain 6 and 2 finite entries")
        if np.any(q < 0) or np.any(r <= 0):
            raise ValueError("Q must be positive semidefinite and R positive definite")
        self.Q, self.R = np.diag(q), np.diag(r)
        parameters = [model.mass, model.i_z, model.x_dot_u, model.y_dot_v, model.n_dot_r,
                      model.x_u, model.y_v, model.n_r, model.D, model.max_force]
        if not np.isfinite(parameters).all():
            raise ValueError("LQR model parameters must be finite")
        if min(model.mass + model.x_dot_u, model.mass + model.y_dot_v, model.i_z + model.n_dot_r, model.D, model.max_force) <= 0:
            raise ValueError("effective masses, thruster arm and maximum force must be positive")
        self.last_iterations = 0
        self.last_residual = math.inf

    def reset(self):
        self.last_iterations = 0
        self.last_residual = math.inf

    # 围绕参考航向和纵向速度线性化，用前向欧拉离散；discrete_dt 独立于本次控制回调 dt。
    def discrete_model(self, reference):
        """Expose the retained A/B model for numerical regression tests."""
        model = self.model
        psi, u = reference.theta, reference.velocity
        if not np.isfinite([psi, u]).all():
            raise ValueError("reference heading and velocity must be finite")
        v = r = 0.0
        mx, my, iz = model.mass + model.x_dot_u, model.mass + model.y_dot_v, model.i_z + model.n_dot_r
        A = np.zeros((6, 6), dtype=float)
        A[0, 2] = -u * math.sin(psi) - v * math.cos(psi)
        A[0, 3], A[0, 4] = math.cos(psi), -math.sin(psi)
        A[1, 2] = u * math.cos(psi) - v * math.sin(psi)
        A[1, 3], A[1, 4] = math.sin(psi), math.cos(psi)
        A[2, 5] = 1.0
        A[3, 3] = model.x_u / mx
        A[3, 5] = (model.mass - model.y_dot_v) / mx
        A[4, 4] = model.y_v / my
        A[4, 5] = -(model.mass - model.x_dot_u) / my
        A[5, 3] = (model.y_dot_v - model.x_dot_u) / iz * v
        A[5, 4] = (model.y_dot_v - model.x_dot_u) / iz * u
        A[5, 5] = model.n_r / iz
        dt = self.config["discrete_dt"]
        A = np.eye(6) + dt * A
        B = np.zeros((6, 2), dtype=float)
        B[3, :] = 1.0 / mx
        B[5, :] = [-model.D / iz, model.D / iz]
        # B 将归一化双桨输入换算为物理力作用，再乘离散时间得到状态增量。
        B *= model.max_force * dt
        if not np.isfinite(A).all() or not np.isfinite(B).all():
            raise ValueError("discretized model is nonfinite")
        return A, B

    # 同时限制迭代次数与可选墙钟耗时，未收敛时抛出超时，禁止使用半成品增益。
    def solve_gain(self, reference):
        """Return K or fail explicitly when a bounded Riccati solve fails."""
        A, B = self.discrete_model(reference)
        P = self.Q.copy()
        started = time.perf_counter()
        self.last_iterations = 0
        self.last_residual = math.inf
        for index in range(self.config["max_iterations"]):
            if self.config["solve_timeout"] and time.perf_counter() - started >= self.config["solve_timeout"]:
                raise TimeoutError("LQR solve exceeded its time budget")
            with np.errstate(over="raise", invalid="raise", divide="raise"):
                hessian = self.R + B.T @ P @ B
                # 解线性方程而非显式求逆，计算离散代数 Riccati 更新中的反馈项。
                feedback = np.linalg.solve(hessian, B.T @ P @ A)
                P_next = self.Q + A.T @ P @ A - A.T @ P @ B @ feedback
                # 相邻 P 矩阵的范数差用于判断收敛，并暴露给数值回归测试。
                residual = float(np.linalg.norm(P_next - P))
            if not np.isfinite(P_next).all() or not math.isfinite(residual):
                raise ValueError("Riccati iteration produced nonfinite values")
            self.last_iterations = index + 1
            self.last_residual = residual
            if residual < self.config["solve_eps"]:
                K = -np.linalg.solve(self.R + B.T @ P_next @ B, B.T @ P_next @ A)
                if not np.isfinite(K).all():
                    raise ValueError("LQR gain is nonfinite")
                return K
            P = P_next
        raise TimeoutError(f"LQR did not converge after {self.last_iterations} iterations (residual={self.last_residual:.6g})")

    # 构造当前减参考的六维误差，航向规整后使用带负号的 K 反馈，异常统一产生零命令。
    def compute(self, state, trajectory, dt):
        reference, failure = self._reference(state, trajectory, dt)
        if failure is not None:
            return failure
        try:
            K = self.solve_gain(reference)
            current = np.array([state.x, state.y, state.psi, state.u, state.v, state.r], dtype=float)
            desired = np.array([reference.x, reference.y, reference.theta, reference.velocity, 0.0, 0.0])
            error = current - desired
            error[2] = wrap_angle(error[2])
            thrust = K @ error
            return self._command(thrust[0], thrust[1])
        except TimeoutError as error:
            return stop(AlgorithmStatus.TIMEOUT, str(error))
        except (ValueError, FloatingPointError, OverflowError, np.linalg.LinAlgError) as error:
            return stop(AlgorithmStatus.INVALID_INPUT, str(error))


LQRControllerOtter = LQRController
