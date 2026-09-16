"""将 0914 PI/艏向控制器接入普通导航，输出归一化左右桨命令。

核心只消费新观测，使用相邻状态时间戳的差作为实际采样周期。
节点回调 dt 仍用于公共有效性检查；等待、任务切换或长间隔后重启历史。
"""
import math

from common.dynamics import allocate_force
from common.models import AlgorithmStatus
from .adrc_core import PIController, ADRCController as HeadingController
from .controller import Controller, select_config, stop, validate_goal_distance_tolerance


class ADRCController(Controller):
    DEFAULTS = {
        "goal_distance_tolerance": 1.5, "goal_speed_tolerance": .25,
        "lookahead_time": 2.6, "command_limit": 1., "sample_time": .1,
        "max_sample_gap": .5,
        "k_p_speed": 8., "k_i_speed": .8,
        "force_min": -8., "force_max": 8.,
        "speed_integral_min": -8., "speed_integral_max": 8.,
        "k_p_heading": 1.5, "k_d_heading": .5, "k_i_heading": .03,
        "moment_min": -10., "moment_max": 10.,
        "heading_integral_min": -.25, "heading_integral_max": .25,
        "error_reset_deg": 20., "acceleration_gain": 0.,
        "filter_window_length": 10, "filter_history_length": 50,
    }

    def __init__(self, model, config=None):
        super().__init__(model)
        # 启动即核验双桨力臂与最大推力，不等到首拍才发现无法分配。
        allocate_force(0., 0., model, command_limit=1.)
        cfg = {**self.DEFAULTS, **select_config(config, "adrc")}
        unknown = set(cfg) - set(self.DEFAULTS)
        if unknown:
            raise ValueError(f"Unknown ADRC adapter config: {sorted(unknown)}")
        for key, value in cfg.items():
            if type(value) not in (int, float) or not math.isfinite(value):
                raise ValueError(f"{key} must be finite and numeric")
            if key in ("filter_window_length", "filter_history_length"):
                if type(value) is not int:
                    raise ValueError(f"{key} must be an integer")
            else:
                cfg[key] = float(value)
        validate_goal_distance_tolerance(cfg["goal_distance_tolerance"])
        if cfg["goal_speed_tolerance"] <= 0 or cfg["lookahead_time"] < 0:
            raise ValueError("goal speed tolerance must be positive; lookahead must be nonnegative")
        if cfg["sample_time"] <= 0 or not 0 < cfg["command_limit"] <= 1:
            raise ValueError("sample_time must be positive; command_limit must be in (0, 1]")
        if cfg["max_sample_gap"] <= 0:
            raise ValueError("max_sample_gap must be positive")
        for prefix in ("force", "moment", "speed_integral", "heading_integral"):
            lower, upper = cfg[prefix + "_min"], cfg[prefix + "_max"]
            if not lower <= 0 <= upper or lower >= upper:
                raise ValueError(f"{prefix} limits must be ordered and include zero")
        if not 0 < cfg["error_reset_deg"] <= 180:
            raise ValueError("error_reset_deg must be in (0, 180]")
        if not 2 <= cfg["filter_window_length"] <= cfg["filter_history_length"]:
            raise ValueError("filter history must cover window_length >= 2")
        for key in ("k_p_speed", "k_i_speed", "k_p_heading", "k_d_heading",
                    "k_i_heading", "acceleration_gain"):
            if cfg[key] < 0:
                raise ValueError(f"{key} must be nonnegative")
        self.config = cfg
        # 两个原核心都叫 integral_min/max，适配层用前缀区分两条通道。
        speed_cfg = {key: cfg[key] for key in PIController.DEFAULTS
                     if key not in ("integral_min", "integral_max")}
        heading_cfg = {key: cfg[key] for key in HeadingController.DEFAULTS
                       if key not in ("integral_min", "integral_max")}
        for suffix in ("min", "max"):
            speed_cfg["integral_" + suffix] = cfg["speed_integral_" + suffix]
            heading_cfg["integral_" + suffix] = cfg["heading_integral_" + suffix]
        self.speed_loop = PIController(speed_cfg)
        self.heading_loop = HeadingController(heading_cfg)
        self.reset()

    def reset(self):
        """沿用节点新任务/取消/时钟回退时的复位入口。"""
        self.speed_loop.reset()
        self.heading_loop.reset()
        self.last_sample_stamp = None
        self.last_command = None

    def on_control_hold(self):
        """节点停止正常跟踪时调用；下一有效观测按首拍开始。"""
        self.reset()

    def _allocate(self, surge, yaw, previous_speed_integral, previous_heading_integral):
        command = allocate_force(surge, yaw, self.model,
                                 command_limit=self.config["command_limit"])
        applied_surge = (command.left + command.right) * self.model.max_force
        applied_yaw = (command.right - command.left) * self.model.max_force * self.model.D
        speed_delta = self.speed_loop.integral_speed - previous_speed_integral
        heading_delta = self.heading_loop.integral_heading - previous_heading_integral
        # 共同缩放意味着两通道的实际作用都可能小于请求值。只冻结会继续
        # 推向饱和方向的积分，保留有助退出饱和的变化；艏向积分对力矩带负号。
        changed = False
        if (surge - applied_surge) * speed_delta > 1e-12:
            self.speed_loop.integral_speed = previous_speed_integral
            surge = self.speed_loop.refresh_output()
            changed = True
        if (not self.heading_loop.integral_reset
                and (yaw - applied_yaw) * (-heading_delta) > 1e-12):
            self.heading_loop.integral_heading = previous_heading_integral
            yaw = self.heading_loop.refresh_output()
            changed = True
        if changed:
            command = allocate_force(surge, yaw, self.model,
                                     command_limit=self.config["command_limit"])
        return command

    def compute(self, state, trajectory, dt):
        reference, failure = self._reference(state, trajectory, dt)
        if failure is not None:
            self.reset()
            return failure
        try:
            stamp = float(state.stamp)
            sample_dt = self.config["sample_time"]
            if self.last_sample_stamp is not None:
                elapsed = stamp - self.last_sample_stamp
                if elapsed < 0:
                    raise ValueError("measurement timestamp moved backwards")
                if elapsed == 0:
                    return self.last_command
                if elapsed > self.config["max_sample_gap"] + 1e-9:
                    self.reset()
                else:
                    sample_dt = elapsed
            # psi/theta: rad；u/velocity: m/s，u 为带符号船体纵向速度。
            # 核心内部从艏向估计导数，不消费 state.r。
            previous_speed_integral = self.speed_loop.integral_speed
            previous_heading_integral = self.heading_loop.integral_heading
            inputs = (reference.theta, state.psi, reference.velocity, state.u, sample_dt)
            surge = self.speed_loop.compute(*inputs)       # N
            yaw = self.heading_loop.compute(*inputs, measurement_stamp=stamp)  # N·m
            command = self._allocate(surge, yaw, previous_speed_integral, previous_heading_integral)
            if not command.is_valid():
                raise ValueError("ADRC allocation produced invalid thruster commands")
            self.last_sample_stamp, self.last_command = stamp, command
            return command
        except (ArithmeticError, TypeError, ValueError) as error:
            self.reset()
            return stop(AlgorithmStatus.INVALID_INPUT, str(error))
