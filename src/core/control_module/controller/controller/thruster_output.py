# 航控唯一输出口的执行器映射：把已限幅的归一化左右桨命令还原为力/力矩，交给 0924 推力分配器换算桨角与转速。
"""Convert normalized differential commands to thruster angles and signed rpm.

Mixing and saturation stay in the controllers (``allocate_force`` scales both
propellers uniformly). This stage re-derives each propeller force from the
published command and lets the delivered 0924 allocator convert it to rpm.
Only mode 0 (fixed propellers, angle 0) is used; mode 1 of the delivered
allocator is not enabled, see adrc_core/SOURCE.md.
"""
from typing import NamedTuple

from .adrc_core.thrust_allocator import ThrustAllocator


# 桨角单位弧度、0 指向船艏；转速单位 rpm，正值沿桨角方向推；mode 0 为固定桨。
class ThrusterSetpoint(NamedTuple):
    mode: int = 0
    left_angle: float = 0.0
    left_rpm: float = 0.0
    right_angle: float = 0.0
    right_rpm: float = 0.0


class ThrusterOutput:
    # 默认值沿用 0924 交付代码；桨间距不在此配置，由船模单侧力臂 D 推出。
    DEFAULTS = {
        "water_density": 1000.0, "propeller_diameter": 0.259, "thrust_coefficient": 0.5,
        "left_rpm_min": -5000.0, "left_rpm_max": 5000.0,
        "right_rpm_min": -5000.0, "right_rpm_max": 5000.0,
    }

    # 启动即核验：转速上下限须允许正反转，且各自对应的推力不低于单桨最大推力 max_force。
    def __init__(self, model, config=None):
        supplied = dict(config or {})
        unknown = set(supplied) - set(self.DEFAULTS)
        if unknown:
            raise ValueError(f"Unknown thrust allocator config: {sorted(unknown)}")
        cfg = {**self.DEFAULTS, **supplied}
        # The allocator takes the full spacing; the model stores the one-sided
        # arm D. Deriving it avoids configuring the same geometry twice.
        self.allocator = ThrustAllocator({**cfg, "propeller_spacing": 2.0 * float(model.D)})
        self.max_force, self.arm = float(model.max_force), float(model.D)
        for side in ("left", "right"):
            lower, upper = self.allocator.config[side + "_rpm_min"], self.allocator.config[side + "_rpm_max"]
            if not lower < 0.0 < upper:
                raise ValueError(f"{side} rpm limits must allow forward and reverse thrust")
            # A limit below max_force would clip one propeller independently
            # and silently change the requested yaw moment.
            reach = min(-self.allocator._rpm_to_force(lower), self.allocator._rpm_to_force(upper))
            if reach < self.max_force * (1.0 - 1e-9):
                raise ValueError(f"{side} rpm limits reach only {reach:.3f} N, "
                                 f"below dynamics.max_force {self.max_force:.3f} N")

    # 由归一化命令还原纵向力和转艏力矩后按 mode 0 换算；侧推须为零，全回转横移尚未启用。
    def compute(self, command):
        if not command.is_valid() or command.lateral != 0.0:
            raise ValueError("Thruster output requires a valid command without lateral thrust")
        surge = (command.left + command.right) * self.max_force
        yaw = (command.right - command.left) * self.max_force * self.arm
        return ThrusterSetpoint(0, *self.allocator.compute(surge, yaw, mode=0))
