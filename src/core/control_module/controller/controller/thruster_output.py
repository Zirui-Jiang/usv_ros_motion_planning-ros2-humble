# 航控唯一输出口的执行器映射：把已限幅的归一化命令还原为力/力矩，换算成每台推进器的桨角与转速。
"""Convert normalized commands to thruster angles and signed rpm.

Mixing and saturation of the two longitudinal commands stay in the controllers
(``allocate_force`` scales both propellers uniformly). This stage re-derives
surge force and yaw moment from the published command.

Fixed propellers (Differential, Omni) use mode 0 of the delivered 0924
allocator: angle 0, signed rpm. A separate lateral thruster (Omni) has no field
in this setpoint, so only the two longitudinal propellers are converted and the
lateral command travels on its own normalized topic.

Two full-rotatable thrusters (Azimuth) share the lateral force whenever the
lateral command is nonzero, and the setpoint is marked mode 1. The geometry of
the delivered mode 1 is not used, see adrc_core/SOURCE.md: it splits the lateral
force equally, so a thruster whose longitudinal thrust changes sign has to swing
through a quarter turn or more while still producing thrust. Here
- the differential longitudinal thrust also cancels the yaw moment that a
  lateral force at the stern produces, so surge force, lateral force and yaw
  moment on the hull are the requested ones;
- each thruster takes a share of the lateral force proportional to the size of
  its longitudinal thrust. Both deflections then have the same size, and a
  thruster changes the side of its deflection only while its thrust is zero;
- the deflection is limited to ``max_deflection``. If the lateral force would
  need more, the lateral force is reduced; surge force and yaw moment are kept;
- all three requests are scaled together when a thruster would exceed
  max_force;
- thrust and rpm are related by the delivered allocator's propeller model.
Angles are in the ROS body frame: 0 toward the bow, positive toward port.
"""
import math
from typing import NamedTuple

from .adrc_core.thrust_allocator import ThrustAllocator


# 桨角单位弧度、0 指向船艏、推力方向转向左舷为正；转速单位 rpm，正值沿桨角方向推。
# mode 0 为固定桨角（恒为 0），mode 1 为全回转桨偏转桨角合成横向力。
class ThrusterSetpoint(NamedTuple):
    mode: int = 0
    left_angle: float = 0.0
    left_rpm: float = 0.0
    right_angle: float = 0.0
    right_rpm: float = 0.0


class ThrusterOutput:
    # 默认值沿用 0924 交付代码；桨间距不在此配置，由船模单侧力臂 D 推出。
    # max_deflection 是全回转桨偏离船艏方向的最大桨角（弧度），不属于交付分配器的配置。
    DEFAULTS = {
        "water_density": 1000.0, "propeller_diameter": 0.259, "thrust_coefficient": 0.5,
        "left_rpm_min": -5000.0, "left_rpm_max": 5000.0,
        "right_rpm_min": -5000.0, "right_rpm_max": 5000.0,
        "max_deflection": math.pi / 4,
    }

    # 启动即核验：转速上下限须允许正反转，且各自对应的推力不低于单桨最大推力 max_force。
    def __init__(self, model, config=None):
        supplied = dict(config or {})
        unknown = set(supplied) - set(self.DEFAULTS)
        if unknown:
            raise ValueError(f"Unknown thrust allocator config: {sorted(unknown)}")
        cfg = {**self.DEFAULTS, **supplied}
        self.max_deflection = cfg.pop("max_deflection")
        if (type(self.max_deflection) not in (int, float) or not math.isfinite(self.max_deflection)
                or not 0.0 < self.max_deflection < math.pi / 2):
            raise ValueError("max_deflection must lie strictly between 0 and pi/2")
        # The allocator takes the full spacing; the model stores the one-sided
        # arm D. Deriving it avoids configuring the same geometry twice.
        self.allocator = ThrustAllocator({**cfg, "propeller_spacing": 2.0 * float(model.D)})
        self.max_force, self.arm = float(model.max_force), float(model.D)
        # 只有能产生横向力的船模才允许命令里出现侧推分量；全回转桨由本级把它分给两台桨，独立侧推器不进入本指令。
        self.has_lateral = float(model.max_lateral_force) > 0.0
        self.azimuth = model.type == "Azimuth"
        self.max_lateral, self.thruster_x = float(model.max_lateral_force), float(model.thruster_x)
        self.reset()
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

    # 推力为零的桨保持上一拍桨角；起点为两台桨都指向船艏。
    def reset(self):
        self._angles = [0.0, 0.0]

    # 由归一化命令还原纵向力和转艏力矩后换算。船模不能产生横向力时侧推须为零；
    # 固定桨和独立侧推器只换算左右桨（mode 0）；全回转桨在侧推非零时把横向力一并分给两台桨（mode 1）。
    def compute(self, command):
        if not command.is_valid() or (command.lateral != 0.0 and not self.has_lateral):
            raise ValueError("Thruster output requires a valid command; "
                             "lateral thrust needs a vessel that can produce lateral force")
        surge = (command.left + command.right) * self.max_force
        yaw = (command.right - command.left) * self.max_force * self.arm
        if not self.azimuth or command.lateral == 0.0:
            self.reset()
            return ThrusterSetpoint(0, *self.allocator.compute(surge, yaw, mode=0))
        return self._vectored(surge, yaw, command.lateral * self.max_lateral)

    # 两台桨的纵向推力（左, 右）：和为纵向力；差既给出转艏力矩，又抵消横向力在回转轴处带出的力矩 thruster_x × 横向力。
    def _longitudinal(self, surge, yaw, lateral):
        differential = yaw - self.thruster_x * lateral
        return .5 * surge - .5 * differential / self.arm, .5 * surge + .5 * differential / self.arm

    # 全回转分配。纵向力、横向力（左舷为正）和转艏力矩（逆时针为正）都是对船体的总需求。
    def _vectored(self, surge, yaw, lateral):
        limit = math.tan(self.max_deflection)

        # 两台桨的偏转角大小相同，正切等于 横向力 / 两桨纵向推力大小之和。
        def feasible(share):
            return abs(share * lateral) <= sum(map(abs, self._longitudinal(surge, yaw, share * lateral))) * limit + 1e-12

        # 桨角超限时只减小横向力，纵向力和转艏力矩照给。零横向力总是可行，二分找出可行的最大比例。
        if not feasible(1.0):
            low, high = 0.0, 1.0
            for _ in range(48):
                middle = .5 * (low + high)
                low, high = (middle, high) if feasible(middle) else (low, middle)
            lateral *= low
        longitudinal = self._longitudinal(surge, yaw, lateral)
        total = sum(map(abs, longitudinal))
        # 横向力按纵向推力大小分给两台桨：纵向推力过零的那台此时也不出横向力，换侧时推力为零。
        forces = [(value, lateral * abs(value) / total if total > 0.0 else 0.0) for value in longitudinal]
        # 任一台桨的合力超过 max_force 时整体同比缩小，保持合力方向。
        scale = max(1.0, *(math.hypot(*force) / self.max_force for force in forces))
        setpoint = []
        for index, (side, (value, sideways)) in enumerate(zip(("left", "right"), forces)):
            thrust = math.copysign(math.hypot(value, sideways), value) / scale
            rpm = self.allocator._force_to_rpm(thrust)
            if rpm == 0.0:
                setpoint += [self._angles[index], 0.0]
                continue
            # 倒推用负转速，桨角仍在船艏一侧：桨角正切 = 横向分量 / 纵向分量。
            self._angles[index] = math.atan(sideways / value)
            limits = self.allocator.config
            setpoint += [self._angles[index], min(max(rpm, limits[side + "_rpm_min"]), limits[side + "_rpm_max"])]
        return ThrusterSetpoint(1, *setpoint)

    # 把桨角/转速指令里的转速换回归一化推力命令（推力 / max_force，沿桨角方向为正），供仿真推进器使用。
    def normalized(self, setpoint):
        return tuple(self.allocator._rpm_to_force(rpm) / self.max_force
                     for rpm in (setpoint.left_rpm, setpoint.right_rpm))
