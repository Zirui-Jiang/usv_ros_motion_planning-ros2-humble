# 跨模块纯数值数据契约：船舶状态、物理参数、只读代价地图、规划结果和归一化推进命令。
"""Numerical contracts. Positions use frame_id; u/v/r are BODY velocities.

Ported from the original USV project by Yang Haodong; see cpp-baseline.
No ROS imports: these values can cross a multiprocessing boundary or be replayed.
"""
from dataclasses import dataclass, field, fields
from enum import Enum
import math
import numpy as np


# 区分成功、无效输入、无路、超时、真实到达与目标区内减速等待。
class AlgorithmStatus(str, Enum):
    SUCCESS = "success"
    INVALID_INPUT = "invalid_input"
    NO_PATH = "no_path"
    TIMEOUT = "timeout"
    GOAL_REACHED = "goal_reached"
    SETTLING = "settling"


# x/y 属于 frame_id 坐标系且单位米；psi 为弧度，u/v 为船体系米/秒，r 为弧度/秒。
@dataclass(frozen=True)
class VesselState:
    x: float = 0.0
    y: float = 0.0
    psi: float = 0.0
    u: float = 0.0
    v: float = 0.0
    r: float = 0.0
    stamp: float = 0.0
    frame_id: str = "otter/map"

    @property
    def pose(self):
        return (self.x, self.y, self.psi)

    # 将船体系纵横速度旋转到世界系；姿态改变不应重复旋转已有船体系测量。
    @property
    def world_velocity(self):
        c, s = math.cos(self.psi), math.sin(self.psi)
        return (c * self.u - s * self.v, s * self.u + c * self.v)

    def is_finite(self):
        return all(math.isfinite(v) for v in (self.x, self.y, self.psi, self.u, self.v, self.r, self.stamp))


# 各船型允许的推进形式：Differential 为两台固定纵向推进器差动；Azimuth 为两台全回转推进器，偏转桨角合成横向力；
# Omni 为两台固定纵向推进器另带一台独立侧推器（WAM-V 的 T 型布局）。
SUPPORTED_VESSELS = {"otter": ("Differential",), "wamv": ("Differential", "Azimuth", "Omni")}
# 能产生横向力的推进形式。
LATERAL_TYPES = ("Azimuth", "Omni")


# 船模参数集合，默认值为 Otter；最大单桨力单位牛顿、质量千克、偏航惯量千克米²、尺寸和力臂为米。
@dataclass(frozen=True)
class USVModel:
    ns: str = "otter"
    name: str = "otter"
    type: str = "Differential"
    cruising_speed: float = 1.0
    max_force: float = 50.0
    mass: float = 29.84
    i_z: float = 10.03
    L: float = 1.35
    W: float = 1.0
    D: float = 0.395
    velocity_min: float = 0.0
    velocity_max: float = 1.0
    acceleration_min: float = -1.0
    acceleration_max: float = 1.0
    jerk_min: float = -2.0
    jerk_max: float = 2.0
    x_dot_u: float = 5.0
    x_dot_v: float = 0.0
    x_dot_r: float = 0.0
    y_dot_u: float = 0.0
    y_dot_v: float = 5.0
    y_dot_r: float = 0.0
    n_dot_u: float = 0.0
    n_dot_v: float = 0.0
    n_dot_r: float = 1.0
    x_u: float = -20.0
    x_v: float = 0.0
    x_r: float = 0.0
    y_u: float = 0.0
    y_v: float = -20.0
    y_r: float = 0.0
    n_u: float = 0.0
    n_v: float = 0.0
    n_r: float = -20.0
    x_u_u: float = 0.0
    y_v_v: float = 0.0
    n_r_r: float = 0.0
    # 归一化侧推命令为 1 时对应的横向力，牛顿，正命令推向左舷；0 表示不能产生横向力。
    # Omni 时是独立侧推器的最大力；Azimuth 时是两台全回转桨合成横向力的归一化基准。
    max_lateral_force: float = 0.0
    # 两台推进器回转轴相对船体转动中心的纵向位置，米，船艉为负；只有 Azimuth 用它补偿横向力带出的转艏力矩。
    thruster_x: float = 0.0

    # 拒绝未知字段、非有限数及不合理物理范围；船型与推进形式须在 SUPPORTED_VESSELS 中，侧推力与推进形式一致。
    @classmethod
    def from_mapping(cls, values):
        names = {f.name for f in fields(cls)}
        unknown = set(values) - names
        if unknown:
            raise ValueError(f"Unknown vessel model fields: {sorted(unknown)}")
        model = cls(**values)
        for f in fields(model):
            value = getattr(model, f.name)
            if f.name not in ("ns", "name", "type"):
                if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
                    raise ValueError(f"Invalid vessel model field: {f.name}")
        if min(model.mass, model.i_z, model.max_force, model.D, model.L, model.W) <= 0:
            raise ValueError("Mass, inertia, force and hull dimensions must be positive")
        if not 0 <= model.cruising_speed <= model.velocity_max or model.velocity_min > model.velocity_max:
            raise ValueError("Invalid vessel velocity limits")
        if model.acceleration_min > 0 or model.acceleration_max <= 0:
            raise ValueError("Acceleration limits must contain zero and allow acceleration")
        if model.type not in SUPPORTED_VESSELS.get(model.name, ()):
            raise ValueError("Supported vessel models: Differential otter; Differential, Azimuth or Omni wamv")
        # A lateral force without the means to produce it, or the reverse, would
        # let the controllers and the simulated vessel disagree about actuation.
        if (model.max_lateral_force > 0) != (model.type in LATERAL_TYPES) or model.max_lateral_force < 0:
            raise ValueError("max_lateral_force must be positive for Azimuth or Omni and zero for Differential")
        return model


# 地图 data 采用 [行y, 列x] 的 uint8 原始代价，分辨率与原点单位米。
@dataclass(frozen=True)
class GridSnapshot:
    data: np.ndarray
    resolution: float
    origin_x: float = 0.0
    origin_y: float = 0.0
    stamp: float = 0.0
    frame_id: str = "otter/map"

    def __post_init__(self):
        a = np.asarray(self.data)
        if a.ndim != 2 or not a.size or a.dtype.kind not in "ui" or np.any(a < 0) or np.any(a > 255):
            raise ValueError("Costmap must be a nonempty 2-D array of integer raw costs 0..255")
        if not all(math.isfinite(v) for v in (self.resolution, self.origin_x, self.origin_y, self.stamp)) or self.resolution <= 0:
            raise ValueError("Invalid costmap metadata")
        a = np.array(a, dtype=np.uint8, copy=True, order="C")
        a.setflags(write=False)
        object.__setattr__(self, "data", a)

    # 跨进程 pickle 恢复时重新调用构造校验，重新建立只读数组，避免工作进程改写障碍。
    def __reduce__(self):
        # NumPy's pickle round-trip does not retain the read-only flag.
        # Re-enter construction at the process boundary to restore validation
        # and an owned, read-only array in the planning worker as well.
        return (type(self), (self.data, self.resolution, self.origin_x,
                            self.origin_y, self.stamp, self.frame_id))

    @property
    def width(self):
        return self.data.shape[1]

    @property
    def height(self):
        return self.data.shape[0]

    # 相对地图原点除以分辨率并向下取整，负向越界不截断回地图。
    def world_to_map(self, x, y):
        if not math.isfinite(x) or not math.isfinite(y):
            return None
        ix = math.floor((x - self.origin_x) / self.resolution)
        iy = math.floor((y - self.origin_y) / self.resolution)
        return (ix, iy) if 0 <= ix < self.width and 0 <= iy < self.height else None

    # 返回栅格单元中心坐标，使用半格偏移而不是左下角。
    def map_to_world(self, ix, iy):
        if not (0 <= ix < self.width and 0 <= iy < self.height):
            raise ValueError("Grid index outside map")
        return (self.origin_x + (ix + 0.5) * self.resolution, self.origin_y + (iy + 0.5) * self.resolution)

    # 越出地图视为阻塞；255 按未知策略处理，其余原始值与阈值比较。
    def is_blocked(self, x, y, threshold=254, unknown_is_obstacle=True):
        cell = self.world_to_map(x, y)
        if cell is None:
            return True
        ix, iy = cell
        cost = int(self.data[iy, ix])
        return unknown_is_obstacle if cost == 255 else cost >= threshold


# 路径为 N×3 位姿数组，携带算法耗时秒和扩展单元数。
@dataclass
class PlanResult:
    status: AlgorithmStatus
    path: np.ndarray = field(default_factory=lambda: np.empty((0, 3), dtype=float))
    message: str = ""
    elapsed: float = 0.0
    expanded: int = 0

    @property
    def success(self):
        return self.status in (AlgorithmStatus.SUCCESS, AlgorithmStatus.GOAL_REACHED)


# 左右桨和侧推均为 [-1,1] 归一化命令；状态和消息供运行层诊断使用。
@dataclass(frozen=True)
class ControlCommand:
    left: float = 0.0
    right: float = 0.0
    lateral: float = 0.0
    status: AlgorithmStatus = AlgorithmStatus.SUCCESS
    message: str = ""

    def is_valid(self):
        return all(math.isfinite(v) and -1.0 <= v <= 1.0 for v in (self.left, self.right, self.lateral))
