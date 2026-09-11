"""Numerical contracts. Positions use frame_id; u/v/r are BODY velocities.

Ported from the original USV project by Yang Haodong; see cpp-baseline.
No ROS imports: these values can cross a multiprocessing boundary or be replayed.
"""
from dataclasses import dataclass, field, fields
from enum import Enum
import math
import numpy as np


class AlgorithmStatus(str, Enum):
    SUCCESS = "success"
    INVALID_INPUT = "invalid_input"
    NO_PATH = "no_path"
    TIMEOUT = "timeout"
    GOAL_REACHED = "goal_reached"
    SETTLING = "settling"


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

    @property
    def world_velocity(self):
        c, s = math.cos(self.psi), math.sin(self.psi)
        return (c * self.u - s * self.v, s * self.u + c * self.v)

    def is_finite(self):
        return all(math.isfinite(v) for v in (self.x, self.y, self.psi, self.u, self.v, self.r, self.stamp))


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
        if model.name != "otter" or model.type != "Differential":
            raise ValueError("This release supports the differential Otter model")
        return model


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

    def world_to_map(self, x, y):
        if not math.isfinite(x) or not math.isfinite(y):
            return None
        ix = math.floor((x - self.origin_x) / self.resolution)
        iy = math.floor((y - self.origin_y) / self.resolution)
        return (ix, iy) if 0 <= ix < self.width and 0 <= iy < self.height else None

    def map_to_world(self, ix, iy):
        if not (0 <= ix < self.width and 0 <= iy < self.height):
            raise ValueError("Grid index outside map")
        return (self.origin_x + (ix + 0.5) * self.resolution, self.origin_y + (iy + 0.5) * self.resolution)

    def is_blocked(self, x, y, threshold=254, unknown_is_obstacle=True):
        cell = self.world_to_map(x, y)
        if cell is None:
            return True
        ix, iy = cell
        cost = int(self.data[iy, ix])
        return unknown_is_obstacle if cost == 255 else cost >= threshold


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


@dataclass(frozen=True)
class ControlCommand:
    left: float = 0.0
    right: float = 0.0
    lateral: float = 0.0
    status: AlgorithmStatus = AlgorithmStatus.SUCCESS
    message: str = ""

    def is_valid(self):
        return all(math.isfinite(v) and -1.0 <= v <= 1.0 for v in (self.left, self.right, self.lateral))
