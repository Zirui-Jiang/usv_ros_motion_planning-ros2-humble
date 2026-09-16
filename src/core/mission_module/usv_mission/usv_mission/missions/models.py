# 任务数据模型：对外接收经纬度任务，对内生成可跨进程传递的局部规划意图。
"""Task requests and planning intents that can cross a process boundary."""
from dataclasses import dataclass
import math
from trajectory_planner.circle import CircleReference


# 纬经度和真航向用度；速度为米/秒，半径与精度为米，持续时间为秒。
@dataclass(frozen=True)
class MissionSpec:
    mode: str
    latitude: float
    longitude: float
    heading_deg: float = 0.
    speed: float = .5
    position_tolerance: float = .5
    watch_radius: float = 10.
    orbit_radius: float = 10.
    clockwise: bool = True
    laps: int = 0
    duration: float = 0.

    # 检查任务类型、数值范围和船速限制；绕圈还必须满足速度/半径不超过最大偏航角速度。
    def validate(self, config=None):
        cfg = config or {}
        if self.mode not in ('standby', 'orbit', 'station_keep'):
            raise ValueError('mode must be standby, orbit or station_keep')
        for name in ('latitude', 'longitude', 'heading_deg', 'speed', 'position_tolerance',
                     'watch_radius', 'orbit_radius', 'duration'):
            value = getattr(self, name)
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
                raise ValueError(f'{name} must be a finite number')
        if not -90. <= self.latitude <= 90. or not -180. <= self.longitude <= 180.:
            raise ValueError('latitude/longitude outside WGS84 degree ranges')
        if type(self.clockwise) is not bool:
            raise ValueError('clockwise must be boolean')
        if type(self.laps) is not int or self.laps < 0 or self.duration < 0:
            raise ValueError('laps must be a nonnegative integer and duration must be nonnegative')
        if min(self.speed, self.position_tolerance, self.watch_radius, self.orbit_radius) <= 0:
            raise ValueError('speed, precision and radii must be positive')
        if not cfg.get('dynamics.velocity_min', 0.) <= self.speed <= cfg.get('dynamics.velocity_max', 1.):
            raise ValueError('speed outside vessel velocity limits')
        if self.position_tolerance >= self.watch_radius:
            raise ValueError('position_tolerance must be less than watch_radius')
        if self.mode == 'orbit':
            if self.position_tolerance >= self.orbit_radius:
                raise ValueError('position_tolerance must be less than orbit_radius')
            if self.speed/self.orbit_radius > cfg.get('usv_simulation.trajectory_planner.los.yaw_rate_max', .3):
                raise ValueError('orbit speed/radius exceeds yaw_rate_max')
        return self


# 保存本轮意图类型、阶段、版本及目标或圆轨迹，候选点由规划进程进一步筛选。
@dataclass(frozen=True)
class MissionIntent:
    kind: str
    phase: str
    revision: int
    spec: MissionSpec
    goal: tuple | None = None
    circle: CircleReference | None = None
    candidates: tuple = ()

    # 修订号、意图类型和目标共同决定参考是否已切换。
    @property
    def signature(self):
        return (self.revision, self.kind, self.goal)
