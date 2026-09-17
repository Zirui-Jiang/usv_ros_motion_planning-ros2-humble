"""Immutable circle reference in the local Cartesian map frame."""
from dataclasses import dataclass
import math


@dataclass(frozen=True)
class CircleReference:
    center: tuple
    radius: float
    speed: float
    clockwise: bool = True

    def __post_init__(self):
        if (len(self.center) != 2 or not all(math.isfinite(v) for v in self.center)
                or not all(math.isfinite(v) and v > 0 for v in (self.radius, self.speed))
                or type(self.clockwise) is not bool):
            raise ValueError('circle requires finite center, positive radius/speed and boolean direction')
        object.__setattr__(self, 'center', tuple(float(v) for v in self.center))

    @property
    def direction(self):
        return -1. if self.clockwise else 1.

    def pose_at(self, angle):
        return (self.center[0]+self.radius*math.cos(angle),
                self.center[1]+self.radius*math.sin(angle),
                (angle+self.direction*math.pi/2+math.pi) % (2*math.pi)-math.pi)
