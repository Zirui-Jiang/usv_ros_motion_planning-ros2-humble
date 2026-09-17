"""WGS84 geodetic coordinates and a Gazebo ENU reference (no ROS dependency).

Public headings are degrees clockwise from true north; ROS yaw is radians
counterclockwise from local east. Coordinates are metres in the world frame.
"""
from dataclasses import dataclass
import math
from pathlib import Path
import xml.etree.ElementTree as ET


WGS84_A = 6378137.0
WGS84_F = 1.0 / 298.257223563
WGS84_E2 = WGS84_F * (2.0 - WGS84_F)


def _finite(*values):
    if any(type(v) not in (int, float) or not math.isfinite(v) for v in values):
        raise ValueError("Coordinates must be finite numbers")


def _geodetic(latitude, longitude, altitude):
    _finite(latitude, longitude, altitude)
    if not -90 <= latitude <= 90 or not -180 <= longitude <= 180:
        raise ValueError("Latitude/longitude outside WGS84 degree ranges")


def heading_to_yaw(degrees):
    """Convert a finite true-north clockwise heading to ROS yaw [-pi, pi)."""
    _finite(degrees)
    return (math.pi / 2.0 - math.radians(degrees) + math.pi) % (2 * math.pi) - math.pi


def _ecef(latitude, longitude, altitude):
    lat, lon = math.radians(latitude), math.radians(longitude)
    sin_lat, cos_lat = math.sin(lat), math.cos(lat)
    radius = WGS84_A / math.sqrt(1.0 - WGS84_E2 * sin_lat * sin_lat)
    return ((radius + altitude) * cos_lat * math.cos(lon),
            (radius + altitude) * cos_lat * math.sin(lon),
            (radius * (1.0 - WGS84_E2) + altitude) * sin_lat)


@dataclass(frozen=True)
class GeoReference:
    latitude: float
    longitude: float
    altitude: float = 0.0

    def __post_init__(self):
        _geodetic(self.latitude, self.longitude, self.altitude)

    @classmethod
    def from_world(cls, path):
        """Read the actual SDF world; reject absent or unsupported references."""
        try:
            root = ET.parse(Path(path)).getroot()
        except (OSError, ET.ParseError) as error:
            raise ValueError(f"Cannot read world geographic reference: {error}") from error
        worlds = [root] if root.tag == "world" else root.findall("world")
        if len(worlds) != 1:
            raise ValueError("World must contain exactly one geographic reference")
        node = worlds[0].find("spherical_coordinates")
        if node is None:
            raise ValueError("World has no spherical_coordinates geographic reference")

        def required(name):
            text = node.findtext(name)
            if text is None or not text.strip():
                raise ValueError(f"World geographic reference is missing {name}")
            return text.strip()

        if required("surface_model") != "EARTH_WGS84":
            raise ValueError("Only EARTH_WGS84 world coordinates are supported")
        if required("world_frame_orientation") != "ENU":
            raise ValueError("Only ENU world coordinates are supported")
        heading = float(required("heading_deg"))
        _finite(heading)
        if abs(heading) > 1e-9:
            raise ValueError("Only zero heading_deg world coordinates are supported")
        return cls(float(required("latitude_deg")), float(required("longitude_deg")),
                   float(required("elevation")))

    def _basis(self):
        lat, lon = math.radians(self.latitude), math.radians(self.longitude)
        slat, clat, slon, clon = math.sin(lat), math.cos(lat), math.sin(lon), math.cos(lon)
        return ((-slon, clon, 0.0), (-slat * clon, -slat * slon, clat),
                (clat * clon, clat * slon, slat))

    def to_local(self, latitude, longitude, alt=0.0):
        _geodetic(latitude, longitude, alt)
        origin = _ecef(self.latitude, self.longitude, self.altitude)
        point = _ecef(latitude, longitude, alt)
        delta = tuple(p - o for p, o in zip(point, origin))
        return tuple(sum(a * b for a, b in zip(axis, delta)) for axis in self._basis())

    def to_geographic(self, x, y, z=0.0):
        _finite(x, y, z)
        origin = _ecef(self.latitude, self.longitude, self.altitude)
        basis = self._basis()
        px, py, pz = tuple(origin[i] + x * basis[0][i] + y * basis[1][i] + z * basis[2][i]
                           for i in range(3))
        horizontal = math.hypot(px, py)
        if horizontal < 1e-9:
            if abs(pz) < 1e-9:
                raise ValueError("The ECEF centre has no geographic coordinates")
            return (math.copysign(90.0, pz), self.longitude,
                    abs(pz) - WGS84_A * (1.0 - WGS84_F))
        lat = math.atan2(pz, horizontal * (1.0 - WGS84_E2))
        for _ in range(15):
            sin_lat = math.sin(lat)
            radius = WGS84_A / math.sqrt(1.0 - WGS84_E2 * sin_lat * sin_lat)
            updated = math.atan2(pz + WGS84_E2 * radius * sin_lat, horizontal)
            if abs(updated - lat) < 1e-14:
                lat = updated
                break
            lat = updated
        sin_lat, cos_lat = math.sin(lat), math.cos(lat)
        radius = WGS84_A / math.sqrt(1.0 - WGS84_E2 * sin_lat * sin_lat)
        # Projection avoids loss of accuracy near the poles.
        altitude = horizontal * cos_lat + pz * sin_lat - radius * (1.0 - WGS84_E2 * sin_lat * sin_lat)
        return (math.degrees(lat), math.degrees(math.atan2(py, px)), altitude)
