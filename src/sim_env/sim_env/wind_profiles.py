"""Small, ROS-free helpers for scenario-specific wind plugin selection.

The normal Otter xacro contains the legacy zero-wind plugin.  The dedicated
Station Keeping world carries its own wind plugin so that the official
disturbance parameters are visible in the world file.  Both launch stacks use
this module to disable the embedded plugin only for that world; all other
worlds retain the historical launch behaviour.
"""

from pathlib import Path


STATIONKEEPING_WORLD = "stationkeeping_vrx"


def _world_stem(world_name: str) -> str:
    """Return a world basename without an optional ``.world`` suffix."""

    return Path(str(world_name)).name.removesuffix(".world")


def is_stationkeeping_world(world_name: str) -> bool:
    """Whether *world_name* selects the dedicated VRX-inspired scene."""

    return _world_stem(world_name) == STATIONKEEPING_WORLD


def xacro_wind_arguments(world_name: str, robot_namespace: str = "otter") -> tuple[str, ...]:
    """Return xacro substitutions needed to avoid duplicate wind forces.

    The dedicated world owns the non-zero wind plugin.  Suppressing the
    zero-wind plugin embedded in the Otter model prevents duplicate publishers
    on ``/vrx/debug/wind/*`` and duplicate force application.  Legacy worlds
    receive no extra substitution and therefore keep their existing behavior.
    """

    if is_stationkeeping_world(world_name):
        namespace = str(robot_namespace).strip("/")
        if namespace != "otter":
            raise ValueError(
                "stationkeeping_vrx requires robot_namespace=otter because its "
                "Classic wind plugin targets the static model name 'otter'"
            )
        return ("wind_enabled:=false",)
    return ()
