"""Otter controller selection independent of ROS parameter APIs."""

from .pid_controller import PIDController
from .lqr_controller import LQRController


def create_controller(name, model, config=None):
    if model.name != "otter":
        raise ValueError(f"unsupported vessel model: {model.name!r}; available: otter")
    algorithms = {"pid": PIDController, "lqr": LQRController}
    try:
        algorithm = algorithms[str(name).lower()]
    except KeyError:
        raise ValueError(f"unsupported controller: {name!r}; available: pid, lqr") from None
    return algorithm(model, config)
