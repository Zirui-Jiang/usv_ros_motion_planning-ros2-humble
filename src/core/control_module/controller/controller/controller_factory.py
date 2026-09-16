# 按名称构造 Otter 的 PID、LQR 或 0914 ADRC 控制器。
"""Otter controller selection independent of ROS parameter APIs."""

from .pid_controller import PIDController
from .lqr_controller import LQRController
from .adrc_controller import ADRCController


def create_controller(name, model, config=None):
    if model.name != "otter":
        raise ValueError(f"unsupported vessel model: {model.name!r}; available: otter")
    algorithms = {"pid": PIDController, "lqr": LQRController, "adrc": ADRCController}
    try:
        algorithm = algorithms[str(name).lower()]
    except KeyError:
        raise ValueError(f"unsupported controller: {name!r}; available: pid, lqr, adrc") from None
    return algorithm(model, config)
