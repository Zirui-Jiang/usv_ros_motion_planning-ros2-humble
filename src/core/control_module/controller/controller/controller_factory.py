# 按名称构造 PID、LQR 或 0914 ADRC 控制器；船模能产生横向力且启用横向环时再叠加 0924 横向控制器。
"""Controller selection independent of ROS parameter APIs."""

from common.models import SUPPORTED_VESSELS
from .pid_controller import PIDController
from .lqr_controller import LQRController
from .adrc_controller import ADRCController
from .lateral_thrust import LateralAssistedController, LateralThrust


# lateral 为横向控制器配置段；缺省或未启用时返回的就是原算法对象。
def create_controller(name, model, config=None, lateral=None):
    if model.name not in SUPPORTED_VESSELS:
        raise ValueError(f"unsupported vessel model: {model.name!r}; available: {', '.join(SUPPORTED_VESSELS)}")
    algorithms = {"pid": PIDController, "lqr": LQRController, "adrc": ADRCController}
    try:
        algorithm = algorithms[str(name).lower()]
    except KeyError:
        raise ValueError(f"unsupported controller: {name!r}; available: pid, lqr, adrc") from None
    controller = algorithm(model, config)
    lateral_loop = LateralThrust(model, lateral)
    return LateralAssistedController(controller, lateral_loop) if lateral_loop.enabled else controller
