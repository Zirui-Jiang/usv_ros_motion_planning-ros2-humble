"""0924 横向控制器接入契约：交付文件未改、符号换算、限幅、采样时间、航线最近点及叠加外壳的生命周期。"""
import hashlib
import math
from pathlib import Path

import pytest

from common.models import AlgorithmStatus, ControlCommand, USVModel, VesselState
from controller import ADRCController, PIDController, create_controller
from controller.lateral_thrust import LateralAssistedController, LateralThrust, closest_route_point
from trajectory_planner import Trajectory
from trajectory_planner.trajectory import TrajectoryCell

DELIVERED = Path(__file__).resolve().parents[1] / "controller" / "adrc_core" / "lateral_controller.py"
WAMV = USVModel(name="wamv", type="Omni", max_force=250., max_lateral_force=250., mass=251.28,
                i_z=500.5, L=4.9, W=2.4, D=1.027, velocity_max=1.5, x_dot_u=0., y_dot_v=0.,
                n_dot_r=0., x_u=-51.3, y_v=-40., n_r=-400., x_u_u=-72.4)
# 只留比例项，便于按 力 = 增益 × 偏差 手算期望值。
PROPORTIONAL = {"enabled": True, "k_p_lateral": 60., "k_d_lateral": 0., "k_i_lateral": 0.,
                "force_min": -100., "force_max": 100.}


def point(x=0., y=0.):
    return TrajectoryCell(x=x, y=y)


# 沿 +x 的直线参考，0.5 m/s；缺省附带同一条直线作航线，route=() 表示参考不带航线。
def reference(goal=None, route=((0., 0.), (5., 0.))):
    return Trajectory([[0, 0, 0, 0, 0, 0, .5, 0], [10, 5, 5, 0, 0, 0, .5, 0]],
                      frame_id="otter/map", goal=goal, route=route)


# 记录调用并按预设状态返回左右桨命令的内层控制器替身。
class Inner:
    def __init__(self, status=AlgorithmStatus.SUCCESS):
        self.config = {"lookahead_time": 2.6}
        self.status, self.calls = status, []

    def reset(self):
        self.calls.append("reset")

    def on_control_hold(self):
        self.calls.append("hold")

    def update_config(self, config):
        self.calls.append(("update", config))

    def compute(self, state, trajectory, dt):
        return ControlCommand(left=.2, right=.3, status=self.status)


# 交付文件必须原样保留；按 LF 计算，避免 Windows 检出换行改变哈希。
def test_delivered_lateral_controller_is_unmodified():
    digest = hashlib.sha256(DELIVERED.read_bytes().replace(b"\r\n", b"\n")).hexdigest()
    assert digest == "e58846243d258b967564abdc192f902f418385c0427d83224d24f86da60de6e4"


# 默认关闭：任何偏差都不出侧推，工厂也不给控制器套外壳。
def test_disabled_by_default_outputs_no_lateral_thrust():
    loop = LateralThrust(USVModel())
    assert not loop.enabled
    assert loop.compute(VesselState(), point(y=3.)) == 0.
    assert isinstance(create_controller("adrc", USVModel()), ADRCController)
    assert isinstance(create_controller("adrc", WAMV, None, {"enabled": False}), ADRCController)


@pytest.mark.parametrize("model, config, message", [
    (USVModel(), {"enabled": True}, "requires a vessel that can produce lateral force"),
    (WAMV, {"enabled": True, "force_max": 251.}, "stay within"),
    (WAMV, {"enabled": True, "force_min": -251.}, "stay within"),
    (WAMV, {"enabled": True, "force_min": 1., "force_max": 6.}, "contain zero"),
    (WAMV, {"enabled": 1}, "must be a bool"),
    (WAMV, {"enabled": True, "max_sample_gap": 0.}, "max_sample_gap"),
    (WAMV, {"k_p_lateral": -1.}, "k_p_lateral must be nonnegative"),
    (WAMV, {"k_p_lateral": "4"}, "must be numeric"),
    (WAMV, {"mode": 1}, "Unknown lateral controller config"),
])
def test_startup_rejects_inconsistent_configuration(model, config, message):
    with pytest.raises(ValueError, match=message):
        LateralThrust(model, config)


# 参考点在左舷（艇体系 +y）时侧推命令为正，右舷为负；艏向旋转后仍按艇体系判断。
@pytest.mark.parametrize("state, target, expected", [
    (VesselState(), point(x=4., y=.5), .12),
    (VesselState(), point(x=4., y=-.5), -.12),
    (VesselState(psi=math.pi / 2), point(x=-.5, y=4.), .12),
    (VesselState(x=10., y=-2., psi=math.pi), point(x=6., y=-1.5), -.12),
])
def test_port_side_reference_gives_positive_normalized_command(state, target, expected):
    # 60 N/m × 0.5 m = 30 N，除以 250 N 得 0.12。
    assert LateralThrust(WAMV, PROPORTIONAL).compute(state, target) == pytest.approx(expected)


# 死区内不出力；超过限幅时按 force_max / max_lateral_force 饱和。
def test_deadband_and_force_saturation():
    loop = LateralThrust(WAMV, PROPORTIONAL)
    assert loop.compute(VesselState(), point(y=.01)) == 0.
    loop.reset()
    assert loop.compute(VesselState(), point(y=1.9)) == pytest.approx(.4)
    loop.reset()
    assert loop.compute(VesselState(), point(y=-1.9)) == pytest.approx(-.4)


# 微分项用相邻观测时间戳之差；重复观测返回上一拍结果，间隔过长按首拍处理，时间倒退报错。
def test_derivative_uses_measurement_stamps_and_gaps_restart_history():
    config = {**PROPORTIONAL, "k_p_lateral": 0., "k_d_lateral": 40., "max_sample_gap": .5}
    loop = LateralThrust(WAMV, config)
    assert loop.compute(VesselState(stamp=1.), point()) == 0.
    # 偏差 0 → 0.2 m 用时 0.2 s：交付约定下误差变化率 -1 m/s，力 -40 N（向左舷），命令 +0.16。
    moved = loop.compute(VesselState(stamp=1.2), point(y=.2))
    assert moved == pytest.approx(.16)
    assert loop.compute(VesselState(stamp=1.2), point(y=5.)) == moved
    # 超过 0.5 s 的间隔清除历史，首拍没有误差差分。
    assert loop.compute(VesselState(stamp=2.), point(y=.4)) == 0.
    with pytest.raises(ValueError, match="moved backwards"):
        loop.compute(VesselState(stamp=1.9), point())


# 外壳只填侧推分量，不改左右桨；艇在航线右侧 0.5 m，航线在左舷，侧推为正。
def test_wrapper_adds_lateral_without_touching_propeller_commands():
    controller = LateralAssistedController(Inner(), LateralThrust(WAMV, PROPORTIONAL))
    result = controller.compute(VesselState(y=-.5), reference(), .1)
    assert (result.left, result.right, result.status) == (.2, .3, AlgorithmStatus.SUCCESS)
    assert result.lateral == pytest.approx(.12)
    assert result.is_valid()


# 到达、减速等待或内层失败时不出侧推，并清除横向环历史。
@pytest.mark.parametrize("status", [AlgorithmStatus.GOAL_REACHED, AlgorithmStatus.SETTLING,
                                    AlgorithmStatus.INVALID_INPUT, AlgorithmStatus.NO_PATH])
def test_wrapper_keeps_zero_lateral_when_inner_is_not_tracking(status):
    lateral = LateralThrust(WAMV, PROPORTIONAL)
    lateral.compute(VesselState(stamp=1.), point(y=.5))
    result = LateralAssistedController(Inner(status), lateral).compute(VesselState(y=-.5, stamp=1.1), reference(), .1)
    assert result.lateral == 0. and result.status == status
    assert lateral.last_stamp is None


# 复位、暂停与在线调参转发给内层；横向环在复位和暂停时都从首拍重新开始。
def test_wrapper_forwards_lifecycle_and_configuration():
    inner, lateral = Inner(), LateralThrust(WAMV, PROPORTIONAL)
    controller = LateralAssistedController(inner, lateral)
    for action in (controller.reset, controller.on_control_hold):
        lateral.compute(VesselState(stamp=1.), point(y=.5))
        action()
        assert lateral.last_stamp is None
    controller.update_config({"k_p_speed": 1.})
    assert inner.calls == ["reset", "hold", ("update", {"k_p_speed": 1.})]
    assert controller.config is inner.config


# 横向环出错时整条命令归零，而不是带着左右桨命令继续。
def test_wrapper_fails_closed_on_lateral_error():
    lateral = LateralThrust(WAMV, PROPORTIONAL)
    controller = LateralAssistedController(Inner(), lateral)
    controller.compute(VesselState(y=-.5, stamp=2.), reference(), .1)
    result = controller.compute(VesselState(y=-.5, stamp=1.), reference(), .1)
    assert result.status == AlgorithmStatus.INVALID_INPUT
    assert (result.left, result.right, result.lateral) == (0., 0., 0.)


# 折线最近点：落在线段内取垂足，越过端点取端点，拐角处取较近的一段；单点与空航线分别返回该点和 None。
@pytest.mark.parametrize("route, position, expected", [
    ([[0., 0.], [10., 0.]], (4., 3.), (4., 0.)),
    ([[0., 0.], [10., 0.]], (-2., 3.), (0., 0.)),
    ([[0., 0.], [10., 0.]], (12., -1.), (10., 0.)),
    ([[0., 0.], [10., 0.], [10., 10.]], (9., 6.), (10., 6.)),
    ([[0., 0.], [10., 0.], [10., 10.]], (6., 1.), (6., 0.)),
    ([[0., 0.], [0., 0.], [10., 0.]], (3., 2.), (3., 0.)),
    ([[7., -2.]], (0., 0.), (7., -2.)),
])
def test_closest_route_point_projects_onto_the_polyline(route, position, expected):
    assert tuple(closest_route_point(route, *position)) == pytest.approx(expected)


def test_closest_route_point_handles_missing_or_invalid_routes():
    assert closest_route_point([], 0., 0.) is None
    assert closest_route_point(reference(route=()).route, 0., 0.) is None
    with pytest.raises(ValueError):
        closest_route_point([[0., 0.], [math.nan, 1.]], 0., 0.)
    with pytest.raises(ValueError):
        closest_route_point([[0., 0.], [1., 1.]], math.inf, 0.)


# 横向偏差取自航线而不是轨迹：轨迹经过艇位、航线在左舷 2 m 处时仍按航线出力（60 × 1.5 = 90 N）。
def test_wrapper_measures_offset_from_route_not_from_trajectory():
    controller = LateralAssistedController(Inner(), LateralThrust(WAMV, PROPORTIONAL))
    result = controller.compute(VesselState(), reference(route=((-5., 1.5), (5., 1.5))), .1)
    assert result.lateral == pytest.approx(.36)
    # 航线与艏向垂直时只取艇体横向分量：最近点在正前方 2 m，横向偏差为零。
    controller.reset()
    ahead = controller.compute(VesselState(stamp=1.), reference(route=((2., -5.), (2., 5.))), .1)
    assert ahead.lateral == 0.


# 参考不带航线（如多航点航路之外的旧消息）时不出侧推，并清除横向环历史。
def test_wrapper_gives_no_lateral_thrust_without_a_route():
    lateral = LateralThrust(WAMV, PROPORTIONAL)
    controller = LateralAssistedController(Inner(), lateral)
    assert controller.compute(VesselState(y=-.5, stamp=1.), reference(), .1).lateral == pytest.approx(.12)
    result = controller.compute(VesselState(y=-.5, stamp=1.1), reference(route=()), .1)
    assert (result.left, result.right, result.lateral) == (.2, .3, 0.)
    assert result.status == AlgorithmStatus.SUCCESS and lateral.last_stamp is None


# 工厂对三种航向/航速算法都能叠加横向环；不支持的船型仍被拒绝。
@pytest.mark.parametrize("name, kind", [("pid", PIDController), ("adrc", ADRCController)])
def test_factory_wraps_supported_controllers_for_lateral_vessel(name, kind):
    controller = create_controller(name, WAMV, None, PROPORTIONAL)
    assert isinstance(controller, LateralAssistedController) and isinstance(controller.inner, kind)
    result = controller.compute(VesselState(y=-.5), reference(), .1)
    assert result.status == AlgorithmStatus.SUCCESS and result.is_valid()
    assert result.lateral == pytest.approx(.12)
    with pytest.raises(ValueError, match="unsupported vessel model"):
        create_controller(name, USVModel(name="other"), None, PROPORTIONAL)
