# 新控制器接入说明

适用：在当前 Python / ROS 2 系统中接入一个已有的新控制算法，沿用现有规划轨迹和双推进器接口。算法名称以 `new` 为例，接入时替换为实际名称。日期：2026-09-14。

## 1. 控制器接收什么、输出什么

调用接口保持如下形式：

```python
controller = NewController(model, config)
command = controller.compute(state, trajectory, dt)
```

| 输入 | 内容 |
| --- | --- |
| `state` | 实际状态 `x, y, psi, u, v, r, stamp, frame_id`：位置、艏向、艇体系纵横向速度、转向角速度、时间戳、坐标系 |
| `trajectory` | 期望轨迹，每行 `[t, s, x, y, theta, kappa, velocity, acceleration]`；另有时间原点 `stamp`、坐标系和最终目标 `goal` |
| `dt` | 两次控制调用之间的实际时间间隔，单位 s；当前默认控制频率 10 Hz |
| `model` | 当前艇型参数，按新算法需要使用 |

位置用 m，速度用 m/s，角度用 rad；艏向正东为零、逆时针为正，艇体系向前、向左为正。状态和轨迹必须使用同一世界坐标系与时钟。实际横向速度 `v` 保留，不要直接填零。

`u,v` 为对地速度在艇体系中的分量，`r` 为转向角速度（rad/s，左转为正）。两个 `stamp` 均以 s 表示：`state.stamp` 是状态实际对应时刻，`trajectory.stamp` 是参考时间原点；轨迹 `t` 是相对此原点的秒数，`s` 是累计弧长（m），`kappa` 是曲率（1/m，当前无侧滑参考中左弯为正），`acceleration` 是参考速度变化率（m/s²）。`goal` 是任务最终位置，局部轨迹末点不一定是最终目标。

输出为 `ControlCommand(left, right, lateral, status, message)`：左右指令取值 `[-1,1]`，正值正推、负值反推，当前 `lateral=0`。这是归一化指令；如果原算法输出力和力矩，需要先完成推力分配与归一化。

接口定义：[models.py](../src/core/shared/common/common/models.py)、[trajectory.py](../src/core/decision_module/trajectory_planner/trajectory_planner/trajectory.py)。

## 2. 给现有算法加一层接口适配

在 [controller 算法目录](../src/core/control_module/controller/controller) 增加 `new_controller.py`，提供 `NewController(model, config=None)`、`compute()` 和 `reset()`。

建议继承现有 [Controller](../src/core/control_module/controller/controller/controller.py)，沿用参考读取、到点判断和输出限幅。下面是类内方法示意，`your_algorithm` 需要替换成已有算法的调用：

```python
def compute(self, state, trajectory, dt):
    reference, failure = self._reference(state, trajectory, dt)
    if failure is not None:
        return failure

    # 接入已有算法；此处约定返回左右归一化指令。
    left, right = your_algorithm(state, reference, dt)
    return self._command(left, right)
```

构造函数中调用 `super().__init__(model)`，将传入的 `config` 合并到 `self.config`。复用 `_reference()` 时至少提供三个参数：`goal_distance_tolerance`、`goal_speed_tolerance`、`lookahead_time`。参数应检查有限值和合理范围；可参照 [PID 实现](../src/core/control_module/controller/controller/pid_controller.py)。

`reset()` 用来清除积分、滤波器或优化器历史；控制器实例会跨周期复用。计算失败返回带原因的 `INVALID_INPUT`，求解超时返回 `TIMEOUT`，并给出零指令，不能返回未完成的解。

当前运行层会在新任务、取消/关闭任务、完成到点，以及关闭任务的时钟倒退或模块重启处理中调用 `reset()`；同一任务收到新的滚动参考时不会自动复位。新算法若有优化器热启动状态，需要自行判断新参考是否仍适合复用。求解器应自行设置计算时限；控制节点记录周期超时，但不会抢占正在执行的 Python 算法。

参考点通常按 `max(0, state.stamp-trajectory.stamp) + lookahead_time` 查询并插值。若新算法需要整段预测轨迹，可以读取 `trajectory.points`，同时保留输入检查和实际到点处理；不要修改这份只读数组。

`reference` 是 `TrajectoryCell`，可按下面的属性读取。注意曲率在 Python 对象中叫 `curv`，在 ROS 消息中叫 `kappa`：

```python
x_ref, y_ref = reference.x, reference.y
psi_ref, speed_ref = reference.theta, reference.velocity
curvature_ref, acceleration_ref = reference.curv, reference.acceleration
```

复用 `_reference()` 会沿用当前终点语义：实际位置在容差内、速度仍高时返回 `SETTLING` 和零指令；位置与速度均满足时返回 `GOAL_REACHED`。当前不检查终点艏向，零指令代表停推，不代表主动制动或立即停船。

需要对齐一个已有差异：PID 使用 `hypot(u,v)` 作为速度反馈，LQR 使用纵向 `u`。接入者应明确新算法的速度口径。期望轨迹由规划层提供，本次接入不需要重新编写轨迹生成器。

## 3. 修改哪些文件

| 文件 | 要做的修改 |
| --- | --- |
| 新增 `new_controller.py` | 适配已有算法，实现上述三个方法 |
| [controller_factory.py](../src/core/control_module/controller/controller/controller_factory.py) | 导入新类，在 `algorithms` 字典增加 `"new": NewController` |
| [configuration.py](../src/core/shared/usv_navigation/usv_navigation/configuration.py) | 在 `DEFAULTS` 中增加新参数；在 `validate_parameters()` 中允许名称 `new`，并校验新参数 |
| [launch_support.py](../src/core/shared/usv_navigation/usv_navigation/launch_support.py) | 在 `controller_name` 的 `choices` 中增加 `new` |
| [controller_params_otter.yaml](../src/core/shared/usv_navigation/config/robots/otter/controller_params_otter.yaml) | 增加 `new_controller` 参数段 |

工厂注册示意：

```python
from .new_controller import NewController

algorithms = {"pid": PIDController, "lqr": LQRController, "new": NewController}
```

在现有 YAML 的 `controller` 下合并以下参数段，保留已有 PID/LQR 配置。数值仅作接入起点，按新算法调整：

```yaml
controller:
  new_controller:
    goal_distance_tolerance: 1.5
    goal_speed_tolerance: 0.25
    lookahead_time: 2.6
    # 在这里增加新算法自己的参数
```

`configuration.py` 中对应的默认键为 `C + "new_controller.参数名"`，其中 `C` 已定义为 `usv_simulation.controller.`。所有 YAML 新参数都要进入 `DEFAULTS`，否则会被参数白名单拒绝。

**必须提供 `new_controller.goal_distance_tolerance`**：当前规划代码会按选中的控制器名称读取这个参数。首次接入采用启动时加载参数；如需在线调参，还要扩展 `HOT_PARAMETERS` 和控制节点的参数更新逻辑。

沿用上述接口后，[usv_control/control.py](../src/core/control_module/usv_control/usv_control/control.py) 会通过工厂自动创建并调用新控制器，通常无需改写 ROS 调度和推进器发布。若算法引入额外库，再补充 `controller` 包的依赖声明。

艇型参数在 [dynamics_otter.yaml](../src/core/shared/usv_navigation/config/robots/otter/dynamics_otter.yaml)。本文假设船体与执行器接口不变；换艇时还需处理当前模型仅支持 Otter/Differential 的限制。

实船开发还需甲方提供质量/惯量、推进器安装位置、命令—推力映射、死区/延迟和实际状态接口。当前 YAML 是 Otter 模型参数；规划方还需控制组确认可用速度、转向与停车能力。算法 YAML 不会自动修改 Gazebo 船体，仿真推进映射另见 [推进器配置](../src/sim_env/urdf/otter/thrusters/otter_gazebo_thruster_config.xacro)。

## 4. 构建、启动与验证

完成上述代码修改后，在已经配置 ROS 2 Humble / Gazebo 依赖的 Linux 或 WSL 环境中，从仓库根目录执行：

```bash
source /opt/ros/humble/setup.bash
colcon build --base-paths src --executor sequential
source install/setup.bash
ros2 launch sim_env main.launch.py world:=ocean headless:=true controller_name:=new
```

`--base-paths src` 避免把 `deliverables` 中的旧副本一并构建。启动读取安装区资源，源码 YAML 变更后应重新构建。仅启动独立控制节点而没有状态、地图、心跳和参考时，不会产生正常推进输出。

另开终端，加载同一安装区后确认实际选中了新控制器：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 param get /otter/usv_control usv_simulation.controller.controller_name
```

默认仿真起点为 `(23,-10)`。待系统状态和地图就绪后，可在已加载同一环境的终端发送目标 `(35,-10)`，作为直线场景的联调入口：

```bash
ros2 topic pub --once /otter/goal geometry_msgs/msg/PoseStamped "{header: {frame_id: 'otter/map'}, pose: {position: {x: 35.0, y: -10.0, z: 0.0}, orientation: {w: 1.0}}}"
```

另开已加载同一环境的终端观察控制状态；需要取消时调用现有服务：

```bash
ros2 topic echo /otter/navigation/control_report
# 结束上述持续观察后，或在另一终端执行：
ros2 service call /otter/cancel_navigation std_srvs/srv/Trigger '{}'
```

报告的 `state/reason` 表示运行状态与原因，`task_id/reference_sequence` 对应任务和参考，`task_finished` 表示完成，`left/right/lateral` 是本节点发布的指令。正常跟踪观察 `NAVIGATING`，到点观察 `GOAL_REACHED`；取消后应观察 `CANCELLED` 和三路零指令。若没有进入跟踪，先按 `WAITING_STATE`、`WAITING_MAP`、`WAITING_REFERENCE`、`WAITING_MODULES` 或 `CONFIG_MISMATCH` 的原因检查输入。报告中的指令不等于实船驱动器执行反馈。

数值测试在仓库根目录运行，依赖 NumPy、PyYAML 和 pytest；将新算法测试放入现有控制器测试目录。下列完整范围也是本文 105 项基线结果所用的命令：

```bash
python -B -m pytest -q -rs -p no:cacheprovider src/core/control_module/controller/test src/core/decision_module/trajectory_planner/test src/core/shared/common/test/test_models.py src/core/shared/common/test/test_dynamics.py src/core/shared/usv_navigation/test/test_protocol.py
```

至少检查以下五项：

- 正常状态和轨迹能生成有限、范围正确的左右指令，侧推为零。
- 空轨迹、非有限输入和求解失败不产生异常推进输出。
- `reset()` 后不会继续使用上一任务的积分或其他历史。
- 直线、转弯场景可以跟踪；实际到点后按约定结束任务。
- 取消任务、状态断流时输出归零；控制计算时间能满足设定频率。

现有 [control_demo.py](../scripts/control/control_demo.py) 和 [ros_acceptance.py](../scripts/validation/ros_acceptance.py) 的算法选项目前只支持 PID/LQR；若用它们测试 `new`，需同步补充选项、参数及相关分支。

## 5. 开发者交付内容

提交新控制器源码、注册与配置修改、默认参数、必要依赖、测试结果和一条可复现的启动命令。说明使用哪种速度反馈、内部输出单位，以及仍未验证的场景即可。

本文是接入说明，未实现具体新算法。编写期间，现有控制器、轨迹、模型与协议相关数值测试共 105 项通过，PID/LQR 一步示例均成功；这些是旧实现的基线结果，不代表新控制器已通过 ROS、Gazebo 或实船验证。
