# 三模块普通导航

本文最初随分支 `codex/distributed-navigation` 编写，该分支基于拆分前未提交代码继续开发；当前工作分支见[项目 README](../../README.md)。障碍物感知仅使用 LiDAR。范围为同机 ROS 2 仿真联调；感知定位、决策规划、航行控制可以分别启动。实船硬件、跨机器验证、值守与阵位保持不属于本次改造。

## 代码在哪里

`*_module` 是普通目录，方便按业务查看；其中带 `package.xml` 的目录才是 ROS 功能包。每个模块的节点包和算法包放在一起，算法包仍可独立复用。

```text
src/
├── core/
│   ├── perception_module/
│   │   ├── usv_perception/             # 感知节点、launch、专属配置
│   │   └── perception/                 # LiDAR 建图算法
│   ├── decision_module/
│   │   ├── usv_decision/               # 决策节点、规划 worker、launch
│   │   ├── path_planner/               # A* / Dijkstra / GBFS
│   │   ├── path_processor/             # 路径后处理
│   │   └── trajectory_planner/         # LOS 与轨迹数据结构
│   ├── control_module/
│   │   ├── usv_control/                # 航控节点、launch
│   │   └── controller/                 # PID / LQR / 0914 PI＋ADRC
│   ├── shared/
│   │   ├── common/                     # 共用模型与工具
│   │   ├── usv_navigation/             # 公共配置、输入适配、协议支持
│   │   └── usv_navigation_interfaces/  # 模块间消息
│   └── mission_module/
│       ├── usv_mission/                # 保留已有任务实现，普通导航不启动
│       └── usv_mission_interfaces/
├── sim_env/                            # 仿真定位、Gazebo、船模、主组合入口
└── plugins/                            # 原仿真及可视化插件
```

当前源码共 21 个 ROS 包，其中 `communication_module/usv_ladds_bridge` 是后来加入的 LaDDS 通信包，上面的目录图未列出。三个模块的 Python 节点实现分别在 `usv_perception/usv_perception`、`usv_decision/usv_decision`、`usv_control/usv_control` 下；感知 costmap 的 C++ 实现位于 `usv_perception/src`。每个节点包都有自己的 `launch/` 和 `config/`。公共 `usv_navigation` 不再安装业务节点 executable。

## 构建与启动

在 ROS 2 Humble 的 Linux/WSL 环境中，从源码工作区执行：

```bash
source /opt/ros/humble/setup.bash
colcon build --base-paths src --executor sequential --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
ros2 launch sim_env main.launch.py headless:=true
```

2026-09-14 整理时使用的是独立安装区 `/home/usv/module-layout-20260914/install`，这只是当时的验证环境。现在直接使用仓库自己的 `install/`：构建完成后，在新终端先加载 `/opt/ros/humble/setup.bash`，再加载 `install/setup.bash`。

原 `main.launch.py` 入口、默认 A* → LOS → PID 以及 LiDAR 输入保持。可通过 `controller_name:=lqr` 选择 LQR 控制器。去掉 `headless:=true` 可使用原 Gazebo GUI/RViz。

分开联调时，在四个已加载同一安装区的终端分别执行以下命令。各终端使用相同 `robot_namespace` 和配置文件参数：

```bash
ros2 launch sim_env simulation_support.launch.py headless:=true
ros2 launch usv_perception perception.launch.py
ros2 launch usv_decision decision.launch.py
ros2 launch usv_control control.launch.py
```

`main.launch.py` 已包含以上四组；完整启动与分别启动每次选一种，避免产生重复节点。

旧 `ros2 launch sim_env perception.launch.py` 等三个入口仍可用，它们转发到各自包的 launch。单独启动决策或航控后，仍需要感知/仿真输入才能执行目标。

| 模块 | 运行内容 | 所有权 |
|---|---|---|
| 感知定位 | `usv_perception` 包的 LiDAR mapping、Costmap、模块身份监测 | 感知输出与地图维护；当前不接真实定位传感器 |
| 决策规划 | `usv_decision/decision_node`，节点名保持 `usv_simulation` | 目标、路径与轨迹规划、轨迹检查、对外任务状态 |
| 航控 | `usv_control/control_node`，节点名 `usv_control` | PID/LQR、当前船体占用检查、推进器发布 |
| 仿真支撑 | Gazebo、船模、fake_localization、状态发布及可视化 | 由仿真真值提供 odom/TF，作为验证环境 |

路径与轨迹规划继续在决策模块内部传递对象。规划 worker 使用原有有界任务队列；航控独立进程不运行规划器。

## YAML 加载

继续使用原项目 YAML 格式，配置项可以保留原有层级与兼容别名。配置文件物理位置可自行调整，通过启动参数指定：

```bash
ros2 launch sim_env main.launch.py headless:=true \
  dynamics_config:=/absolute/config/dynamics_otter.yaml \
  controller_config:=/absolute/config/controller_params_otter.yaml \
  trajectory_config:=/absolute/config/trajectory_planner_params.yaml \
  planner_config:=/absolute/config/common_planner_params.yaml
```

这四个参数也适用于决策、航控独立入口。默认配置位于 `src/core/shared/usv_navigation/config`，只维护一份；加载顺序为四份项目 YAML → 非空算法启动覆盖 → namespace → 参数规范化与校验。空 `controller_name` 等启动覆盖保留 YAML 的选择。

感知专属 YAML 位于 `src/core/perception_module/usv_perception/config`。通过 `mapping_config:=/absolute/mapping.yaml` 和 `costmap_config:=/absolute/costmap.yaml` 指定其他文件，独立感知入口与主入口均支持。决策和航控的 `config/README.md` 指向上述共享配置，不复制 YAML。

### 船型选择

`robot_type` 取 `otter` 或 `wamv`，四个入口都接受，分开启动时各终端须一致。默认值来自用户场景配置 `user_config.yaml` 的 `robot1_type`（沿用原工程的写法），命令行写了 `robot_type:=` 时以命令行为准：

```yaml
# src/user_config/user_config.yaml
robots_config:
  - robot1_type: "wamv"    # otter 或 wamv
```

这份文件按以下顺序查找，先找到的生效；都没有时船型为 `otter`：

1. 环境变量 `USV_USER_CONFIG` 指定的文件（指定了却不存在会报错）。
2. 当前所用安装区所在工作区里的 `src/user_config/user_config.yaml`。它在启动时直接读源码目录，改完不用重新构建。
3. `~/.rmp_user_config.yaml`。

`world` 的默认值取自同一份文件。启动日志的第一行 `[usv_navigation] robot_type: ... (user config: ...)` 写明实际生效的船型和所读的文件。文件里其余 `robot1_*` 字段当前不读取。决策和仿真分装在两个容器时，两边各读各的文件，要保持一致或都在命令行写明 `robot_type:=`。

船型决定四件事：

- 仿真支撑展开哪个船模：`sim_env/urdf/<robot_type>/<robot_type>.urdf.xacro`。
- 决策与航控的默认船模、控制器 YAML：`usv_navigation/config/robots/<robot_type>/`。
- 感知的默认代价地图 YAML：`usv_perception/config/robots/<robot_type>/`。
- `robot_namespace` 的默认值：与船型同名，即 `/otter/...` 或 `/wamv/...`。Gazebo 实体名、TF 前缀和全部话题随之变化。

显式传入的 `robot_namespace`、`dynamics_config` 等仍然优先。

```bash
ros2 launch sim_env main.launch.py world:=ocean headless:=true robot_type:=wamv
ros2 topic pub --once /wamv/goal geometry_msgs/msg/PoseStamped \
  '{header: {frame_id: wamv/map}, pose: {position: {x: 35.0, y: -10.0}, orientation: {w: 1.0}}}'
ros2 topic echo /wamv/navigation_status
```

随包安装的 `sim_env.rviz` 按 `otter` 命名空间写固定坐标系和话题；用其他命名空间带界面启动时，仿真支撑会在系统临时目录生成一份替换过前缀的副本交给 RViz。LaDDS 桥的三份配置和 `start_ladds_*.bash` 仍按 `/otter/...` 和 Otter 的 YAML 写，WAM-V 走 LaDDS 联调前需要另配。

WAM-V 是两台艉部全回转推进器，没有独立侧推器：船模 YAML 中 `type: Azimuth`，仿真用双艉推的 H 布局。推力插件按线性映射（推力 = 命令 × 最大力），与导航侧模型一致。

横向力需求由 0924 横向控制器给出（控制器 YAML 的 `lateral_controller` 段，`enabled: true`），叠加在所选的 PID/LQR/ADRC 之外：

- 只在普通导航（`NAVIGATE`）跟踪时起作用；制动、值守和到达后侧推为零。
- 输入是艇到**航线最近点**的横向偏差。决策随每条参考在 `route_x`/`route_y` 里附上艇附近的一段全局航线，航控每个控制周期用最新位姿重新求最近点。参考不带航线时侧推为零。
- 多航点航路（`NavigateRoute`，LaDDS 0211）每次规划都从当前船位重新连线到下一航点，航线随船走，横向偏差约为零，侧推在该模式下基本不出力。
- 归一化的横向力需求写入 `control_report.lateral`（正值向左舷，1 对应船模 YAML 的 `max_lateral_force`）。

航速环、艏向环和横向环分别给出纵向力、转艏力矩和横向力，由航控的推力输出口一起分给两台桨：

- 两桨纵向推力之和是纵向力；两者之差既给出转艏力矩，又抵消横向力作用在船艉带出的力矩（船模 YAML 的 `thruster_x` 是回转轴的纵向位置）。只要横向力时，两台桨一正一倒。
- 横向力按两台桨纵向推力的大小成比例分摊，两台桨偏转的角度大小相同。某台桨的纵向推力过零时它分到的横向力也是零，所以换侧只发生在该桨推力为零的瞬间。
- 桨角不超过 `thrust_allocator.max_deflection`（默认 45°）。横向力需要更大桨角时只减小横向力，前进和转艏照给。任一台桨超过最大推力时三项需求同比缩小。
- 横向力需求为零时桨角恒为 0，与固定桨差动完全相同。制动和值守不带横向力，所以始终是这种情况。

对应的输出：

- `thrusters/left_thrust_cmd`、`right_thrust_cmd`：每台桨沿自身桨角方向的归一化推力，倒推为负。
- `thrusters/left_thrust_angle`、`right_thrust_angle`：桨角，弧度，0 指向船艏，推力方向转向左舷为正。只有全回转船型发布。
- `thrusters/vector_cmd`（东湖 0054）：同一组桨角和转速；横向力需求非零时 `mode` 为 1，否则为 0。
- `thrusters/lateral_thrust_cmd` 没有对应的推进器，恒为零。

0924 交付的分配器只用了它的推力—转速换算，`mode=1` 的几何没有采用：它把横向力平均分给两台桨，纵向推力变号的那台桨要带着推力转过四分之一圈以上，仿真里船会被甩开。

船模 YAML 的 `type` 也可以改成 `Differential`（两台固定桨，不出横向力，须同时关闭横向环）或 `Omni`（固定桨加一台独立侧推器，仿真自动换成 T 布局，横向力直接发到 `thrusters/lateral_thrust_cmd`）。

横向环用比例加微分（`k_p_lateral: 60`、`k_d_lateral: 150`、`k_i_lateral: 0`）。WAM-V 横移方向阻尼很小，纯比例会绕航线来回摆；积分在转弯时攒下的单侧力之后退不掉，所以取零。无界面仿真的转弯场景里，起航 15 秒后艇到航线的距离最大 0.13 米，关掉横向环的对照为 2.4 米。

WAM-V 的 ADRC 增益由 Otter 按质量和阻尼等比放大得到，是仿真起点。值守控制器参数和制动尾迹预测里的制动增益仍是 Otter 标定值，WAM-V 上的 `BRAKE`/`HOLD` 尚未调整。

WAM-V 激光的最小量程设为 1.5 米（`wamv.urdf.xacro`）。扫描高度上的 GPS 天线和后支柱距激光 1.2–1.35 米，按原来的 1.2 米会产生自身回波；代价地图 5 米一格时，这些回波会把紧邻船体的一格标成致命障碍并触发 `CURRENT_FOOTPRINT_BLOCKED`。

公共解析实现位于 `src/core/shared/usv_navigation/usv_navigation/configuration.py`；`sim_env.configuration` 是兼容导入。决策和航控启动时都记录完整有效参数，航控收到轨迹时验证静态配置一致。既有 PID/LOS 热参数仍可通过原节点参数服务修改，控制端在下一控制周期应用 PID 修改，不清积分。

```bash
ros2 param dump /otter/usv_simulation
ros2 param dump /otter/usv_control
ros2 service call /otter/export_parameters std_srvs/srv/Trigger '{}'
ros2 param set /otter/usv_simulation usv_simulation.controller.pid_controller.k_p_speed 2.5
```

## 话题与任务接口

以下名称均相对于 `robot_namespace`，默认 `/otter`。

| 接口 | 发布/提供方 | 接收方 | 内容 |
|---|---|---|---|
| `odom` | 当前由仿真 fake_localization 提供 | 决策、航控 | 原 Odometry；将来接定位模块 |
| `costmap_raw` | 感知定位 | 决策、航控 | 原 Costmap，接收后各持有独立快照 |
| `goal` | 外部任务工具 | 决策 | 原 PoseStamped |
| `cancel_navigation` | 决策服务 | 外部任务工具 | 原 Trigger；成功前确认航控已发布零输出 |
| `navigation_status` | 决策 | 监控 | 保留原 JSON 字段，汇总实际控制反馈 |
| `global_path`、`local_trajectory` | 决策 | RViz/监控 | 原 Path 显示接口 |
| `navigation/reference` | 决策 | 航控 | 完整八列轨迹、源时戳、目标、任务编号、有效配置，以及艇附近的一段全局航线（`route_x`/`route_y`，供横向环求航线偏差） |
| `navigation/control_report` | 航控 | 决策 | 实际模式、推力、完成或参考撤销反馈 |
| `navigation/heartbeat` | 三模块 | 决策、航控 | 模块进程身份及存活信息 |
| `thrusters/*_thrust_cmd` | 航控 | 原仿真推进器 | 原 Float32，左右及横向三路。固定桨船型左右两路是控制器的归一化命令；全回转船型是每台桨沿桨角方向的归一化推力。横向一路只在带独立侧推器的船模上非零 |
| `thrusters/*_thrust_angle` | 航控 | 仿真推进器 | Float32 桨角（弧度，转向左舷为正），左右两路，只有全回转船型发布 |
| `thrusters/vector_cmd` | 航控 | LaDDS 桥（东湖 0054） | `ThrusterCommand`：同一命令经 0924 推力分配器换算的桨角（rad）与转速（rpm） |

`Path` 仅供显示。执行轨迹使用 `usv_navigation_interfaces/NavigationReference`，完整保存 t、s、x、y、theta、kappa、velocity、acceleration。重发消息不改变规划源时戳。

同一任务的参考替换与等待新轨迹不重置 PID。目标替换、取消、完成及模块重启具有独立任务边界。任一模块重启后旧任务不能自动恢复，需要重新下发目标；感知监测通过已有输出发布者身份变化识别其子进程重启。短暂输入断流继续按原状态/地图/轨迹时效规则停推与恢复。

## 验证范围

各业务节点的测试位于对应业务包的 `test/`，跨模块闭环测试位于 `usv_decision/test/test_distributed_ros.py`；公共协议测试位于 `shared/usv_navigation/test/`。它们覆盖完整轨迹传递、旧消息、真实 ROS 目标/取消/参数/重启闭环，以及感知发布者重启。`src/sim_env/test/test_launch_modules.py` 检查 YAML 和启动组合。

原单节点仅作为 `src/sim_env/test/legacy_reference.py` 中的回归参照，不安装、不作为生产入口。旧测试验证参照行为；新模块测试和实际 Gazebo 记录才用于证明拆分版本。[三模块拆分验证记录](../validation/distributed-navigation-001/README.md) 保存清理前的阶段结果，属于历史证据。

本轮组织调整的需求与边界见[已确认需求](module-package-organization.md)，新的构建与运行证据见[模块目录整理验证](../validation/module-layout-001/README.md)。[毫米波移除验证](../validation/lidar-only-001/README.md) 对应整理前的 17 包版本，保留为历史证据。

## 通过脚本选择安装区

`scripts/main.sh` 默认加载源码项目内的 `install/setup.bash`。如果构建输出位于独立目录，通过 `USV_INSTALL_DIR` 显式指定包含 `setup.bash` 的安装目录：

```bash
# 在当前源码项目内启动默认安装区。
bash scripts/main.sh headless:=true

# 使用独立安装区；路径含空格时保留引号。
USV_INSTALL_DIR="/absolute/path/to/install" bash scripts/main.sh headless:=true controller_name:=lqr
```

未设置或设置为空时使用项目内的默认目录。指定目录缺少 `setup.bash` 时，脚本报出所检查的完整路径和指定方法并退出，不自动回退或搜索历史安装区。后面的 launch 参数原样传给 `ros2 launch sim_env main.launch.py`；原有 ROS/Gazebo 环境加载和场景生成流程保持。
