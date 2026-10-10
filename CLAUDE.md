# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

ROS 2 Humble + Gazebo Classic 11 的 Otter 无人艇导航仿真：感知定位 → 决策规划（全局搜索 + LOS 轨迹）→ 航行控制（PID/LQR/ADRC）。导航算法为 Python/NumPy；Gazebo/RViz 插件、仿真定位与 Nav2 costmap 支撑为 C++。项目文档、源码注释均为中文。术语表见 `CONTEXT.md`，接口与话题全表见 `docs/distributed-navigation/README.md`。

## 运行环境

- 仓库位于 Windows `E:` 盘，但构建、测试、启动全部在 **WSL `Ubuntu-22.04`** 的 Bash 中执行（仓库路径 `/mnt/e/usv_ros_motion_planning-ros2-humble`）。PowerShell / Git Bash 下没有 ROS。从 Windows 侧调用：
  `wsl.exe -d Ubuntu-22.04 -- bash -lc "cd '/mnt/e/usv_ros_motion_planning-ros2-humble' && source /opt/ros/humble/setup.bash && ..."`
- 本机基线安装区是 `/home/usv/.local/share/usv_ros_motion_planning/base/install`。LaDDS 航路脚本（`build_ladds_route.bash`、`start_ladds_route*.bash`）以它为 base，构建和启动都用仓库自己的 `/mnt/e/usv_ros_motion_planning-ros2-humble/install`，不再使用 `/tmp` 独立 overlay。不要复用其他机器的 `build/`、`install/`。
- 例外：LaDDS 中继 `scripts/start_ladds_relay.ps1` 在 Windows PowerShell 运行（Windows SDK → 本机 TCP → WSL）。

## 常用命令（WSL，仓库根目录）

```bash
# 构建；--base-paths src 必须保留，否则会收集 deliverables/ 里的旧导出包副本（scripts/build.sh 未加此参数）
source /opt/ros/humble/setup.bash
colcon build --base-paths src --executor sequential --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON
colcon build --base-paths src --packages-select usv_decision usv_control   # 只重建部分包
source install/setup.bash

# 启动（launch 读取安装区资源：改了默认 YAML / world / URDF 后必须重新构建）
ros2 launch sim_env main.launch.py world:=ocean headless:=true
ros2 launch sim_env main.launch.py world:=ocean headless:=true planner_name:=dijkstra controller_name:=lqr
ros2 topic pub --once /otter/goal geometry_msgs/msg/PoseStamped \
  '{header: {frame_id: otter/map}, pose: {position: {x: 35.0, y: -10.0}, orientation: {w: 1.0}}}'
ros2 topic echo /otter/navigation_status
ros2 service call /otter/cancel_navigation std_srvs/srv/Trigger '{}'

# 单个测试 / 单个用例（conftest.py 已把各 Python 包源码根加入 sys.path，纯算法测试无需构建）
python3 -m pytest src/core/decision_module/path_planner/test/test_grid_planning.py -q
python3 -m pytest src/core/control_module/controller/test/test_controllers.py -q -k lqr

# 全量源码 + ROS 回归（两个参数都必须是绝对路径，输出目录必须不存在）
export USV_DISTRIBUTED_INSTALL="$(pwd)/install"
bash scripts/validation/run_distributed_source_tests.bash "$USV_DISTRIBUTED_INSTALL" "$(pwd)/validation-results/source-tests-NNN"

# Gazebo 场景验收（当前三模块版本必须带 --distributed；--dry-run 只生成计划）
python3 scripts/validation/ros_acceptance.py --workspace "$(pwd)" --distributed \
  --controllers pid --scenario straight --repeats 1 --output /tmp/usv_acceptance_smoke_NNN
```

测试注意：
- `*_ros.py` 测试需要已 `source` 的安装区，缺少 rclpy 时会跳过——**跳过不能算作 ROS 集成通过**。安装区审计测试靠 `USV_DISTRIBUTED_INSTALL` 启用。
- `pytest.ini` 用 `--import-mode=importlib` 并显式列出 `testpaths`；新增 Python 包或测试目录时，要同时加到 `pytest.ini` 的 `testpaths` 和 `conftest.py` 的源码根列表。
- 各测试入口使用隔离 ROS 域（源码回归默认 192、验收 182、LaDDS 联调 42，均 `ROS_LOCALHOST_ONLY=1`）；不要与正在运行的仿真共用域。
- C++ 格式遵循根目录 `.clang-format`。

## 架构要点

### 包的分层
`src/core/*_module/` 只是按业务分组的普通目录，带 `package.xml` 的子目录才是 ROS 包。每个模块成对出现：
- **节点包**（`usv_decision`、`usv_control`、`usv_perception`）：ROS 消息、参数、TF、定时器、发布。
- **算法包**（`path_planner`、`path_processor`、`trajectory_planner`、`controller`、`perception`、`shared/common`）：与 ROS 解耦的纯 Python，可独立测试复用。
- **`shared/usv_navigation`**：公共配置解析（`configuration.py`）、节点基类 `NavigationInputs`（`ros_support.py`：odom/costmap 订阅、心跳、会话、当前船体占用检查）、轨迹编解码（`protocol.py`）、就绪判定（`state_rules.py`）、launch 公共参数。它不安装业务节点。
- **`shared/usv_navigation_interfaces`**：模块间消息 `NavigationReference`、`ControlReport`、`ModuleHeartbeat` 及服务 `NavigateTo`/`NavigateRoute`/`StopNavigation`。改消息需同时更新决策、控制和 LaDDS 适配器。
- **`sim_env`**：Gazebo 世界、船模、`fake_localization`（定位/TF 取仿真真值）及 `main.launch.py` 组合入口。`sim_env/sim_env/{configuration,runtime,simulation}.py` 只是旧导入路径的兼容转发。

### 进程与数据流
仿真支撑、感知、决策、航控可由 `main.launch.py` 一起启动，也可四个终端分别启动（二选一，避免重复节点）。决策 → 航控只通过 `navigation/reference`（完整 8 列轨迹 t,s,x,y,θ,κ,v,a + 目标 + 任务身份 + 完整有效参数 JSON + 艇附近的一段航线折线 `route_x/route_y`，后者只供横向环求航线偏差）；`global_path`/`local_trajectory` 的 `Path` 仅供显示。航控回 `navigation/control_report`（实际推力、完成、参考撤销）。

- **决策节点**（`usv_decision/decision.py`）对外节点名仍为 `usv_simulation`，以保持参数名兼容（参数前缀 `usv_simulation.controller.` / `usv_simulation.trajectory_planner.`，代码里分别是常量 `C` / `T`）。
- **规划在独立子进程中执行**：`runtime.py` 的 `LatestPlanWorker` 用 spawn 模式的单进程 `ProcessPoolExecutor`，最多一个运行中 + 一个可被覆盖的待处理 `PlanningRequest`（冻结快照）。`plan_navigation` / `plan_route_navigation` 是纯函数；过期结果靠 `version` 过滤。流程：修剪并在最新地图上复核旧路线 → 失败才重新搜索 → LOS 生成轨迹 → 完整船体扫掠碰撞检查。
- **任务身份**：`task_id`（目标生命周期）、`revision/version`（规划版本）、`sequence`（快照内容），加上 planner/control/perception 三方进程 `session`（经心跳发现）。任一模块重启后旧任务不会恢复，必须重新下发目标；同一任务内参考替换不重置 PID。
- **碰撞检查位置**（ADR 0002）：轨迹扫掠检查在决策侧；当前船体占用检查在航控的每个控制周期。航控也读取 costmap，但地图所有权属于感知。
- **完成判定**用用户请求的精确目标坐标，而非栅格中心；LOS 停止半径会据此收缩。
- `cancel_navigation` 要等航控确认已发布零推力才返回成功；零推力≠船已停止。`motion_mode` 除 `NAVIGATE` 外还有 `BRAKE`（0331 主动刹停）和 `HOLD`（0211/taskType=80 终点值守），见 `usv_ladds_bridge/README.stop-guard.md`。

### 算法选择与配置
- 四份共享算法 YAML 只维护一份：`src/core/shared/usv_navigation/config/`（动力学、控制器、LOS、全局规划）；感知专属的 mapping/costmap YAML 在 `usv_perception/config/`。决策与航控加载同一份。
- 加载顺序：项目 YAML → 非空 launch 覆盖（`planner_name` ∈ astar/dijkstra/gbfs/hybrid_astar，`controller_name` ∈ pid/lqr/adrc，`trajectory_name` = los；空值保留 YAML）→ namespace → 规范化与校验。YAML 被展平成点分键；**新增参数必须在 `configuration.py` 的 `DEFAULTS` 中登记**，历史拼写别名也在此处理。`ros2 param dump` 的输出不能当项目 YAML 用。
- 船型由 launch 参数 `robot_type`（`otter` / `wamv`）选择，默认值取用户场景配置 `user_config.yaml` 的 `robot1_type`（查找顺序：环境变量 `USV_USER_CONFIG` → 安装区所在工作区的 `src/user_config/user_config.yaml`，直接读源码、不用重建 → `~/.rmp_user_config.yaml`；`world` 默认值同源；根目录 `conftest.py` 把该环境变量指向空设备，测试不受本机文件影响）：它决定仿真船模 xacro、`robots/<robot_type>/` 下的默认船模、控制器、代价地图 YAML，以及 `robot_namespace` 的默认值（与船型同名，即 `/otter/...` 或 `/wamv/...`；显式传入时以其为准）。LaDDS 桥配置和 `start_ladds_*.bash` 仍只按 Otter 写。WAM-V 是 `type: Azimuth`（两台艉部全回转桨，仿真用 H 布局；另支持 `Differential` 和带独立侧推器的 `Omni`）。横向力需求由 0924 横向控制器给出，`thruster_output.py` 把纵向力、转艏力矩和横向力一起分给两台桨（横向力按纵向推力大小分摊、桨角不超过 `thrust_allocator.max_deflection`、补偿艉部力臂 `thruster_x`），并多发 `thrusters/*_thrust_angle` 两路桨角；交付分配器的 mode 1 几何没有采用。横向环只在 `NAVIGATE` 跟踪时出力，输入是艇到航线最近点的横向偏差（决策在 `NavigationReference.route_x/route_y` 里附上艇附近的一段航线，多航点航路模式下航线随船走、偏差约为零）；`BRAKE`/`HOLD` 及其制动尾迹预测不含侧推，相关增益仍是 Otter 标定值。船体占用尺寸 `navigation.footprint_*` 随船模 YAML 给出。
- 在线可改的只有 `HOT_PARAMETERS`（PID 6 个增益 + LOS 4 个导引参数）。改 LOS 参数会提升规划版本并使当前轨迹失效；航控会校验收到的参数快照与启动配置只在 HOT 键上有差异。
- 工厂：`path_planner/utils/path_planner_factory.py`（`AStarPathPlanner` 一个类实现 astar/dijkstra/gbfs，`hybrid_astar` 走独立实现）、`controller/controller_factory.py`（船模 `otter` / `wamv`；船模能产生横向力且启用横向环时在所选算法外叠加 `lateral_thrust.py`）、`trajectory_planner_factory.py`。Hybrid A* 带 `preserves_kinematics`：走 `hybrid_path_reuse` 复用路线、不做直连简化，且必须精确连到目标。

### 其他子系统
- `communication_module/usv_ladds_bridge`：LaDDS（主题 `0211` 下发航路、`0052` 回报完成）与 ROS 2 的桥接/适配器，调用决策的导航服务；用 `scripts/build_ladds_route.bash` 以基线安装区为 underlay、增量构建到仓库自己的 `install/`，启动见 `docs/ladds-multi-waypoint-quickstart.md`。
- `mission_module/usv_mission`：保留的定点值守/圆形巡航/阵位保持，普通导航主入口不启动。
- `src/plugins/dynamic_xml_config` 按 `src/user_config/user_config.yaml` 生成场景障碍（`scripts/main.sh` 会执行）；`user_config.yaml` 里只有 `world` 和 `robot1_type` 在启动时生效，旧的算法字段和出生位姿字段**不**决定当前行为。
- `src/sim_env/test/legacy_reference.py` 是拆分前单节点实现，仅作回归参照，不是生产入口。

## 工作约定

- 保持现有注释风格：每个类/函数上方一行中文注释说明意图，另有英文 docstring/行内注释说明约束。
- 不属于当前运行链、除非明确要求否则不要改：`deliverables/`（旧导出快照）、根目录 `0924controller/`、`USV_control/`（控制参考算法）、`validation/`、`validation-results/`、`build/`、`install/`、`log/`。
- `docs/validation/` 下是带日期、绑定具体源码版本的历史证据；README 中的通过数字也是快照。未实际运行不要声称新代码已通过。
- 不要用 `killall`/`pkill` 批量结束进程；`scripts/killpro.sh` 按命令行子串强杀，可能误伤其他仿真实例。验收脚本只结束自己创建的进程组。
- 相关分支：`main` / `cpp-baseline` 为原作者 C++ 版本，`refactor/python-navigation` 为 Python 迁移基线。
