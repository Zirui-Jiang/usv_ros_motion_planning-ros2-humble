# 定点值守与阵位保持使用及验收指南

任务下发、状态机及执行协调现已迁入 `core/usv_mission`，本指南第 1–4 节使用迁移后的入口。迁移范围、独立构建和迁移后验证进度见 [任务模块迁移记录](station-guard-migration.md)，迁移后的通过结论以该记录对应的新证据为准。

迁移前的冻结版本通过 713 项源码测试、14 包构建／安装审计，matrix006 七场景三重复 **21/21 全部通过**；另保留原导航六项历史闭环通过记录，按该记录自身版本解释。第 5–6 节保留这轮历史方法与数据；[迁移前交付证据入口](validation/station-guard-001/README.md) 中的补丁和归档只用于复现旧冻结版本，不代表当前迁移后源码已通过相同矩阵，也不包含共享根目录并行新增的雷达改动。

本功能位于 `function/station-guard-and-keeping` 分支。`usv_mission` 负责接收任务、维持状态机、协调导航／保持／避障和控制发布；A* / LOS 与现有 PID、LQR 跟踪器承担航行阶段，现有保持控制器承担近场驻留。迁移只调整任务应用的职责位置，算法仍由其所属包提供。

| 模块 | 当前职责 |
|---|---|
| [`core/usv_mission`](../src/core/usv_mission/README.md) | 任务 CLI、任务参数、任务状态机、执行协调、任务 ROS 节点及组合启动入口 |
| `core/usv_mission_interfaces` | 保留现有 `SetMission` 类型化服务合同 |
| `core/controller`、`core/trajectory_planner`、`core/path_planner`、`core/common` | 保持已有控制、轨迹、路径及数值算法，不在本次迁移中重设计公共接口 |
| `sim_env` / Gazebo | 只读复用既有 world、船体描述、物理模型、定位与 costmap 支持节点；任务运行时不继承 `Simulator` |

任务仿真入口不启动旧 `sim_env_node`，由 `/otter/mission_manager` 统一发布任务执行控制。不要同时启动向同一艇发布推力的旧导航入口或另一控制节点。后续任务可以继续在任务模块扩展；不同艇型的航控公共接口及控制器设计按各自边界独立处理。

当前避障依据实时 LiDAR 栅格及完整船体碰撞检查，包含障碍出现后的退出、绕行和任务恢复。本次实现没有接入 AIS/Radar 融合、来船运动预测、CPA/TCPA 或 COLREGs 决策，不能将本轮栅格避障结果作为这些能力的验收证据。

## 1. 构建、启动与快速操作

运行环境为 WSL/Linux、ROS 2 Humble 和项目使用的 Gazebo。在所选工作区构建并加载同一安装区：

```bash
cd /absolute/path/to/workspace
source /opt/ros/humble/setup.bash
colcon build --packages-up-to usv_mission --parallel-workers 3 --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash

# 不启动 Gazebo GUI 或 RViz，适合后台运行。
ros2 launch usv_mission mission_sim.launch.py world:=ocean headless:=true
```

需要图形界面时使用 `ros2 launch usv_mission mission_sim.launch.py world:=ocean`。默认艇命名空间为 `otter`，初始位姿为世界坐标 `(23, -10, 0)`，最后一个分量是 ROS yaw，单位 rad。

该组合入口使用已有 Gazebo、定位和传感器资源，同时保留 `world`、`robot_namespace`、`robot_x/y/z/yaw`、`gui`、`rviz`、`headless`、`use_sim_time`、算法选择和传感器参数。已有外部 odom、TF、costmap 时，可只启动任务节点：

```bash
ros2 launch usv_mission mission.launch.py robot_namespace:=otter use_sim_time:=true \
  world_path:="$(ros2 pkg prefix --share sim_env)/worlds/ocean.world"
```

`mission.launch.py` 不启动 Gazebo、感知或定位。`world_path` 在此只提供显式地理参考，外部状态与地图必须使用相同坐标系；也可改用第 3 节的 `geo_reference` 配置。

另开终端，加载相同 ROS 环境和工作区安装区后执行：

```bash
source /opt/ros/humble/setup.bash
source /absolute/path/to/workspace/install/setup.bash
mission_share="$(ros2 pkg prefix --share usv_mission)"

# 选择其中一个 start；后续 start 会替换正在执行的任务。
ros2 run usv_mission mission_cli start "$mission_share/config/mission/examples/standby.yaml"
ros2 run usv_mission mission_cli status

# 切换为绕圆值守；安装的绕圆示例设置为三圈。
ros2 run usv_mission mission_cli start "$mission_share/config/mission/examples/orbit.yaml"

# 切换为阵位保持。
ros2 run usv_mission mission_cli start "$mission_share/config/mission/examples/station_keep.yaml"

# 取消当前任务或普通单点导航。
ros2 run usv_mission mission_cli cancel
```

`status` 等待并打印下一条导航状态 JSON，不会修改任务。CLI 默认响应等待上限为 5 s；非默认艇命名空间和较长等待时间写在子命令之前，例如：

```bash
ros2 run usv_mission mission_cli --namespace boat --timeout 10 start /absolute/path/mission.yaml
ros2 run usv_mission mission_cli --namespace boat status
ros2 run usv_mission mission_cli --help
```

`--namespace` 必须与启动时的 `robot_namespace` 一致。任务被拒绝或调用超时均返回非零退出码；超时只说明尚未确认响应，应先查询状态，避免重复提交导致任务重新计时、计圈。

## 2. 任务参数、接口与切换语义

| 功能 | `mode` | 行为 |
|---|---|---|
| 待机值守 | `standby` | 先航行至中心；进入位置精度且低速持续 1 s 后待机；距中心超过值守半径时重新返中心 |
| 绕圆值守 | `orbit` | 选择通过碰撞检查的切向切入路径；按指定方向连续绕圆；避障后回归原圆；达到圈数或时长后返中心待机 |
| 定点阵位保持 | `station_keep` | 远距先导航接近，近场恢复位置并调整艏向，随后持续输出纠偏指令；不会因首次进入容差而结束任务 |

以下参数均为 `usv_mission_interfaces/srv/SetMission` 请求字段，也可直接作为任务 YAML 的顶层字段：

| 字段 | 单位／类型 | 默认值与约定 |
|---|---|---|
| `mode` | string | CLI 必填；只能是上表三个值 |
| `latitude` | WGS84 度 | CLI 必填；范围 `[-90, 90]` |
| `longitude` | WGS84 度 | CLI 必填；范围 `[-180, 180]` |
| `heading_deg` | 度 | `0.0`；真北为 0°、顺时针为正，90°为正东；固定艏向控制用于阵位保持 |
| `speed` | m/s | `0.5`；航行、切入和绕圆请求航速 |
| `position_tolerance` | m | `0.5`；中心到达精度、阵位位置精度或绕圆径向精度 |
| `watch_radius` | m | `10.0`；待机越界触发半径及区域内外监测半径 |
| `orbit_radius` | m | `10.0`；名义圆轨迹半径，圆心取请求经纬度 |
| `clockwise` | bool | `true`；`false` 为逆时针 |
| `laps` | uint32 | `0`；仅用于绕圆，0 表示不限圈数 |
| `duration` | s | `0.0`；仅用于绕圆，0 表示不限总时长 |

请求中的数值必须有限；航速、精度和半径必须为正，时长和圈数必须非负。服务端还检查航速是否位于当前模型速度范围内，要求 `position_tolerance < watch_radius`；绕圆额外要求 `position_tolerance < orbit_radius`，以及 `speed / orbit_radius` 不超过配置的 `yaw_rate_max`。非法请求不会被静默改成其他参数。YAML 客户端拒绝未知字段和重复键，模型相关约束仍由服务端最终校验。

绕圆的 `watch_radius` 用于区域监测及完成后的待机行为，不把名义巡航圆裁剪到该半径以内。若 `orbit_radius > watch_radius`，正常圆巡航也可能报告处于监测区外，应根据实际需求设置二者关系。绕圆期间的目标艏向由圆轨迹切线决定。

一个最小阵位任务如下；经纬度仅适用于下节说明的内置 world 示例：

```yaml
mode: station_keep
latitude: -33.72429015691839
longitude: 150.6802481598667
heading_deg: 90.0
```

默认命名空间的 ROS 接口为：

| 名称 | 类型 | 用途 |
|---|---|---|
| `/otter/set_mission` | `usv_mission_interfaces/srv/SetMission` | 创建或替换任务；返回 `accepted`、`mission_id`、`reason` |
| `/otter/navigation_status` | `std_msgs/msg/String` | 导航及任务状态 JSON |
| `/otter/cancel_navigation` | `std_srvs/srv/Trigger` | 取消任务或单点导航，清除临时目标并持续输出零推力 |
| `/otter/goal` | `geometry_msgs/msg/PoseStamped` | 保留原有单点导航接口；按旧语义先清除当前任务，再校验收到的目标 |
| `/otter/mission_markers` | `visualization_msgs/msg/MarkerArray` | 任务中心、边界、圆轨迹、目标艏向及阶段显示 |

需要直接调用服务时，可使用与 YAML 相同的字段：

```bash
ros2 service call /otter/set_mission usv_mission_interfaces/srv/SetMission \
  "{mode: station_keep, latitude: -33.72429015691839, longitude: 150.6802481598667, heading_deg: 90.0}"
```

待机与绕圆通过再次 `start` 切换；服务端完成全部校验后原子替换旧任务，产生新任务编号并重新计时、计圈。`SetMission` 服务拒绝的请求保持旧任务。原 `/otter/goal` 话题沿用“先清除当前任务、再校验目标”的既有语义，因此无效 goal 消息也可能终止当前任务；需要非法请求不影响旧任务时应使用 `SetMission`。绕圆达到限制后的返中心属于原任务的后续行为，状态中的 `mode` 仍为 `orbit`，`phase` 最终变为 `STANDBY`，`orbit_completed` 表示已进入完成后返中心流程。

`laps` 与 `duration` 同时非零时先达到者结束绕圆；二者均为 0 则持续巡航。总时长从接受请求起按仿真时间计算，包含切入、等待和避障；仿真暂停不累计。达到时长不保证当时已经返回中心，返航和安全抢占仍须继续完成。圈数仅累计正常巡航、处于请求精度带内的有符号净角位移；切入、避障、回归过程中跳过的圆弧不补计为有效圈数，反向晃动抵消前向进度。

## 3. 地理参考与独立配置

`mission_sim.launch.py` 从实际选定 world 的 `spherical_coordinates` 读取地理原点。当前支持 `EARTH_WGS84`、`ENU`、`heading_deg=0`；转换采用 WGS84 ECEF 与局部 ENU，内部位置为世界坐标米，内部艏向为从正东逆时针计量的 ROS yaw。组合仿真中的参考字段缺失、数值非法或坐标朝向不支持时，经纬度任务会被拒绝，普通本地坐标 `/otter/goal` 仍可使用。

仅任务入口 `mission.launch.py` 要求显式提供 `world_path` 或 `geo_reference`，二者不能同时使用。`geo_reference` 指向一个仅含下列字段的 YAML：

```yaml
latitude: -33.7242
longitude: 150.68
altitude: 0.0
```

该配置定义外部 odom 与地图使用的 WGS84/ENU、零地图艏向原点；`altitude` 可省略，默认 `0.0 m`。使用方式为 `ros2 launch usv_mission mission.launch.py geo_reference:=/absolute/path/origin.yaml`。未提供任何参考时地理任务保持禁用，不能把默认 `(0, 0)` 当作有效原点；非法或相互冲突的显式参考会阻止启动。

内置 `ocean.world` 与 `example_course.world` 使用相同原点：纬度 `-33.7242°`、经度 `150.68°`、高程 `0 m`。三个任务示例的经纬度对应本地中心约 `(23, -10) m`，不代表企业实际海域坐标，也不保证附近整圆和切入走廊均无障碍。更换 world 后应从实际参考重新生成任务经纬度。例如，在已加载 Python 安装区的终端执行：

```bash
python3 - <<'PY'
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from common.geodesy import GeoReference
world = Path(get_package_share_directory("sim_env")) / "worlds" / "ocean.world"
reference = GeoReference.from_world(world)
latitude, longitude, altitude = reference.to_geographic(23.0, -10.0)
print({"latitude": latitude, "longitude": longitude, "altitude": altitude})
PY
```

任务 YAML 与控制配置 YAML 分开。公开的 `mission_config` 当前包含保持艏向容差和保持控制扫掠检查的模型裕量：

```yaml
mission:
  heading_tolerance_deg: 5.0
  safety_margin: 0.1
```

```bash
ros2 launch usv_mission mission_sim.launch.py world:=ocean headless:=true \
  mission_config:=/absolute/path/mission_params.yaml
```

该配置由新任务包安装到 `share/usv_mission/config/mission/mission_params.yaml`，仅接受 `mission` 顶层映射，地理原点不得在其中覆盖；这些参数不是运行时热更新参数，修改后须重启节点。位置精度通过任务请求配置；其他保持控制增益和限幅是控制器中的代码默认值，不能直接添加未知 ROS 参数进行调整。四个算法 YAML 仍默认只读复用 `sim_env` 原配置，可通过 `dynamics_config`、`controller_config`、`trajectory_config`、`planner_config` 显式给定路径，迁移未改变其数值。

## 4. 执行流程、控制限制与状态解释

待机采用“外圈触发、中心解除”的滞回：到达中心精度且平面速度不超过 `0.1 m/s`、持续 `1 s` 后输出零推力；越过 `watch_radius` 后返中心。待机不是持续位置纠偏；需要在精度带内长期保持位置和固定艏向时，应使用 `station_keep`。

绕圆切入在有限切向候选中，根据路径长度和转角代价选择已通过规划及完整船体检查的方案，表述为候选集内择优，不主张连续空间的全局最优。进入圆轨迹时还要求径向误差进入精度带、与目标切线的艏向误差不超过 15°。连续圆轨迹没有终点减速；触发避障后保留原圆心、半径、方向和已有有效进度，绕行完成后重新对齐并恢复累计。

避障后的回圆候选按规定绕行方向分组：几何筛选后，`15°–180°` 前半圆非空时只在该组内排序和规划；仅当前半圆全部不安全时，才启用 `195°–345°` 后半圆。半圆边界按 `π+1e-9 rad` 判断，避免浮点误差排除 `180°` 点。两组不混排，因此后半圆短路径不会抢占已有可行前半圆目标；每次仍最多处理四个候选、规划预算 `1 s`。

所有活动任务持续检查定位、地图和船体安全，包括没有航行目标的待机与保持阶段。完整船体外扩 `3 m` 的近场包络受侵入时启动退出或绕行；安全点及必要的返回目标通过外扩 `5 m` 检查，并有连续 `2 s`、不同时间戳的新鲜地图支持后恢复。后续返回路径仍需通过规划和碰撞检查。障碍长期占据中心或阵位时，在安全点等待；避障可暂时超出值守边界，状态会报告越界。

选取临时安全点和避障后的回圆点时，还要覆盖到达精度范围内的任意转艏方向。令船长、船宽为 `L`、`W`，请求到达精度为 `δ = position_tolerance`，候选点处轴对齐正方形检查包络的半边长为 `hypot(L/2 + 5, W/2 + 5) + δ`：先对船体两个轴各外扩 `5 m`，再求外接圆半径并加入到达精度。默认 `2.2 × 1.2 m` 船体、`δ=0.5 m` 时，半边长约 `8.781 m`。这项保守检查避免候选点按某一方向看似足够安全、实际到达后转艏却始终不能解除避障；它不替代后续路径和控制扫掠检查。

退出阶段尚无可用参考轨迹，或进入 `AVOIDANCE_BLOCKED` 时，系统尝试经过完整预测制动路径检查的主动制动，减小零推力滑行继续接近障碍的风险；检查不通过则零输出。安全制动可尝试比普通保持更强的阻尼候选，包括增益 60/120、纵向力请求上限 40 N，但每路归一化命令仍不超过 `±0.4`，每个候选都必须单独通过完整停止轨迹及船体扫掠检查；默认模型裕量保持 `0.1 m`，不靠缩小安全裕量放行。当前船体已经与 lethal/unknown 栅格相交时仍输出零指令，不忽略起点碰撞强行穿越障碍。默认定位超时为 `0.5 s`、地图超时为 `1.5 s`。取消、输入超时和仿真时钟回退触发零输出；时钟回退属于阻塞条件，应重发任务或重启相应实验。零推力不是瞬时停车，船体仍可能因惯性或外界扰动移动。

如果普通逃逸轨迹不可行，但规划器证明“先停稳、转艏，再驶向原安全目标”的条件路径可行，任务执行 `BRAKING_ESCAPE → ALIGN_ESCAPE → AVOIDING`。先主动制动，直到实测平面速度 `≤0.04 m/s` 且角速度绝对值 `≤0.02 rad/s`；随后以当时实测位置冻结转艏点，仅采用规划器给出的出发艏向。位置偏差不超过 `min(0.25 m, position_tolerance/2)`、艏向误差 `≤5°`、速度和角速度仍满足上述阈值，并由连续新 odom 证明持续 `0.5 s` 后，才释放至原安全目标并从当前真实状态重新规划。规划器返回的条件轨迹不直接发布执行；原圆、临时安全目标和已有有效圈数均保留。输入陈旧或临时安全点失效会重置相关前置条件。

ROS 适配层对携带 `departure_pose` metadata 的响应统一禁止直接接管其中的条件轨迹，不因状态机已消费过相同出发姿态而放行。相同条件响应重复出现时，对当前地图记录 `NO_PATH`，进入经过验证的安全制动；收到新地图后再重新选择安全目标。这样即使重复响应到达，也必须遵守实测停稳、对齐和重新规划的执行顺序。

当停车点缺少安全原地转艏空间、规划证明后退再转艏可行时，执行 `BRAKING_ESCAPE → BACKING_ESCAPE → ALIGN_ESCAPE → AVOIDING`。先实测停稳并冻结位置和艏向，以固定艏向直接保持控制后退。后退目标位置误差须 `≤min(0.05 m, position_tolerance/10)`、平面速度 `≤0.04 m/s`、角速度绝对值 `≤0.02 rad/s`、艏向误差 `≤5°`，由连续新 odom 证明持续 `0.5 s` 后，再从实测位置转艏并对原目标重新规划。原任务、原目标及有效圈数不因后退而改变；后退条件轨迹同样不直接接管执行。

Otter 仅有左右固定纵向推进器。阵位保持使用 `RECOVER → ALIGN → HOLD`：位置恢复时选择前进或反推中较小的转艏方向，位置收敛后调整目标艏向，再持续进行纵向位置和艏向 PD 控制。横向误差超限时重新回位，期间允许暂时偏离目标艏向。持续横流场景下，不能承诺严格同时固定二维位置和艏向。

保持控制默认恢复速度上限为 `0.3 m/s`，请求纵向力与艏摇力矩分别限于 `±20 N`、`±8 N·m`，左右归一化保持推力请求各限于 `±0.4`，普通推力变化率不超过 `0.6/s`，侧向推力恒为零。保持控制从导航接管时继承唯一实际推力出口的最后命令；该命令可能处于更宽的 `[-1,1]` 范围，因此过渡期实际输出允许暂时超出 `±0.4` 的保持请求包络，再以规定变化率连续接近保持请求。安全预测始终检查实际将发布的输出。取消、陈旧数据和安全制动可绕过普通变化率限制，立即输出零或经过验证的制动指令。普通 PID/LQR 导航阶段仍按其既有输出规则执行，不能将保持请求限幅视为所有阶段统一限幅。

默认 `0.5 m` 位置精度下，进入对艏阶段要求位置误差不超过 `0.15 m`、速度不超过 `0.04 m/s`；位置误差超过 `0.35 m` 或艇体系横向误差超过 `0.25 m` 时重新恢复位置。其他精度按相同比例缩放。每个直接保持控制候选都会检查一个控制周期及后续制动的低速 3-DOF 预测轨迹，使用 `2.2 × 1.2 m` 船体和默认 `0.1 m` 外扩裕量；候选不安全时尝试同样经过检查的制动指令，仍不可用则零输出并请求安全退出。

`navigation_status.mission` 包含任务编号、模式、阶段、原经纬度与本地位姿、临时目标、位置/艏向/径向误差、有效圈数、用时、区域内外状态和阻塞原因。`control_phase` 表示直接保持控制的内部阶段，`safety_reason` 给出保持控制安全门的原因。以下常见状态用于判断当前步骤，最终验收仍以原始观测为依据：

| 阶段 | 含义 |
|---|---|
| `RETURN_CENTER` / `STANDBY` | 回中心／待机值守 |
| `ENTERING_ORBIT` / `ALIGN_ENTRY` / `ORBITING` | 切入规划／切向对齐／连续绕圆 |
| `APPROACH_STATION` / `STATION_KEEP` | 远距接近阵位／近场保持 |
| `BRAKING_ESCAPE` / `ALIGN_ESCAPE` | 为逃逸先制动至实测停稳／在冻结的实测点对准出发艏向，满足稳定条件后重新规划 |
| `BACKING_ESCAPE` | 保持实测停车时冻结的艏向后退，到位且低速稳定后从实测位置转艏 |
| `AVOIDING` / `WAITING_CLEARANCE` | 执行安全退出或绕行／在安全点等候解除条件 |
| `WAITING_INPUT` / `AVOIDANCE_BLOCKED` / `BLOCKED` | 等待有效输入／未找到安全出口／任务阻塞，结合原因字段诊断 |

RViz 中的 `mission_markers` 显示中心、值守边界、名义圆轨迹、目标艏向和阶段；切入与绕行路径沿用 `global_path`、`local_trajectory`。

## 5. 迁移前冻结版本的复现方法与并行矩阵

本节及第 6 节保留迁移前版本的验收方法、脚本命令和真实结果。下面的旧脚本应在 [历史交付补丁](validation/station-guard-001/feature.patch) 复现的独立工作区中运行；这些命令不是当前迁移后入口的验收说明。迁移后的构建、入口验证和结果见 [任务模块迁移记录](station-guard-migration.md)。

迁移前的源码回归、安装审计与 Gazebo 闭环分别记录。在旧冻结工作区及其安装区已加载的终端运行：

```bash
cd /absolute/path/to/workspace
bash scripts/validation/run_source_tests.bash
python3 scripts/validation/verify_install.py \
  --source "$PWD" --install "$PWD/install" --output /tmp/mission-install-audit-001.json

# 单场景 smoke；harness 会自行启动和关闭它拥有的 Gazebo。
python3 scripts/validation/mission_acceptance.py \
  --workspace "$PWD" --output /tmp/mission-station-smoke-001 \
  --scenarios station --repeats 1 --domain-id 191
python3 scripts/validation/plot_missions.py /tmp/mission-station-smoke-001
```

不要先手工启动同一试验域中的仿真。Harness 在运行前检测既有仿真，在指定测试区域检查 lethal/unknown 栅格；环境不满足要求时记录 `blocked`，不能解释成受控避障测试通过。每个 trial 使用独立启动的 Gazebo 进程组、独立 ROS domain 和 Gazebo 端口，只清理自己创建的进程与实体。

正式矩阵包含下列 **7 个场景，每个重复 3 次，共 21 次独立 trial**：

| 场景名 | 固定输入及受控操作 |
|---|---|
| `station` | 目标本地 `(25, -8) m`、真北艏向 0°；收敛后评分 |
| `station_disturbance` | 同一阵位，先完成初始评分；同位置通过 `SetEntityState` 注入艇体横向 `0.4 m/s` 速度脉冲，再次收敛和评分 |
| `standby` | 中心 `(25, -10) m`、值守半径 `3 m`；待机后实际移到半径外，验证返中心 |
| `standby_obstacle` | 同一待机任务；近场出现真实圆柱，保持圆柱直到到达安全点低速等候，再移除并验返回 |
| `orbit_cw` / `orbit_ccw` | 圆心 `(23, -10) m`、半径 `8 m`、航速 `0.5 m/s`、径向精度 `0.5 m`、一圈；完成后返中心待机 |
| `orbit_obstacle` | 同一顺时针圆轨迹；前方出现圆柱，保持至实测绕过并重新在原圆巡航，随后移除并继续完成任务 |

所有 trial 初始位姿为 `(23, -10, 0)`。实测矩阵的圆半径和圈数是固定试验配置，与安装目录中供人工操作的 `10 m`、三圈示例不同。

正式矩阵统一使用 `mission_matrix.py` 调度。它默认包含全部七个场景、每个三次，最多同时运行三个独立 trial；父进程不初始化 ROS，子任务各自启动并管理仿真。

```bash
cd /absolute/path/to/workspace
source /opt/ros/humble/setup.bash
source install/setup.bash
mission_results="/tmp/mission-matrix-001"

python3 scripts/validation/mission_matrix.py \
  --workspace "$PWD" --output "$mission_results" \
  --jobs 3 --repeats 3 --domain-id 170

# matrix 的每个子目录是一项独立 harness 运行，分别生成其图表。
for mission_trial_run in "$mission_results"/*/; do
  python3 scripts/validation/plot_missions.py "$mission_trial_run"
done
```

默认矩阵使用 ROS domain `170–190`；即使 `--jobs` 少于 21，每个 trial 仍分配不同的域。机器资源不足时可将 `--jobs` 改为 1 或 2，场景及验收判据保持不变。不要再同时手工启动相同正式矩阵的分组队列。若运行其他 ROS 实验，应使用完全不重叠的域范围，并保证最大域编号不超过 232。

调度器通过运行根目录的 `progress.json` 提供各项 `running`、`passed`、`failed`、`blocked`、`not_run` 数量；最终汇总写入根目录 `summary.json`。出现失败、阻塞或指纹不一致时停止派发新 trial，已经运行的独立 trial 继续结束，未派发项保留 `not_run`，因此不能把“进程结束”解释为“21 次已完成”。调度器核对每个子任务的场景、域、结果及前后源码/安装指纹。重复编号以矩阵汇总的 `matrix_repeat` 为准，内部单项 harness 固定使用 `repeat=1`，原始子任务证据不被改写。

输出目录必须不存在，由脚本创建；重新运行须使用新目录。只预览正式队列可在上述 matrix 命令末尾加 `--dry-run`，并另选新的输出目录。

`--dry-run` 仅生成运行计划和 `not_run` 摘要，不启动 ROS/Gazebo，也不代表通过。正式运行期间保持同一源码和安装区不变；在该安装区重建、修改源文件或替换已安装模块会破坏指纹一致性。

## 6. 迁移前正式验收判据与证据阅读

阵位测试要求从每次开始或扰动后，在 `180 s` 仿真时间内同时达到位置误差 `≤0.25 m`、艏向误差 `≤3°`、平面速度 `≤0.04 m/s`，连续保持 `5 s`。随后固定采集 `60 s`，至少 `95%` 原始 odom 样本同时满足位置误差 `≤0.5 m`、艏向误差 `≤5°`。窗口中超限不会重置评分起点；缺失连续观测会使窗口不完整。横向速度脉冲必须从新 odom 中得到确认，记录请求的世界速度及实际艇体横向速度；没有观察到有效脉冲时判 `blocked`，不使用位置瞬移替代脉冲证据。

待机初次到达、越界后返回及绕圆结束后返回，均需观察到 `STANDBY`，并由独立原始 odom 证明位置误差不超过请求精度、速度不超过 `0.1 m/s`，连续 `1 s`。待机无扰动片段还检查实际推力指令为零，不能只凭状态标签判断已驻留。

绕圆的正常 `ORBITING` 样本径向误差 P95 不超过 `0.5 m`。验收脚本独立按指定方向对原始 odom 极角作有符号净增量累计，仅连接正常巡航、位于精度带内、时间连续且无不合理跳变的相邻样本；不累计避障和跨阶段弧段。独立累计圈数必须满足请求，不能仅依据节点的 `orbit_completed` 或 `completed_laps`。

状态约以 1 Hz 观测，阶段切换时刻存在采样不确定性。脚本记录状态源时间戳及边界间隔，以运行节点的 `dynamics.velocity_max` 作为速度上界，采用

\[
\varepsilon_\theta=\min\left(0.15,\;\frac{2v_{\max}}{R}\,\Delta t_{\mathrm{boundary,max}}\right)
\]

作为独立净角与请求圈数的有限缺口容差，单位 rad。相关 odom 间隔超过 `0.5 s`、相关状态间隔超过 `1.5 s` 或检测到不合理位置跳变时验收失败，不能通过放大容差掩盖缺样。原地停留、反向行驶、提前返回和把避障弧误计成巡航圈数均不能满足该判据。

仿真时钟暂停时，墙钟定时器可能重复发布同一源时间戳、同一阶段的状态。评分器折叠这种相邻重复快照并记录重复数；同时间戳改变阶段、负时间间隔和超长间隔仍判异常。该处理不改变原始 odom 净角，也不放宽 `0.15 rad` 容差上限。

障碍为 Gazebo 中真实创建的半径 `0.5 m`、高度 `4 m` 圆柱，须在实时代价地图中观察为 occupied/lethal。待机场景保持障碍，直到 `WAITING_CLEARANCE` 且实际到达所选临时安全点、低速持续 `1 s`，再删除障碍并验返中心。绕圆场景必须保持障碍至独立观测证明绕过并恢复原圆巡航；仅离开原位置 `1 m` 不构成恢复证据。

`recovery-iKvKeO` 中的新版绕障评分以完整船体曾越过障碍切向下游平面作为越障事件，并保留该事件。令 `t` 为障碍相位处沿规定绕行方向的单位切向，`q=(p-p_obstacle)·t` 为艇中心相对障碍的下游投影，`h` 为船体沿 `t` 的投影半宽；要求 `q-h>0`，即整艇进入下游半平面。此后还必须满足当前处于原圆精度带、切向艏向误差 `≤15°`、速度 `≥0.1 m/s`、阶段为 `ORBITING`，连续驻留 `1 s`；从障碍生成至当前的 odom 间隔须 `≤0.5 s`，全程完整船体扫掠间隙须 `≥0.2 m`。全部满足后才删除障碍，恢复期限仍为 `180 s`。离圆避障的净极角仅保留作诊断，不用来否定已观测的下游越障；正常精度带内的独立有效计圈算法不变。该修正与原 matrix004 评分器的版本边界见验证报告。

受控圆柱存在期间，完整 `2.2 × 1.2 m` 船体与其最小间隙要求 `≥0.2 m`；相邻 odom 间使用保守扫掠估计。非有限或超出 `[-1,1]` 的归一化推力，以及负扫掠间隙，会终止采集并触发清理。所有场景还核对原任务编号、模式、经纬度和艏向没有因避障被改写。

独立 harness 运行根目录包含 `manifest.json`、`summary.json`，其 trial 子目录包含 `launch.log`、原始 `events.jsonl` 和 `metrics.json`。matrix 将每项独立运行置于对应场景及重复编号的子目录，并在矩阵根目录额外提供统一 `progress.json` 和汇总 `summary.json`。其中保留任务请求、状态变化、每条 odom/推力、受控实体操作、固定评分窗口、独立计圈及源码/安装文件指纹。绘图根据这些日志生成轨迹、位置/径向与艏向误差、推力图；绕圆额外生成独立净圈数图。推力图保留原始墙钟时间轴，与 odom 图的仿真时间轴区别标注。

结果按 `passed`、`failed`、`blocked` 区分：`passed` 需要该 trial 全部实测条件满足；`failed` 表示执行中至少一个验收条件不满足；`blocked` 表示启动、测试水域、服务、传感器观测或受控扰动条件未建立。后两者均使运行返回非零退出码。结论应同时核对计划 trial 数、实际完成数、全部通过标志及源码/安装指纹一致性。

迁移前最终冻结构建 `validation/priority-IzMlsY` 的 713 项源码测试、构建及 14 包安装审计通过，完整源码测试耗时 `13.15 s`，包含候选分组优先级修复；当时新增八项回归在旧实现失败、修复后通过。此前已实现后退再转艏、条件轨迹接管和越障评分修复。原导航六项回归和部分任务初测已有闭环证据。早期待机与绕圆 smoke 使用的是强化独立观测判据之前的脚本，仍保留为诊断记录。绕圆障碍 `smoke004` 原判失败仅因同阶段同时间戳状态重复；修正评分器后，对不变的原始日志另存 `rescore.json`，全部测量判据通过。这是离线重评分，不是新增一次 Gazebo 重跑，原失败判定与日志不覆盖。

正式 `matrix001` 因返中心安全点的转艏包络不足而手动中断，归档为 2 项 `passed`、3 项 `blocked`、16 项 `not_run`。旧式 `hypot(L/2, W/2)+5` 未包住旋转外扩矩形且遗漏到达精度，现已按第 4 节公式修正。随后在独立 WSL 副本 `/home/usv/usv_migration/station-guard-001` 冻结启动的 `matrix002`，因复审发现重复条件规划响应可能被直接接管为执行轨迹而手动中断并完成自有实例清理，归档为 0 项 `passed`、0 项 `failed`、3 项 `blocked`、18 项 `not_run`。三次均实测恢复 `ORBITING`，但全流程未完成，不算正式通过。该轮对应历史 `envelope-en3qIF`、653 项源码回归版本，不改写成后续修复版本的结果。

`matrix003` 已自然结束，结果为 2 项 `passed`、1 项 `failed`、0 项 `blocked`、18 项 `not_run`，完整矩阵未通过。`orbit_obstacle` 的 trial01、trial03 完整通过，trial02 在受控绕障后的 `180 s` 期限内未满足实测回圆驻留判据，长期报告 `NO_SAFE_EXIT`；出现失败后停止派发其余 18 项。该结果属于冻结的 `handoff-fNJc74`、656 项源码回归版本，说明源码测试通过不能替代闭环验收。

现场数值诊断已确认，trial02 的两组真实状态／地图中仍有合法回圆候选；扩大为整圆搜索也不能得到通过扫掠检查的直接 LOS 路径，停车后的原地转艏同样失败。沿当前艏向反向后退 `0.5 m`，再转艏并对原目标规划 LOS，在两组输入上均通过数值检查。`0.5 m` 是离散候选 `[0.5, 1, 2, 3] m` 中最短的被测可行值，不是理论最短距离。

第 4 节的 BACKING_ESCAPE 流程已实现。上述两组原始输入的数值重放均返回成功，选择后退 `0.5 m` 且保留原目标，耗时分别为 `31.9 ms`、`26.5 ms`；这不是新仿真。随后 `matrix004` 在 `backoff-T510NA` 冻结副本自然结束，2 项通过、1 项失败、18 项未运行；两个通过的 trial 均实际使用了 BACKING_ESCAPE，但完整矩阵仍未通过。

matrix004 的 trial02 暴露两个独立问题。原评分器把离圆避障的净极角作为越障条件，而路径曾接近圆心至 `0.01748 m`，该净极角为负，错误拒绝了 `137.4–138.4 s` 已满足的下游回圆窗口，导致障碍继续保留。后续 `254.6 s` 状态／`253.9 s` 地图确有恢复候选遗漏：原 `15°–180°` 搜索无合法点，扩展至 `15°–345°` 后有 11 个合法点，按既有排序和四候选预算完成规划。候选修复新增四项回归在旧实现失败、修复后通过；两项修复均已进入 recovery-iKvKeO 冻结版本。

旧 trial 的后续有效圈数与返中心没有完成，因此 `137.4–138.4 s` 的 partial rejoin rescore 仅验证局部回圆窗口及评分修正，不能把整次 trial 改为通过。控制扫掠 `0.1 m` 裕量、解除区 `5 m` 裕量和 `180 s` 恢复期限均不变。

`matrix005` 随后在 recovery-iKvKeO 冻结副本运行，三次均因全圆候选混排选中上游点而反复绕障，已对自有矩阵父进程发送 SIGINT 并正常结束，保留 0 项通过、0 项失败、3 项阻塞、18 项未运行及实际循环日志。三张初始地图各有 7 个合法候选；前半圆仅有 `180°` 点，而按距离混排的前四个点为 `270°/255°/240°/225°`。worker 选中 `270°` 点，全艇下游投影约 `−8.4 m`；单独使用原 `180°` 点也能成功规划，下游投影为 `+1.36–+1.67 m`。

第 4 节的候选分组回退修复已实现并冻结。三张 matrix005 初始图均选择 `180°` 下游点并成功规划，matrix004 圆东侧输入仍可通过后半圆回退成功恢复规划；这些原图重放属于数值证据。四候选上限、`1 s` 规划预算及安全裕量均不变。随后 `matrix006` 在 priority-IzMlsY 冻结副本完成全部 21 次独立运行，退出码 0，21 项通过、0 项失败／阻塞／未运行，`source_unchanged=true`、`installation_unchanged=true`。

正式结果中，阵位及阵位扰动共九个固定 60 s 窗口、每窗 600 个样本，联合达标率全部为 100%；评分窗口最大位置误差 0.084887 m、最大艏向误差 0.258537°。九次绕圆的径向 P95 均不超过 0.143623 m，独立净圈数均达到一圈；三次绕圆障碍最小船体间隙为 0.465323–0.584453 m，全部待机及绕圆返回的最大中心误差为 0.477636 m，并通过低速连续驻留判据。完整七场景结果、重复范围、[代表图件来源](validation/station-guard-001/figures/figure_sources.json) 和版本指纹见 [验证报告](station-guard-validation.md)。

matrix001–005 的全部历史失败、中断及重评证据保留，未并入 matrix006 的 21 次统计。迁移前可应用 [源码补丁](validation/station-guard-001/feature.patch)、[逐次指标](validation/station-guard-001/metrics-summary.csv)、[原始证据压缩包](validation/station-guard-001/raw-evidence.tar.gz) 和图件均由 [历史交付入口](validation/station-guard-001/README.md) 索引。旧补丁已验证可应用且应用后的树与旧冻结源码一致；它保留迁移前的集成位置，不应作为当前 core 任务架构的安装补丁。当时共享根目录四个 sim_env 文件中的并行雷达差异未被覆盖，该历史结论不覆盖额外改动或本次迁移。历史诊断见 [调试记录](station-guard-debugging.md)，迁移后证据见 [任务模块迁移记录](station-guard-migration.md)。

