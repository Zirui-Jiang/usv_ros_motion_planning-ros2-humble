# ROS 2 / Gazebo 算法迁移验收

本目录运行真实系统并保存观测。`--dry-run` 只输出计划，明确标记 `not_run`。当前脚本开发阶段已做 Python 语法检查与几何单元测试；是否通过 Gazebo 验收，以每次运行生成的 `summary.json` 为准。

## 调用方式

在 WSL Ubuntu / Linux 中先加载 ROS 2 Humble 与已构建工作区。`PY_WS` 和 `CPP_WS` 都应是含 `install/setup.bash` 的绝对路径；C++ 基线工作区与 Python 工作区分开构建。

```bash
source /opt/ros/humble/setup.bash
export PY_WS=/absolute/path/to/python_workspace
source "$PY_WS/install/setup.bash"

# PID / LQR × 直线 / 转弯 / 单圆柱绕行 × 各 3 次，共 18 个独立 launch。
python3 "$PY_WS/scripts/validation/ros_acceptance.py" \
  --workspace "$PY_WS" --mode python --costmap-topic /otter/costmap_raw \
  --output /tmp/usv_acceptance_python_001

# 先用 1 次直线检查环境与接口。
python3 "$PY_WS/scripts/validation/ros_acceptance.py" \
  --workspace "$PY_WS" --controllers pid --scenario straight --repeats 1 \
  --output /tmp/usv_acceptance_smoke_001

# 候选参数校准：每次首个goal前原子设置并读回；整次导航期间保持固定。
python3 "$PY_WS/scripts/validation/ros_acceptance.py" \
  --workspace "$PY_WS" --controllers pid --scenario single_obstacle --repeats 3 \
  --costmap-topic /otter/costmap_raw \
  --parameter usv_simulation.trajectory_planner.los.switch_dist=1.0 \
  --output /tmp/usv_acceptance_switch1_candidate_001

# 诊断模式：首次NO_PATH/map-invalid即保存当前输入并结束该例，不计通过。
python3 "$PY_WS/scripts/validation/ros_acceptance.py" \
  --workspace "$PY_WS" --controllers pid --scenario single_obstacle --repeats 1 \
  --costmap-topic /otter/costmap_raw --stop-on-reference-failure \
  --output /tmp/usv_acceptance_first_failure_001

# 两个工作区的同场景比较，共 36 个导航试验，再加故障用例。
export CPP_WS=/absolute/path/to/cpp_baseline_workspace
python3 "$PY_WS/scripts/validation/ros_acceptance.py" \
  --workspace "$PY_WS" --baseline-workspace "$CPP_WS" --mode both --faults \
  --output /tmp/usv_acceptance_comparison_001

# 只跑目标替换、取消，以及 Python 的受控 odom 断流。
python3 "$PY_WS/scripts/validation/ros_acceptance.py" \
  --workspace "$PY_WS" --only-faults \
  --output /tmp/usv_acceptance_faults_001

# 不加载 ROS、不启动 Gazebo，只检查矩阵与配置。
python3 "$PY_WS/scripts/validation/ros_acceptance.py" \
  --workspace "$PY_WS" --dry-run --output /tmp/usv_acceptance_plan_001
```

输出目录必须不存在，防止覆盖已有实验结果。退出码为 `0` 仅表示所请求且实际运行的用例全部通过；`blocked`、`unsupported` 或失败都会使 live run 返回非零。`--dry-run` 成功只表示计划生成成功。

每次导航试验运行 `ros2 launch sim_env main.launch.py world:=ocean gui:=false rviz:=false controller_name:=...`，并指定起点与 namespace。只终止本脚本 `start_new_session=True` 建立的子进程组，不使用 `killall`、`pkill` 或现有终端。默认使用隔离的 `ROS_DOMAIN_ID=182`、本机通信，以及每次独立的 Gazebo master 端口；`--domain-id` 可调整。若发现该域已有 `/otter/odom` 发布者，该例中止，不复位其他仿真。

## 基线注意事项

脚本自动发现 `/otter/.../costmap_raw` 的 `nav2_msgs/Costmap`；没有 raw 话题时尝试包含 `costmap` 的 `nav_msgs/OccupancyGrid`，也可用 `--costmap-topic /absolute/topic` 指定。最终选用的话题和消息类型写入 JSONL。

旧 C++ `main.launch.py` 如果没有真正接入 `controller_name` 参数，不能仅凭 launch 命令认定 LQR 已运行。脚本通过 `/otter/usv_simulation/get_parameters` 校验实际 `usv_simulation.controller.controller_name`，不匹配就记 `blocked`。可用 `--baseline-launch-file validation.launch.py` 指向基线工作区中已安装、已接好算法选择的 launch 文件；该文件应启动同一 Gazebo/传感器/算法链。脚本不会修改基线源文件或用户 YAML。

原 C++ 没有 `/otter/cancel_navigation` 时记为 `unsupported`，不伪造取消成功。odom 断流是针对 Python 调度节点的独立测试：通过 `ros2 run sim_env sim_env_node` 启动独占 namespace，持续发布空闲 raw costmap 和静止 odom，观察非零推力后仅停止 odom，costmap 保持新鲜。它不依赖 Gazebo，也不切断其他节点的数据。

## 场景与环境预检

固定配置在 `scenarios.json`。所有场景从 `(23, -10, 0 rad)` 开始：直线与单障碍目标为 `(35, -10)`，转弯目标为 `(23, -22)`；单障碍是在 `(29, -10)` 通过 Gazebo `SpawnEntity` 生成的静态圆柱，半径 `1 m`、高度 `4 m`、底面 `z=-1 m`。每次试验重新启动自己的 launch，并通过 `/gazebo/set_entity_state` 将艇位姿与速度复位；若存在 `/reset_world` 或 `/gazebo/reset_world`，也调用它。不会改写 world 文件，也不删除原 world 中的物体。

已检查 `ocean.world` 的原始 box 碰撞几何：

| 物体 | 中心 (m) | 平面尺寸 (m) |
|---|---|---|
| BOX_0 | (5, -2) | 10 × 20 |
| BOX_1 | (-35, -20) | 10 × 20 |
| BOX_2 | (105, -9) | 5 × 50 |
| BOX_3 | (55, -9) | 5 × 30 |

上述测试走廊避开这些原始 box 的碰撞范围。`sandisland` 网格的原始 XY 范围很大，不能依据其模型中心断言局部水面畅通，因此每次 **在 spawn 前** 检查实际 costmap 中 nominal route 周围 `3 m` 半宽走廊：不允许 lethal/unknown cells；若有干扰，记 `blocked` 并保存位置，不把原场景障碍算成受控测试障碍。圆柱 spawn 后还会检查其周围确实出现 occupied cell。原场景中的非受控障碍没有精确的实时碰撞计量，因而脚本的数值几何间隙只针对明确 spawn 的圆柱。

## 通过判据和记录语义

阈值是显式工程验收默认值，均保存在每次 `manifest.json` 与 `trial_started` 事件中；它们不是论文实验结果或性能保证。修改阈值应创建新的配置与输出目录，不覆盖旧结果。

| 指标 | 默认判据 |
|---|---|
| 到达 | 距离目标 ≤ **运行中控制器的 `goal_distance_tolerance`**（默认 PID 1.5 m / LQR 1.0 m），平面速度 ≤ 0.25 m/s，持续 1 s wall time |
| 超时 | 180 s simulation time 或 300 s wall time |
| 最大横向误差 | 相对该目标第一次有效全局路径 ≤ 4 m |
| 单障碍最小间隙 | 艇完整 2.2 × 1.2 m 矩形 footprint 与圆柱间 ≥ 0.2 m |
| 推力合法性 | 三个话题齐备、有限且归一化命令在 [-1, 1] |
| 取消 | 接受 Trigger 后 0.5 s 内三路收到零命令，随后观察 1 s 不再出现非零 |
| odom 断流 | state timeout 设置为 0.4 s；停 odom 后 0.7 s 内三路零推力，之后观察 1 s |

几何检测将圆柱中心变换到艇体坐标系，计算圆与有向矩形的有符号间隙，覆盖艇首、艇尾与四角。还对相邻 odom 的位置和最短 yaw 插值，样本间距不超过 `0.05 m / 2°`，并减去半个采样区间的平移加船角旋转上界，得到对该插值轨迹保守的间隙。它不重建两次观测之间未知的真实动力学，也不替代物理接触传感器。

配置中的 `arrival_distance_m=1.75` 仅作预备记录；live 判定必定由节点实际参数覆盖，并单独记录 `effective_arrival_threshold`。读取不到实际容差时中止，不能以 1.75 m 放宽通过条件。

每例输出 `launch.log`、`events.jsonl`、`summary.json`，总目录另有 `manifest.json` 与总 `summary.json`。JSONL 保存 odom 时间戳/坐标系/位姿/体轴速度、全局参考路径、局部轨迹、每条推力、目标、navigation_status 与 mapping_status 原始 JSON、到达状态、固定参考与实时参考横向误差，以及 footprint 最小间隙。目标替换后以抵达新目标附近的第一条路径作为新的固定参考，避免把旧路径当作新目标参考。

每次发布目标前还保存真实 costmap NPZ（数据、分辨率、origin、frame、stamp、raw/occupancy 类型）与同名标识的自船状态 JSON，供失败例离线重放；它们由 `goal_input_snapshot` 事件索引。`navigation_parameter_event` 记录节点参数事件，特别是诊断过程中实际发生的在线参数修改。正式对比应使用从启动即固定的配置；中途改变参数的 run 应单列为诊断。

每次进入 `NO_PATH`、`WAITING_REFERENCE` 或地图失效状态时，还会以 `navigation_failure_snapshot` 索引即时保存最新 raw/occupancy 地图、状态、路径及最后非空轨迹；恢复 `NAVIGATING` 后再次失败会生成新快照。各订阅的源时间戳单独保留，不能将异步回调快照当成规划线程内部原子输入。`--stop-on-reference-failure` 会将该例标为 `diagnostic_stopped`、`passed=false`，适合捕获后立即开展离线诊断。

`--parameter NAME=JSON` 可重复给出。值会在每次导航试验的任何目标发布前通过 ROS 原子参数服务设置并读回核对，分别记录 `pre_goal_parameter_configuration`、`pre_goal_parameter_readback` 和 manifest；任一拒绝或读回不一致均中止该例。应使用与节点声明匹配的 JSON 类型（例如 `1.0` 是浮点数）。这只支持节点本身允许在线设置的参数，不替代 `controller_name` 的 launch 选择。

需要改变必须重启生效的控制参数时，使用 `--controller-config /absolute/path/to/controller_params.yaml`。harness 将绝对路径透传为 launch 的 `controller_config` 参数，在 manifest 保存文件 SHA-256，并在每次 launch 前重新核对与记录；配置文件在矩阵中途改变时，该例会中止。`controller_verified` 同时采集 PID/LQR 的 `goal_speed_tolerance`；旧 C++ 未声明此字段时记录 `null`，不推测其实现。

加载 Python 工作区后，可用以下命令对真实 raw 快照比较不同 LOS 前视距离。脚本拒绝把 OccupancyGrid 当成无损 raw 数据，保留地图、自船状态和首条路径各自时刻，并记录完整八字段预测轨迹。预发布快照与规划线程稍后的私有输入时刻可能不同，因此输出属于捕获输入重放；输出中的几何间隙是预测轨迹指标，闭环实际间隙仍以原验收日志为准。

```bash
python3 "$PY_WS/scripts/validation/replay_los_snapshot.py" \
  /absolute/path/to/python_pid_single_obstacle_01 \
  --lookaheads 5 1 --cylinder 29 -10 1 \
  --output /absolute/path/to/new_los_replay.json
```

延迟记录包括目标发布至首条有效路径/首条非零推力的 wall time、推力接收间隔，以及 ROS 仿真时间下消息 header age。`observed_odom_receipt_to_command_receipt_ms` 只是两个回调的接收时间差，不能当作算法运行耗时或源端到执行器端的端到端延迟。节点自行发布的实际计算 metrics 原样保存在 `navigation_status` 中。

`acquisition_finished` 在导航采集循环结束、汇总分布计算和 JSON 落盘之前记录；中断或运行错误也保留对应结束标记。性能覆盖区间应以该事件为采集终点，不能把 `trial_finished` 前的汇总和进程清理耗时算作缺失状态数据。

## 源时间对照与图表

旧比较默认采用 `legacy_wall`，即 summary 中按墙钟循环采样的 RMSE。仿真暂停时
同一源状态会被重复计入。新格心终点复验在采集前固定选择 `source_time_trapezoid`，
按实际 odom 源时间对误差平方作梯形积分；原墙钟指标同时保留，不回写旧报告。
只有有证据的本项目 map/odom 恒等变换才能选择以下 frame 契约。

```bash
python3 "$PY_WS/scripts/validation/compare_results.py" \
  --python /home/usv/usv_migration/results/python-grid-endpoint-final-matrix-001 \
  --baseline /home/usv/usv_migration/results/baseline-grid-interior-001 \
  --rmse-metric source_time_trapezoid \
  --frame-contract otter_fake_localization \
  --output /tmp/usv_source_time_comparison_001

python3 "$PY_WS/scripts/validation/plot_acceptance.py" \
  --report /tmp/usv_source_time_comparison_001/report.json \
  --output /tmp/usv_source_time_figures_001
```

这两个命令应在全部采集结束后分别执行。比较器缺少有效配对或发现失败时返回非零，
仍可运行图表导出以保留失败证据，不能修改 `all_passed` 来获取图表。
源间隔和首尾覆盖差固定不超过 0.5 s；缺少 frame 证据、同时间戳矛盾位姿、时间回退
或覆盖不足均为 unknown，不用墙钟值顶替所选主判。所有输出目录必须是新目录。
图和 CSV 标明所选度量，并列两种 RMSE、源样本数、实际跨度、间隔和边界缺口。
规则、历史失败和适用范围见
[固定协议](../../docs/validation/grid-endpoint-source-time-protocol.md) 和
[图表说明](../../docs/validation/plot-acceptance-guide.md)。

## 独立定位与建图检查

定位检查启动本脚本自有节点，验证畸形输入拒绝、速度坐标转换、重复/回退 clock
及 SIGINT 退出。建图吞吐脚本用单调时钟产生仿真 /clock，发送 450 帧、2000 束的
15 Hz 扫描，并先确认各扫描的 TF 已可查；它运行真实 Python mapper，不运行 Gazebo。
两者都使用专用 ROS domain，应与正式性能采集分时执行。

```bash
python3 "$PY_WS/scripts/validation/localization_smoke.py" \
  --workspace "$PY_WS" --domain-id 188 --mode acceptance \
  --output /tmp/usv_localization_green_001

python3 "$PY_WS/scripts/validation/mapping_throughput.py" \
  --source-repo "$PY_WS" --domain-id 189 \
  --output /tmp/usv_normal_tf_throughput_001
```

吞吐脚本的 CPU/RSS 包括同进程的驱动和观测，不能解释为 mapper 独占消耗。
正常 TF 条件下通过也不能回写 Gazebo live TF 缺失、过期或观察不足的 unknown。
实测证据见 [定位边界记录](../../docs/validation/localization-boundary-001/README.md) 与
[建图吞吐记录](../../docs/validation/normal-tf-throughput-001/README.md)。

## 本地检查

```bash
python3 -m py_compile scripts/validation/ros_acceptance.py scripts/validation/metrics.py
python3 -m pytest scripts/validation/test_metrics.py -q
```

本目录中的 `run_source_tests.bash` 是主集成任务提供的全仓库 ROS/source 测试入口，与本验收 harness 独立。
