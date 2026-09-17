# 三模块普通导航拆分验证记录

验证日期：2026-09-13。分支：`codex/distributed-navigation`。范围：同机 ROS 2 Humble / WSL Ubuntu 22.04 / Gazebo 普通导航。

本次从用户当时的未提交工作树创建分支，保留所有已有修改与未跟踪文件。起始 HEAD 为 `81ab8ed1a228a23f8c1f0a1c071ef4acdfbe34dd`，它不能单独代表本次基线；基线还包括 [原工作树状态](baseline-records/source-status.txt)、[原差异](baseline-records/source.diff) 和 [文件清单](baseline-records/source-manifest.json)。未自动提交或推送。

本次首版改造与验证已完成。最终全源码 914 项通过；取消修复后的独立进程与任务/故障复验分别为 12/12 和 3/3，通过直接 Python 退出码 0 确认。测试运行器、业务测试节点及 Gazebo 均已清理。

## 实现与启动

感知定位保留原定位、mapping/mmwave 和 Costmap 节点；决策规划由 `decision_node` 运行；航控由 `control_node` 运行。感知身份监测用于识别原输出发布者重启。三个业务模块可独立启动，仿真支撑另设入口。路径与轨迹规划在决策模块内部继续传递对象；跨进程执行参考使用完整八列轨迹消息，航控独立订阅状态与地图。

本机已经构建的安装区为 `/home/usv/distributed-navigation-build-20260913/install`。在新的 WSL 终端执行：

```bash
source /opt/ros/humble/setup.bash
source /home/usv/distributed-navigation-build-20260913/install/setup.bash
ros2 launch sim_env main.launch.py headless:=true
```

分模块启动、YAML 路径覆盖、参数与话题见 [使用说明](../../distributed-navigation/README.md)。默认 A* → LOS → PID、原 LiDAR 模式和 `main.launch.py` 入口保持。真实测试均使用 headless 模式；GUI/RViz 启动配置有 launch 测试，本轮没有重新进行画面验收。

## 验证结果与证据层次

| 检查 | 结果 | 原始证据与适用范围 |
|---|---|---|
| 冻结基线构建 | 18 包通过 | [构建日志](baseline-records/build.log) |
| 冻结基线完整测试 | 846 passed | [日志](baseline-records/full-source-tests.log)、[JUnit](baseline-records/full-source-tests.xml)；使用下文的既有导出测试夹具环境 |
| 冻结基线代表场景 | 4/4 通过 | [PID 三场景](baseline-records/representative-pid-001/summary.json)、[LQR 直航](baseline-records/representative-lqr-001/summary.json) |
| 新系统完整构建 | 20 包通过 | [colcon 构建记录](build-records/)；消息包、导航包及已有仿真依赖 |
| 新系统完整源码测试 | 914 passed，无失败、无跳过 | [日志](source-tests-003/tests.log)、[JUnit](source-tests-003/tests.xml)、[运行参数](source-tests-003/command.json)；显式收集 `src scripts` |
| 干净安装环境审计 | 48 项通过 | [实际安装审计](install-audit/distributed-install-audit-final.json)；故意错误安装根 [被拒绝](install-audit/distributed-install-audit-mismatch.json) |
| 源码与已验证安装文件 | 71 项 SHA256 一致 | [最终核对结果](install-audit/source-install-final-002.json)；覆盖导航、兼容入口、算法 Python、启动、YAML 和消息定义 |
| 原计算逻辑保留 | 53 项通过 | [源码和 AST 对照](algorithm-preservation.json)；计算函数/类、配置与传感器解析逻辑 |
| 独立 launch 自定义 YAML | 4 项检查通过 | [实际参数读回与导出](yaml-runtime-001/summary.json)；详见下节 |
| 新系统普通导航矩阵 | 18/18 通过 | [完整矩阵](gazebo-matrix-001/summary.json)，PID/LQR × 3 场景 × 3 次 |
| 原矩阵任务/故障 | 3/3 通过 | 同上；故障结论优先使用下述增强断言复验 |
| 最终独立进程联调 | 12/12 通过，exit 0 | [分进程记录](process-smoke-004/summary.json)、[执行终态](process-smoke-004/execution-observation.json)；真实 ROS 进程、合成状态和地图 |
| 最终任务/故障复验 | 3/3 通过，exit 0 | [故障记录](faults-004/summary.json)、[执行终态](faults-004/execution-observation.json)；目标替换和取消使用 Gazebo，odom 断流使用受控 ROS 输入 |
| 毫米波模式 | 20/20 断言通过 | [报告](mmwave-001/summary.json)；几何雷达、地图、短距离目标、感知暂停与恢复 |

旧单节点仅在 `src/sim_env/test/legacy_reference.py` 中作为回归参照，不安装。上述完整源码测试包含这些旧参照测试；新节点测试、分进程运行和 Gazebo 矩阵分别提供不同层次的拆分证据，不把旧参照测试计作新进程联调。

基线仅跑了 4 次代表场景，本次结果证明既定场景的功能回归通过，不据此宣称拆分前后闭环统计等价或实时性提升。回调接收间隔是观测量，不等于算法 WCET 或源端到执行器端的端到端延迟。

主矩阵和毫米波试验完成后，最终审查修正了一处取消服务反馈匹配顺序：已验证会话的旧任务真实零报告可以完成对应的待处理取消请求，但仍不能更新当前任务状态。该修复不改变规划、控制、感知计算和参数加载。对应 3 项新增用例先复现 2 条失败，再与原 5 项集成测试一起通过，见 [红绿回归证据](cancel-overlap-fix/README.md)；最终安装区另行运行完整源码、分进程及任务/故障复验。主矩阵记录仍对应修复前的快照，保留原始结果而不改写时间顺序。

## 普通导航矩阵

固定场景定义和阈值见 [manifest](gazebo-matrix-001/manifest.json)。以下时间为达到验收条件的仿真时间范围，不是墙钟运行时间。

| 控制器 | 场景 | 成功次数 | 仿真时间范围 / s | 三次中的最小船体障碍净距 / m |
|---|---|---:|---:|---:|
| PID | 直航 | 3/3 | 20.8–21.5 | 无指定障碍 |
| PID | 转弯 | 3/3 | 20.6–21.2 | 无指定障碍 |
| PID | 单障碍 | 3/3 | 22.6–23.3 | 0.689 |
| LQR | 直航 | 3/3 | 16.2–16.3 | 无指定障碍 |
| LQR | 转弯 | 3/3 | 17.2–18.0 | 无指定障碍 |
| LQR | 单障碍 | 3/3 | 18.5–18.7 | 0.871 |

## YAML 与接口兼容

四份原项目 YAML 被复制到新位置并修改为：质量 31.25、Dijkstra、LQR、控制频率 8 Hz（通过原 `control_frequncy` 别名）、障碍因子 0.35、PID 增益 3.25、LOS 最大转率 0.23、命名空间 `yaml_probe`。分别实际启动 `decision.launch.py` 与 `control.launch.py`。

- 两进程启动日志中各 86 项有效参数逐项一致；[决策日志](yaml-runtime-001/decision.log)、[航控日志](yaml-runtime-001/control.log)。
- 通过两个真实参数服务读回 8 个代表值，与预期全部一致；随后将热参数修改为 4.75，实际航控节点读回成功，旧 `export_parameters` 导出成功。
- 该独立 YAML 探针选择 LQR，PID 键的修改证明参数传播。PID 算法对象实际更新且不清积分，由新 `test_control_ros.py` 和 `test_distributed_ros.py` 的真实 ROS 用例验证。
- 原格式、默认值、别名、空覆盖保留 YAML、命令行覆盖顺序以及 LiDAR/mmwave 参数组均有 launch 测试。

`goal/PoseStamped`、`cancel_navigation/Trigger`、`navigation_status/String`、两种显示 Path 和原三路 Float32 推进器话题保持。状态保留原字段，新增模块身份与实际航控进程统计。当前碰撞检查职责符合 Q10：决策检查轨迹；航控周期检查当前位置船体占用。

## 重启、断流、取消和仿真时钟

增强分进程测试记录不同的模块 PID 和三路各一个推进器发布者。决策、航控、感知依次重启后旧任务保持失效，重新下发目标可恢复；重建实际 odom 发布者也会触发感知身份变化。这里的“感知进程”包含身份监测；实际感知模块由原多个节点组成，整组通过独立 launch 启动。

取消后重放旧参考时，测试短暂暂停自有决策进程，在其心跳尚未超时的窗口内检查旧参考拒绝计数增加、已取消任务编号及序号不回退、状态保持 `CANCELLED`，避免靠持续发送取消参考掩盖错误。

增强故障测试要求每轴至少 3 个有限新零输出样本，窗口首尾和内部接收间隔均不超过 0.35 s，同时检查运行进程存活。最终 `faults-004` 取消停推观测为 0.007972 s，odom 断流为 0.433820 s；约 1 s 的尾部窗口每轴至少 9 个新零样本，最大间隙约 0.109 s。这些是软件观测结果，不能外推为实船停车距离或执行器保证。

暂停及时间回退由**新 ControlNode 的真实 ROS `/clock` 测试**验证：暂停进入 `SIMULATION_PAUSED` 并停推，恢复后续行；时钟回退后清目标和参考。这组是同一测试 executor 中的新节点测试，不表述为 Gazebo 全链路暂停/重置压力测试。

毫米波试验中，SIGSTOP 暂停原感知输出，Gazebo raw scan 和 Costmap 持续更新，控制状态进入 `WAITING_SENSOR`，三路推力为零；SIGCONT 恢复后继续原任务。该临时断流与模块重新启动的处理不同，保持 Q8/Q12 要求。

## 复现命令

从原仓库的 WSL 路径运行，先 source ROS 和上文安装区。所有结果目录必须是新的目录，脚本拒绝覆盖旧记录。Gazebo 用例串行运行。

```bash
# 全源码测试，启用实际安装审计用例。
USV_DISTRIBUTED_INSTALL=/home/usv/distributed-navigation-build-20260913/install \
  bash scripts/validation/run_distributed_source_tests.bash \
  /home/usv/distributed-navigation-build-20260913/install \
  /absolute/new/source-tests

# 普通导航矩阵和任务/故障。
python3 scripts/validation/ros_acceptance.py \
  --workspace /home/usv/distributed-navigation-build-20260913 \
  --distributed --controllers pid lqr --repeats 3 --faults \
  --scenarios-file scripts/validation/scenarios_grid_interior.json \
  --domain-id 216 --costmap-topic /otter/costmap_raw \
  --output /absolute/new/gazebo-matrix

# 毫米波模式。
python3 scripts/validation/mmwave_smoke.py --distributed \
  --domain-id 218 --timeout 180 --output /absolute/new/mmwave
```

完整构建和各次增量构建的实际 argv 在 [build-records](build-records/) 中的 `logger_all.log` 首行；每包退出结果在对应 `events.log`。本轮没有更换或安装系统依赖。

## 独立记录的问题与边界

1. 既有控制导出测试在 pytest 6.2.5 下缺少夹具目录。基线及本次代码均可复现；本次没有修改该工具或原断言。全源码运行沿用基线 `PYTEST_ADDOPTS='-o testpaths=src/common/test'`，外层仍显式收集完整 `src scripts`；该设置只影响原测试中未指定路径的子 pytest。完整对照与修复建议见 [缺陷记录](preexisting-export-test/findings.md)。因此 914 项通过的结论带有此明确环境条件。
2. 最初的源码/安装哈希核对脚本误将 `sim_env` 当作 ament_python 目录，并少计算了 path_planner 的一层目录。[初次记录](install-audit/source-install.json) 保留；按独立安装审计确认的真实目录修正检查后，71 项全部一致。没有为使检查通过而改动安装文件。
3. `mmwave_smoke.py` 传入的临时 world 路径改为不带 `.world` 后缀，与原 launch 自动追加后缀的语义一致；这是验收脚本适配。
4. 原统计字段继续保留。状态合并中同名地图指标由航控值覆盖，字段应理解为该模块的观测，不能相加当作完整系统 CPU 或延迟统计；统计接口重构另行确认。
5. 首版范围为同机 ROS 2 仿真。真实传感器、实际执行器、跨机器部署与甲方 DDS 接口对接均未实施。航控进程本身退出期间没有推进器发布者；设备侧失联处理需要后续硬件联调确定。
6. 最终修复后的 `process-smoke-003` 和 `faults-003` 断言均通过，但外层 bash 的退出码变量为空，使包装命令返回 1。原日志和 [说明](faults-003/wrapper-observation.json) 保留，没有把该返回值改写成成功。004 两套改用 `exec python3` 直接取得 Python 退出状态，作为最终复验证据。

普通导航改造未新增定点值守、绕圈、阵位保持、COLREGs 规则或算法改进，也没有引入未确认的碰撞执行许可协议。已存在的相关文件继续保留。
