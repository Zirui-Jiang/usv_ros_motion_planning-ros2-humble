# 文档索引

本页列出仓库里的全部说明文档，并标明每份是**现行**（描述当前代码）还是**历史**（记录某个阶段，不再更新）。第一次接手项目，先读根目录的 [README.md](../README.md)，再按需要从下面找。

术语统一见 [CONTEXT.md](../CONTEXT.md)。

## 上手与操作指南（现行）

### 普通导航

| 文档 | 内容 |
| --- | --- |
| [项目 README](../README.md) | 环境、构建、启动、下发目标、配置入口、测试 |
| [三模块普通导航](distributed-navigation/README.md) | 模块划分、分模块启动、YAML 加载顺序、话题与任务接口全表 |
| [自动验收入口](../scripts/validation/README.md) | Gazebo 场景验收、重复试验、指标与对比 |

### LaDDS 联调

| 文档 | 内容 |
| --- | --- |
| [0211 多航点快速联调](ladds-multi-waypoint-quickstart.md) | 构建、Windows 中继、仿真与适配器启动、报文、0052/0053 语义 |
| [0411 本船导航](ladds-0411-quickstart.md) | 仿真真值 → 0411 → 内部 odom 的链路 |
| [0141 目标运动联调](ladds-0141-quickstart.md) | 接收 0141 障碍、障碍栅格、双容器提示 |
| [LaDDS 通信桥](../src/core/communication_module/usv_ladds_bridge/README.md) | 桥的原理、0054 推力、0051 心跳、话题表、排错 |
| [0331 刹停与终点值守](../src/core/communication_module/usv_ladds_bridge/README.stop-guard.md) | 0331 主动刹停、0211 taskType=80 终点值守、0212 靠离泊 |
| [报文样例](examples/) | 0211、0212、0141 的 JSON 样例 |

### 控制器

| 文档 | 内容 |
| --- | --- |
| [新控制器接入说明](controller-development-guide.md) | 在当前系统中接入一个新控制算法的步骤 |
| [0914 PI＋ADRC 接入说明](0914-controller-integration.md) | ADRC 的接口、参数来源、调参与回退命令 |
| [速度环调参台](../scripts/control/speed_tuner/README.md) | 不启动 ROS 的本地 Kp/Ki 调参界面（[设计说明](../scripts/control/speed_tuner/DESIGN.md)） |
| [控制器独立开发工作区](../scripts/control/README.md) | 导出只含控制算法的数值开发快照 |

### 算法讲解与观察工具

| 文档 | 内容 |
| --- | --- |
| [Hybrid A* 讲解页](explainers/hybrid-astar-explained.html) | 浏览器打开的可视化讲解 |
| [LOS 轨迹讲解页](explainers/los-trajectory-explained.html) | 浏览器打开的可视化讲解 |
| [Hybrid A* 参数观察台](../scripts/hybrid_tuner/README.md) | 离线网页，调用仓库里的真实规划器 |
| [LOS 轨迹生成示例](examples/trajectory_generation/README.md) | 手工指定路径，生成一次参考轨迹 |

## 包内说明（现行）

| 文档 | 内容 |
| --- | --- |
| [path_planner](../src/core/decision_module/path_planner/README.python.md) | A* / Dijkstra / GBFS / Hybrid A* 的实现与开关 |
| [path_processor](../src/core/decision_module/path_processor/README.python.md) | 路径后处理算法 |
| [perception](../src/core/perception_module/perception/README.md) | LiDAR 建图算法 |
| [usv_perception](../src/core/perception_module/usv_perception/README.md) | 感知节点、launch 与配置 |
| [usv_decision/config](../src/core/decision_module/usv_decision/config/README.md)、[usv_control/config](../src/core/control_module/usv_control/config/README.md) | 指向共享算法配置的位置 |
| [ADRC 移植来源](../src/core/control_module/controller/controller/adrc_core/SOURCE.md) | `adrc_core` 各文件与交付源码的对应关系 |
| [sim_env/test](../src/sim_env/test/README.md) | 旧单节点参照测试与 costmap 退出回归 |

## 设计记录

| 文档 | 内容 |
| --- | --- |
| [ADR 0001 艇端三模块与地图所有权](adr/0001-onboard-navigation-modules.md) | 已采纳 |
| [ADR 0002 碰撞检查的部署位置](adr/0002-collision-check-placement.md) | 已采纳 |
| [ADR 0003 统一控制指令流](adr/0003-ordered-control-directives.md) | 历史提案，未采纳 |
| [规划层与航控接口](planning-control-interface.md) | 2026-09-12 接口设计稿，部分内容未落地 |
| [新控制器设计开发说明](controller-design-brief.md) | 2026-09-14 给控制器开发者的任务书 |
| [拆分需求澄清记录](distributed-navigation/requirements-clarification.md) | 三模块拆分的需求问答 |
| [按业务模块整理功能包的需求](distributed-navigation/module-package-organization.md) | 2026-09-14 目录整理的已确认需求 |
| [独立控制开发与航控接口](control-autopilot-interface.md) | **已被"规划层与航控接口"取代**，仅作记录；控制工作区导出工具仍会复制它 |

## 历史记录（不再更新）

### Python 迁移阶段

| 文档 | 内容 |
| --- | --- |
| [实施状态](python-migration-status.md)、[验收报告](python-migration-validation.md)、[迁移差异](python-migration-differences.md) | 固定在源码 `dfb1fb7` 的迁移记录 |
| [工程使用指南](python-navigation-guide.md) | 迁移期的使用说明，其中的包路径已过时 |
| [原 C++ 版技术文档](技术文档.md) | 2026-09-10 的 v1.0 技术文档，描述原作者 C++ 实现 |

### 任务模块（定点值守、阵位保持）

| 文档 | 内容 |
| --- | --- |
| [使用及验收指南](station-guard-guide.md) | `usv_mission` 的任务入口与验收方法 |
| [迁移记录](station-guard-migration.md) | 任务逻辑从 `sim_env` 迁入 `usv_mission` 的范围 |
| [实施记录](station-guard-implementation.md)、[验证报告](station-guard-validation.md)、[调试记录](station-guard-debugging.md) | 迁移前版本的实施、验证与调试 |
| [usv_mission 包说明](../src/core/mission_module/usv_mission/README.md) | 任务模块的构建与启动（[任务参数](../src/core/mission_module/usv_mission/config/mission/README.md)、[验证脚本](../src/core/mission_module/usv_mission/validation/README.md)） |

### 其他

| 文档 | 内容 |
| --- | --- |
| [虚拟毫米波雷达](mmwave-simulation.md) | 已移除功能的记录 |
| [2026-09-18 运行时话题图](runtime-topic-graph-20260918.md) | 当时运行实例的话题快照 |
| [0211 单段航渡联调](../src/core/communication_module/usv_ladds_bridge/README.single-leg.md) | 早期单段联调说明，其中"航线编号与执行约定"一节仍然有效 |
| [AI 执行计划](superpowers/) | 2026-09-13 至 09-16 各项任务的分步计划与设计稿 |

## 验证证据

[validation/](validation/README.md) 保存各轮验证的原始记录，每份绑定具体的源码版本，不随代码更新。入口是按阶段分组的[证据索引](validation/README.md)。

## 维护记录

| 文档 | 内容 |
| --- | --- |
| [2026-10-08 整理审计](maintenance/2026-10-08-reorg-audit.md) | 结构、命名、文档的审计结论与逐项执行状态 |
