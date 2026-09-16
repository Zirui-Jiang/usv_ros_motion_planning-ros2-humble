# Module Package Organization Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development. Steps use checkbox syntax for tracking.

**Goal:** 按感知、决策、航控及共享/任务目录重组源码和启动归属，保持当前普通导航行为。

**Architecture:** 算法库与各模块的节点包并列。公共 usv_navigation 保存共享解析、协议与单份配置；sim_env 作为上层组合，不被业务包反向依赖。

**Tech Stack:** ROS 2 Humble、Python 3.10、C++17、ament/colcon、WSL Ubuntu 22.04、pytest、Gazebo Classic。

**Spec:** docs/distributed-navigation/module-package-organization.md

## Global Constraints

- 用户授权在当前 codex/distributed-navigation 和已有未提交状态上整理；本轮不提交、不推送、不换分支。
- 新任务不接入，保持现在的导航任务；算法包名、数值算法、YAML 格式、ROS 名称和行为保持。
- 快照：/home/usv/module-layout-20260914/source-before，647 文件复制后 SHA-256 已核验。
- 本轮独立构建根：/home/usv/module-layout-20260914；旧安装区不覆盖、不作为最终验证环境。
- 共享配置四份由 usv_navigation/config 持有；mapping/costmap 由 usv_perception/config 持有。
- 新公共 API：launch_support.navigation_arguments()、navigation_settings(context)、navigation_node(settings, package, executable, name, extra=None)。settings 提供 namespace、use_sim_time、navigation、logs。
- 三个包内启动入口：usv_perception/perception.launch.py、usv_decision/decision.launch.py、usv_control/control.launch.py。
- 公共 node_runner.run_node(factory, args=None) 原样保留节点创建/退出语义；heartbeat.py 持有 HEARTBEAT_PERIOD/PEER_TIMEOUT/PeerSession，protocol 保留兼容导入及轨迹转换。

## Task 1: 基线、分组移动和依赖契约（root）

**Files:** src/core 下既有十个业务/公共包；scripts/validation/test_module_package_layout.py。

- [x] 执行新业务包独立依赖测试，确认旧布局因缺失业务包失败。
- [x] 在 PowerShell 中校验全部源/目标绝对路径均在仓库内，目标不存在，然后按 spec 逐包 Move-Item，保留已有未提交文件。
- [x] 将 decision.py/runtime.py、control.py、mapping_node.py/perception_monitor.py/costmap C++ 及对应 ROS 测试移动到新归属；算法核只移动目录。
- [x] 移动四份共享 YAML 与两份感知 YAML，文件内容不变。

## Task 2: 导航节点及公共支持（navigation worker）

**Files:** src/core/decision_module/usv_decision；src/core/control_module/usv_control；src/core/shared/usv_navigation 的 Python、setup.py、package.xml（launch_support.py 和 config 由 root）。

**Interfaces:** 消费上述公共 launch API；产生 decision_node/control_node executable；ROS 名称分别仍为 usv_simulation/usv_control。

- [x] 调整对应 ROS 测试导入到实际新包，保持原有断言。
- [x] 节点及 runtime 仅调整导入；公共 run_node 和心跳逻辑提取，函数/类主体保持。
- [x] setup.py 注册各自 executable 与 launch/config；公共 setup.py 移除业务 entrypoint，安装共享配置。
- [x] 独立 launch 示例实现使用 `navigation_settings(context)`，返回 `navigation_node(settings, 'usv_decision', 'decision_node', 'usv_simulation')`；control 对应自身包。
- [x] 执行移入的协议/决策/航控测试并自查；不构建、不启动 Gazebo。

## Task 3: 感知业务包（perception worker）

**Files:** src/core/perception_module 下两包（原算法文件主体保持）；不改 sim_env CMake。

**Interfaces:** executable 名保持 perception_mapping_node/costmap_support_node/perception_monitor/usv_tf_expander.py；新包为 usv_perception；节点名 mapping/costmap/usv_perception 保持。

- [x] 增加/迁移真实 launch 行为测试，验证只有感知的三个主要节点，无仿真定位或规划/控制节点。
- [x] ament_cmake + ament_cmake_python 安装 C++/Python、launch、config。mapping 的相对算法导入改为原 perception 库。
- [x] 感知 launch 仅声明 namespace、sim time、初始地图 offset 及两个自身配置路径；保持 scan/TF/frame/plugin 参数构造和校验。
- [x] monitor 仅调用公共 heartbeat/node_runner，避免因入口加载规划/控制。
- [x] 原 perception 去掉被迁移的 ROS executable，保留纯算法与原测试；不构建、不启动 Gazebo。

## Task 4: 入口、工具与兼容（root + tooling worker）

**Root files:** src/sim_env 全部；共享 launch_support.py；README.md 与 docs 当前说明。

**Tooling worker files:** scripts 全部、conftest.py、pytest.ini、mission_module 两包；算法库 test 中的路径定位调整。不改算法生产文件和其他 worker 的新包测试。

- [x] root 将 shared launch helper 从原 launch_support 中提取为只查询公共包；旧 main 组合四个模块入口，旧三份模块 launch 兼容转发。
- [x] simulation_support 启动 fake_localization；sim_env 不再构建 costmap；sim_env Python 兼容导入指向新决策包。
- [x] 更新 source discovery、导出脚本及真实安装审计，检查新包来源与无旧入口/雷达包污染。
- [x] preservation 按 package.xml 发现旧新包位置，比较算法字节与迁移定义 AST；不得放宽数值行为比较。
- [x] mission 仅适配共享 YAML 与感知 executable 新来源，保持任务实现不变。
- [x] 运行各自相关测试；每个 worker 报告所有修改和实际退出码，root 复查。

## Task 5: 独立构建和运行验证（root）

- [x] `source /opt/ros/humble/setup.bash` 后在全新 build/install/log 根完整构建，预期 20 包。
- [x] 新安装区审计、算法保留、全 src/scripts 源码与 ROS 测试；保留既有 exporter 测试夹具环境说明。
- [x] 各新包独立入口的分进程 smoke（domain 217）、YAML probe（218）。
- [x] 完整主入口 PID/LQR × straight/turn/single_obstacle × 1，另目标替换、取消、odom 断流（domain 216）。
- [x] 新鲜 reviewer 只读比较本轮快照/源码，重点取消 ACK、时效、依赖边界、任务未接入与单份参数。修复有依据的问题后仅重跑受影响检查。
- [x] 记录当前目录、启动命令和真实证据；保持未提交状态，将剩余限制如实告知用户。
