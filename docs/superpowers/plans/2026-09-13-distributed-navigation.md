# 三模块普通导航拆分实施计划

> **For agentic workers:** Use superpowers:subagent-driven-development to implement the assigned files, then review behavior and run the full integration validation. Do not revert other workers' edits.

**Goal:** 基于当前未提交代码，在新分支形成同机 ROS 2 三模块导航；仿真导航和 YAML 加载保持原系统行为。

**Architecture:** 感知定位继续使用现有定位、mapping/mmwave、Costmap 节点，提供独立启动入口。新增 `usv_navigation` 包承载独立的决策规划与航控进程，`usv_navigation_interfaces` 提供跨进程轨迹、反馈和模块身份消息。原算法和项目 YAML 解析逻辑复用；`sim_env` 负责仿真、组合启动及旧入口兼容。

**Tech Stack:** ROS 2 Humble / rclpy / rosidl / Python / NumPy / Gazebo / pytest / WSL Ubuntu 22.04.

**Spec:** [已确认需求](../../distributed-navigation/requirements-clarification.md)。用户已另行授权进入实现，并补充 YAML 按原系统加载。

## Global Constraints

- 分支 `codex/distributed-navigation`，保留所有已有未提交和未跟踪文件。
- 只实现普通导航；不改值守与阵位保持算法；不接真实传感器或执行器，不做跨机器验收。
- 保留算法、目标替换/取消/完成、输入临时失效及恢复、仿真暂停/回退和热参数行为。
- 轨迹碰撞检查位于决策；当前实际船体占用检查位于航控控制周期；感知维护地图。
- 每个业务模块重启后需要新目标；旧轨迹不能恢复任务。输入短暂中断不等同于进程重启。
- 保留 goal/cancel_navigation/navigation_status/global_path/local_trajectory/推进器外部接口，以及 /<namespace>/usv_simulation 参数和导出接口。
- 继续加载四个现有项目 YAML，保留格式、默认值、参数别名、校验、启动覆盖顺序、传感器和命名空间覆盖。用户确认物理路径可以随拆分调整，不要求旧路径不变。

## 任务 1：基线与通信边界

**Files:** `src/core/usv_navigation_interfaces/{CMakeLists.txt,package.xml,msg/*.msg}`；`src/core/usv_navigation/{setup.py,setup.cfg,package.xml,resource/usv_navigation,usv_navigation/{protocol,configuration,state_rules,runtime,sensor_configuration}.py,test/test_protocol.py}`。

**接口：**

```text
TrajectoryPoint: float64 time,s,x,y,theta,kappa,velocity,acceleration
NavigationReference: Header header; string planner_session,control_session,perception_session;
  uint64 task_id,revision,sequence; bool active; Pose2D goal;
  string state,reason,parameters_json; float64 trajectory_stamp;
  string trajectory_frame; TrajectoryPoint[] points
ModuleHeartbeat: string module,session; uint64 sequence; bool ready; string reason
ControlReport: Header header; string control_session,planner_session;
  uint64 task_id,reference_sequence; string state,reason;
  bool revoke_reference,task_finished,clock_reset;
  float64 left,right,lateral; string metrics_json,counters_json,process_json
```

消息 `navigation/reference` 传递完整八列轨迹和源时戳，不用显示用 Path 替代轨迹。空轨迹且 active=true 表示等待新参考，不能当作新任务重置控制器。任务编号仅在目标替换、取消、完成、时钟回退或模块重启时递增；参考序号随轨迹或配置变化递增。模块重启身份用 UUID；消息通过目标航控身份约束，旧身份退休后不重新接受。

- [x] 创建分支；验证基线快照逐文件哈希与原源码一致。
- [x] 为完整轨迹保真、退休会话与乱序拒绝写失败测试，运行并保存结果。
- [x] 实现消息、转换与身份工具；移动现有 configuration/runtime/sensor_configuration，保留旧导入兼容。
- [x] 构建消息包并运行协议测试和现有配置/算法测试。

```python
assert peer.observe('first', 1, 10.0) == 'new'
assert peer.observe('second', 1, 11.0) == 'restart'
assert peer.observe('first', 2, 12.0) == 'ignored'
assert peer.session == 'second'
```

## 任务 2：决策与控制进程

**Files:** `usv_navigation/ros_support.py,decision.py,control.py,perception_monitor.py`；`src/sim_env/sim_env/{simulation,configuration,runtime,sensor_configuration}.py`；ROS 集成测试。

**接口：** `NavigationInputs(Node)` 提供原样的坐标/里程计/地图/雷达时效处理；`DecisionNode` 保留 usv_simulation 节点名，拥有目标、规划 worker 和对外状态；`ControlNode` 名 usv_control，拥有控制器与推进器发布者。分别运行独立进程；控制不调用规划器。

- [x] 用真实 ROS 话题测试取消、参考失效、过时/旧任务参考及模块重启；先观察缺失行为。
- [x] 提取共用 ROS 输入处理；决策以原 5 Hz 发起 worker，非阻塞接收规划结果；保留原规划校验。
- [x] 航控按 YAML 中的控制频率、steady timer 运行，先应用热参和输入，再检查当前船体，计算并发布推进器指令。
- [x] 完成控制反馈与状态汇总；目标完成立刻停推，决策收到反馈清任务。取消服务等待对应任务零输出反馈后成功返回。
- [x] 感知监测原业务输出发布者身份；变更时广播新的模块身份。此身份检测不替代原观测新鲜度检查。
- [x] 运行 ROS 集成测试，确认 PID 热参数/LOS 参考变化不会误清 PID 积分。

```python
# 新目标重置控制器；同任务的新参考及 LOS 热参仅替换参考。
assert controller.task_id == reference.task_id
assert controller.trajectory.stamp == reference.trajectory_stamp
# 重启航控获得新 session，旧 reference.control_session 无法匹配。
assert old_reference.control_session != restarted_controller.session
```

## 任务 3：启动与 YAML 兼容

**Files:** `src/sim_env/launch/{main,perception,decision,control,simulation_support}.launch.py`、`src/sim_env/sim_env/launch_support.py`、sim_env 包依赖、launch 测试。

**接口：** 旧 main 命令仍启动完整仿真；四个分入口可分别启动。所有入口共用旧四 YAML 解析流程、旧参数名与相同覆盖次序；规划和控制同时收到相同有效参数。

- [x] 测试使用自定义 YAML 与非默认控制器/LOS 参数启动后两端实际参数相同；空覆盖保留 YAML。
- [x] 抽取原 main 组合逻辑，提供独立感知、决策、航控与仿真支撑入口。
- [x] main 默认组合三个模块与仿真；保留 namespace、sensor、world、headless/gui/rviz 及配置路径参数。
- [x] 构建后通过实际 GetParameters 与 export_parameters 检查有效参数，并测试旧热参数接口。

## 任务 4：仿真和交付验证

**Files:** `scripts/validation/` 中新增拆分验证脚本、必要的既有 harness 适配；`docs/validation/distributed-navigation-001/`、用户启动说明。

- [x] 在基线独立 install 跑原仿真代表用例，保留结果。
- [x] 在新 install 跑完整源码/ROS 测试和独立进程集成测试。
- [x] PID/LQR 的直航、转弯、障碍，目标替换、取消、odom/map 断流、仿真暂停/回退、毫米波断流均验证；暂停/回退的证据层为新节点真实 ROS `/clock` 测试。
- [x] 分别重启感知、决策、航控，验证不会自动续行旧任务，重新下目标可以恢复导航。
- [x] 检查 ROS 图中业务独立进程和唯一推进器发布者、原话题/服务兼容、自定义 YAML 加载。
- [x] 审查各需求对应证据，记录真实运行命令、结果和限制；本次首版目标已完成。

## 实施记录

- 基线：`/home/usv/distributed-navigation-baseline-20260913`，890 文件复制后逐一 SHA256 一致。
- 原数值/规划测试：257 passed；基线完整构建18包通过、全源码846 passed，代表仿真4/4通过。未把旧测试记录当作新版本证明。
- Ruling：按用户明确要求在当前未提交工作区上新建分支，不另建丢失未提交内容的工作树。
- Ruling：通信包的会话与序号只落实重启和旧消息处理要求，不引入此前未选用的碰撞执行许可。
- 完整新构建：20 包通过；构建根 `/home/usv/distributed-navigation-build-20260913`。现有 Gazebo/CMake 警告与基线一致。
- 单进程真实 ROS 组合测试发现新目标先关闭后激活问题，已改为目标验证后一次发布；回归通过。
- 独立进程 smoke：12 项通过，三业务 PID 不同、推进器单一发布者、取消/断流/三模块重启及重新下目标恢复。
- 真实独立 launch 的自定义 YAML 探针通过：修改位置及模型、控制频率、算法选择、LOS 值后两端读回一致，热更新与导出通过。
- 安装审计：独立干净 shell 下 48 项通过；故意错误安装根被拒绝。
- 原基线完整测试846通过使用既有 `PYTEST_ADDOPTS="-o testpaths=src/common/test"` 导出夹具兼容环境，外层仍显式收集 `src scripts`。无该环境时，旧导出测试在基线及新代码均失败；缺陷单列，本次未修改旧导出实现或断言。
- 新全源码无夹具兼容设置：893 passed / 1 旧夹具失败；以相同基线环境重跑全 `src scripts`，最终加入取消并发测试及实际安装检查后为914 passed，无失败、无跳过。未排除外层测试。
- 固定安装区仿真矩阵：PID/LQR × 三场景 × 三次加3任务/故障共21/21通过；毫米波实际Gazebo 20/20断言通过。
- 收尾审查发现取消并发反馈匹配问题：新增3例先复现2失败，再最小修正已验证会话报告的匹配顺序；全集成8/8通过，缺失实际ACK仍超时失败。相关红绿日志已保存。
- 取消修复增量构建通过，71项最终源码/安装SHA256一致。最终process-smoke-004为12/12、faults-004为3/3，两者直接Python退出码均0；003断言通过但外层日志包装返回1，原始记录保留。运行进程已清理。
- 最终证据、命令、适用范围及既有缺陷见[验证报告](../../validation/distributed-navigation-001/README.md)。未提交或推送Git；保留当前工作树供用户审阅。
