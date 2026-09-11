# 控制与规划时序源码核查

**原 C++ 没有 `onPlan` 或独立 5 Hz 规划 timer。** 规划在 `onOdomCallback` 内同步执行，随后更新控制器缓存；10 Hz 控制回调不重新读取 odom。不能据源码称其固定复用 5 Hz 状态/轨迹。Python 则有独立 5 Hz 规划任务提交与 10 Hz 控制 timer；这是明确的采样和延迟处理差异，不能直接等同于本次跟踪误差差异的根因。

本次只读核查未运行测试、ROS 节点或 Gazebo，也未修改源码。C++ 来源为 `/home/usv/usv_migration/baseline` 的 `632734ba31478bdb833e15949287b53ae9334cf3`；核查的 simulation、node、controller、factory、PID、LQR 六份 C++ 文件与原 `cpp-baseline=ed4ed1803b1d520e19712df82f54ffac6ff313b3` Git blob 均相同。Python 为当前 Windows 源码，采样时 HEAD 为 `dbef058f754fe1f26d1337cbde4a874ddf384fb8`，实际文件以逐文件 SHA 为准，不把源码审计当作安装运行测量。

完整绝对路径、SHA256、代码行号和原文见 [sources.json](sources.json)，该索引 SHA256 为 `f25e6cb62edfacbfb7a0690284e75389a0b7919cbb24900bef30dc6be7a13dd9`。下列 C++ 相对路径以 baseline 根为准，Python 相对路径以 Windows 仓库根为准。

## 源码确定的行为

| 项目 | C++ | Python |
| --- | --- | --- |
| 控制定时器 | `src/sim_env/src/simulation.cpp:111–113` 使用 `create_wall_timer(1/control_frequency)`；`src/core/controller/controller/src/controller_factory.cpp:45` 声明完整键 `usv_simulation.controller.controller_frequncy`，默认 10.0 Hz | `src/sim_env/sim_env/simulation.py:82–85` 显式使用 steady clock 控制 timer；`configuration.py:24` 的规范键 `control_frequency` 默认 10.0 Hz |
| 规划触发 | `simulation.cpp:164–223` 在收到并执行 odom 回调后同步检查/规划/更新缓存，没有规划 timer | `simulation.py:85,221–230` 按节点时钟以默认 5 Hz 提交快照；`configuration.py:19` 声明该频率。运行 `use_sim_time=true` 时是 ROS 仿真时钟，不能等同固定 5 Hz 墙钟 |
| 状态更新 | `simulation.cpp:173–179` 从本次 odom 组装状态，`:215–223` 完成规划调用后执行 `setGlobalPlan`、`setLocalPlan`、`setState`；`controller.cpp:130–133` 只是复制状态 | `simulation.py:128–151` 每次 odom 回调尝试转换并保存状态，TF 暂不可用则保留 pending；控制入口 `:284` 再尝试消费 pending，`:299` 将当时 `self.state` 直接交给 controller |
| 控制回调 | `simulation.cpp:236–256` 使用缓存调用 `computeControlCommand`，没有 `setState`；在下一次 odom 更新缓存前，可能重复使用相同状态和局部轨迹 | `simulation.py:237–267,273–299` 先消费新状态、已完成规划结果并检查有效性，再 compute；新状态和轨迹可来自不同源时间 |
| 参考时间查询 | `otter_pid_controller.cpp:93`、`otter_lqr_controller.cpp:100` 都以固定 `lookahead_time_` 查询缓存局部轨迹，没有加源时间年龄 | `src/core/controller/controller/controller/controller.py:61–62` 查询 `max(0,state.stamp−trajectory.stamp)+lookahead_time` |
| PID 时间步 | `controller.cpp:38–42` 固定 `control_dt=1/control_frequency`，PID `:130–131,145–146` 用于积分/差分 | `simulation.py:279–280,299` 传入本次节点时钟与上次控制时钟之差；`pid_controller.py:53–58` 等使用该 dt。steady timer 的触发时钟与计算 dt 的节点时钟不同 |

原项目 YAML 的 `control_frequncy` 与 C++ factory 声明的 `controller_frequncy` 拼写不同；本核查所述 10 Hz 具有 factory 默认值依据，不将任意 YAML 同名近似字段都当作有效覆盖。Python 配置入口将历史拼写归一到规范键。

C++ 的 `onGoalCallback` 只设置 global/local plan（`simulation.cpp:157–159`），没有调用 `setState`；本文件中唯一状态写入位于 odom 规划回调 `:223`。该回调可能因 pending 或空路径/空轨迹提前返回（`:167–197`），所以收到 odom 不代表一定写入控制器状态。局部规划返回 false 时则按源码日志保留旧计划，仍继续后面的状态赋值。

## Python 的 age 补偿

`PlanningRequest` 保存提交时的状态；`runtime.py:134–174` 使用一个 spawned 规划进程、一个正在执行任务和一个可替换 pending 快照。任务完成后由控制回调调用 `poll` 消费。**5 Hz 是请求提交周期，不能从源码断言结果也恰好每 0.2 s 更新。**

LOS 在 `los_trajectory_planner.py:148` 将请求状态的源戳保存为 `Trajectory.stamp`；结果消费时没有把它改成完成时刻（`simulation.py:266`）。设规划状态源时间为 \(t_p\)、当前已消费状态源时间为 \(t_s\)、预瞄时长为 \(L\)，则查询的轨迹相对时间为：

\[
q=\max(0,t_s-t_p)+L.
\]

在 \(t_s\ge t_p\) 时，这把查询推进到与当前测量状态对应的预瞄时刻，而不是对旧轨迹反复查询起点后固定 L 秒。它补偿的是**轨迹基准与状态源戳之差**，包含源时间意义下的队列/规划滞后；不是独立测量的 worker CPU 耗时，也不是按当前墙钟预测艇体状态。若没有新的 odom，状态戳不变，age 不会因 steady timer 再次触发而自动增长；过期数据另由 `readiness` 拒绝。`trajectory.py:69–95` 在已有轨迹内插值，查询超过末端时取末点。

因此，保留 PID/LQR/LOS 方法并不意味着保留了原 C++ 的全部采样系统：状态与轨迹的更新耦合、规划结果延迟、相对参考时刻、PID 的 dt 处理都存在明确变化。这里只说明差异，不量化其误差贡献。

## 不能从源码断言的调度关系

两版主程序均使用默认单线程 node executor：C++ `simulation_node.cpp:66` 的 `rclcpp::spin`，本机 `/opt/ros/humble/include/rclcpp/rclcpp/executors.hpp:41–49` 明确说明默认单线程；Python `simulation.py:358` 调用 `rclpy.spin`，本机 rclpy `__init__.py:101–106,223–229` 使用全局 `SingleThreadedExecutor`。C++ 同步 odom 规划可能推迟控制回调；Python 规划进程能独立执行，但节点侧回调及控制计算仍会占用 executor。

这些事实不保证 timer 与 odom 固定交替，也不保证控制实际达到 10 Hz。源码不能确定每份 C++ 状态实际被复用了几次、odom 实际处理频率、worker 完成到消费的延迟、DDS 中尚未处理的消息数量或各回调抖动。Python 的“最新状态”仅指节点已成功消费的最新状态，不能写成物理世界最新状态。

故问题“原 C++ 10 Hz 是否实际复用 5 Hz 状态/轨迹”的严格回答是：**会复用缓存，但源码没有固定 5 Hz 更新依据；只有实际 odom/规划执行恰为 5 Hz 时才可能近似出现这一现象。** 确定实际关系需逐回调源戳、开始/完成时间和命令记录，本轮没有追加测量或扩大根因结论。
