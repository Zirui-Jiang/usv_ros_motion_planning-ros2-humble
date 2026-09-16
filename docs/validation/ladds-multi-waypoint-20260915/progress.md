# SDD ledger — plan: docs/superpowers/plans/2026-09-15-ladds-multi-waypoint.md

## 预检与裁定

| 任务/接口 | 检查结论 |
|---|---|
| Task1→Task3 NavigateRoute/RouteWaypoint | 固定header、waypoints、waypoint_radius；由Task1写msg，Task3消费 |
| Task2→Task3 PlanningRequest | 尾部可选waypoints/radius，保持旧位置参数兼容；各点(x,y,theta,speed) |
| Task1→Task3 pass_through | ROS bool默认false，协议解码仅显式true取消中间停车判定 |
| Task1 | 完整解析后提交；重试与新下发分离 |
| Task2 | 预测通过不改变真实index；原单点入口回归 |
| Task3 | 切点不换task_id；全部前点完成才允许最终停止 |
| Task4 | 编译消息后ROS验证，独立profile与仿真域，清理自有进程 |

Ruling: 在用户当前非main工作分支直接修改并按文件分工，不另建缺少未提交前置模块的worktree；不提交无关文件。

Ruling: 用户已批准设计与实施，无需再询问执行方式；按清晰文件边界并行开发，由parent整合并复核。

## 进度

- 设计已批准，计划已记录。
- Task1: implemented；新接口/整包解析/整条NavigateRoute；167包测试+2隔离ROS夹具通过。独立桥层审查40测试通过，未发现必须修复问题。
- Task2: implemented；真实10s和20s时域数值闭环均通过直线、90°、2/4/2kn和掉头。独立审查发现后续地图外/障碍失败丢弃可行前缀，已TDD修复并回归。
- Task3: complete；RouteProgress基于新鲜实际位置，含采样段顺序/跳变防护；revision切点保持task_id和旧安全参考；pass_through及终点完成资格门控。独立审查通过；补充无终点轨迹值编解码往返测试并修复。
- Task4: complete；10包新overlay与交付构建脚本成功。最终442passed/1平台skip；独立域88/Gazebo11359转弯/短直线、2/4/2节两case通过，中间不断任务、最后停稳且0052各一次。清理自有进程。启动脚本、可复制安全水域三点例子、指南和实测图完成。

最终裁定：首轮4项启动断言失败源于测试硬编码PID，用户源配置当前为ADRC；仅修测试读取共享配置与覆盖优先级，生产控制器/YAML保持原样。Gazebo使用ADRC，数值闭环PID，两者证据分别记录。Windows工具端本轮多点DDS发收未重验，报告明确区分ROS侧注入验证。
