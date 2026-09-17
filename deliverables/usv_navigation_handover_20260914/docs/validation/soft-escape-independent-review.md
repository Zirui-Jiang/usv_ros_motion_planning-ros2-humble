# Soft inflation escape 独立边界审查

日期：2026-09-11。审查对象为 planner 提供 ready 信号后冻结的工作区源码。审查者未修改 path_planner、common 或生产配置，也未运行 ROS/Gazebo。结论：本次限定范围内没有发现阻塞缺陷，**619 项独立数值检查全部通过**；真实闭环效果仍需后续候选验证。

## 固定输入

| 源文件 | SHA256 |
| --- | --- |
| `src/core/path_planner/path_planner/path_planner/graph_planner/astar_planner.py` | `e59cf8174a18488323057463ea31c06c8e993a2818319931dfa644c29f1bc659` |
| `src/core/path_planner/path_planner/path_planner/path_planner_core.py` | `ba28ca32373067906d7cd0163be5708bc7f85336b3a268ebb149febcf4b63980` |
| 未修改的 `src/core/common/common/math_helper.py` | `9d68927cb51ee686a03cbd5d4f7f7e3bf9ea50755ed537b8ea65ea38af223cc0` |

独立脚本：`/home/usv/usv_migration/results/soft-escape-independent-review-001.py`，SHA256 `6f876ff200b3c55bb3b49d285bbf35de54bde5159e9230ee344be81fb2b604d9`。结果：`/home/usv/usv_migration/results/soft-escape-independent-review-001.json`。结果保留每个检查名称、预期值、实际值、源码 SHA256 与完整失败列表；该次失败列表为空。

## 已检查边界

119 项定向断言覆盖 A*、Dijkstra、GBFS 与重采样开关组合，阈值采用本次配置 `254 × 0.25 = 63.5`：

- `[75,70,64,63,0]` 可以从 soft 起点严格下降退出；`[75,75,0]` 的高代价平台及 `[75,76,0]` 的上升被拒绝。
- `[75,60,70,0]` 进入低代价区后再进入高区被拒绝；终点仍在高区的 `[75,70,64]` 被拒绝。不同采样点位于同一个栅格时，不被误判为跨格平台。
- 254 不能作为起点或中间通行格；255 在默认策略下被阻挡。显式允许 unknown 时沿用现有 `unknown_is_obstacle=False` 接口，不增加 `allow_unknown` 别名，也不把 raw255 当作新的逃逸来源：unknown→高区仍被拒绝。
- outline 边界与地图外坐标被拒绝；outline 内部正常规划仍成功。
- 构造了一条在硬格内短暂穿行、旧半格间距采样可能漏检的连续段，新的逐格验证能够拒绝它。相同位置的 unknown 默认拒绝，显式允许时按既有策略通行。
- 在 Core 的实际重采样接口注入一条有效 L 形 grid path，以较大重采样间距产生切入 254 的候选弦段：Core 拒绝候选并保留原有效路径。
- common.path_is_clear 仍严格拒绝高代价起点；escape 语义只位于规划包，没有扩散到公共硬碰撞检查。

另以 seed `730219` 执行 500 项随机连续路径比对。独立参考方法把每段按全部网格线交点分段，读取各开放区间中点对应的栅格，再按栅格序列判断严格下降与硬障碍；它不复用被审阅函数的 DDA 递推。包含非零原点、0.25 m 分辨率、多个路径段以及默认/显式允许 unknown 两种策略。500 项结果均一致。

## 一致性与限制

A* 的高代价转移条件要求当前格也处于高区，并且下一高格代价严格更低；进入低区后无法重新取得逃逸资格。Core 对原始结果与重采样候选调用同一个专用验证函数，使用相同 factor、unknown、outline 和 corner-cutting 配置。精确角点继续遵循原有显式 corner-cutting 策略，没有借此次修改改变该默认行为。

这些是源码与合成数值边界检查，不是 619 次仿真实验，也不证明实艇/全艇扫掠安全。轨迹的全艇碰撞验证仍由既有独立检查负责；不能用本报告消除之前的负扫掠间隙或 NO_PATH 原始记录。脚本保留运行时固定源码哈希，重新执行时应选取新的结果输出路径，避免覆盖本次凭据。

M5 删除、干净构建和最终 OS 采样仍等待主任务的明确执行信号；本次审查没有触发这些动作。
