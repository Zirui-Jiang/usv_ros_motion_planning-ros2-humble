# Hybrid A* 路径闪烁、USV 静止：实时诊断与修复

日期：2026-09-16。目标：正在运行的 ROS 域 42、LaDDS 单航段仿真、Hybrid A* + ADRC。

## 已确认的原因

1. 当前决策与控制节点均已运行，问题发生在规划执行阶段。
2. 原 Hybrid 每次从当前船位重搜，搜索展开量较大，反复耗尽 `navigation.planning_timeout=1.0` 秒。
3. 决策节点 `_consume_plan` 会发布每次结果的 `route`。失败结果为空，偶尔成功结果非空，导致 RViz 路径出现后又被清空。
4. 偶尔成功的轨迹发布时已经比其源船位时间旧 1.2–1.3 秒，超过 `navigation.trajectory_timeout=1.0` 秒；控制层进入 `WAITING_REFERENCE` 并输出零推力。

这些结论来自只读订阅及同一地图快照的离线复现，不是仅根据源码推测。

## 实时证据

`capture-001/summary.json`、`events.json`、`snapshot.json`、`map.npz`：

- 约 15 秒收到 13 次全局路径：11 次 0 点，2 次 408 点。
- 导航参考：139 条 `TIMEOUT: collision checking time budget exceeded`，24 条 `NAVIGATING`（包含重传）。
- 控制报告：139 条 TIMEOUT、22 条 WAITING_REFERENCE、2 条回调阶段 NAVIGATING。
- 所有 163 条控制报告的左右桨/侧推命令均为零；NAVIGATING 回调报告也没有实际非零推力。
- 观测速度约 0.00034–0.00955 m/s，属于基本静止、轻微漂移。
- 两次成功参考初次发布时的 `published_stamp - trajectory_stamp` 分别约 1.3、1.2 秒。
- 路径、里程计、左桨命令各只有一个发布节点，未发现这些话题上的重复发布者。

`stale_references` 计数主要还包含相同序号重传被过滤的情况，不能单独作为轨迹过期证据；本次按实际源时间差确认过期。

## 离线复现与性能证据

冻结输入：800×800 raw costmap，分辨率 0.5 m，原点 (-179,-218) m；
起点 `(20.4631,-17.8934,2.2071)`，目标 `(-10.3535,21.0811,2.1980)`。

| 版本 | 结果 | 展开数 | 耗时 |
| --- | --- | --- | --- |
| 原源码，WSL，1 s 预算 | TIMEOUT，路径/轨迹为空 | 本次2263/2354 | 约1.001 s |
| 原源码，延长到4 s的性能分析 | 找到408点路径 | 2628 | 性能分析仪下2.75 s；Windows完整链约1.61 s |
| 修复源码，WSL，原1 s预算 | SUCCESS，417点路径、21点参考 | 196 | 全局约0.086 s，另一次完整链约0.078 s |

耗时是本次单快照测量，受进程调度、仿真负载和分析仪影响，不是所有场景的实时性保证。
原源码的分析记录中约91%的规划时间位于运动段碰撞检查调用链。

## 最小修复

- `hybrid_astar_planner.py` 增加 `heuristic_weight`，默认 `1.2`，采用 `f=g+w*h` 排序。
- `w=1` 可以恢复原排序；`w>1` 更优先向目标推进，减少近似等代价分支展开，可能得到稍长路线。
- 共享参数声明/校验及独立 Hybrid YAML 同步增加 `path_planner.heuristic_weight`。
- 不改变碰撞阈值、船体尺寸、最小转弯半径、规划预算和轨迹有效期；不修改控制增益。
- 本次路径长度由约50.769 m变为51.861 m，增加约1.092 m。
- 增加实际长航段回归：以500次展开作为工作量上限，旧代码失败，新代码通过；同时检查精确端点、前进曲率及船体可行性。

回归快照在 `src/core/decision_module/path_planner/test/fixtures/hybrid_long_leg/`，
地图 SHA256：`15820b1a175b6b6e05da4ceb9d46219c0323f452083a96f3778948c880ed2cf2`。

## 检查结果

- Windows：431 passed、9 skipped（ROS依赖）、1 deselected。
- WSL / ROS 2 Humble：277 passed、1 deselected，使用隔离 ROS 域 163。
- 被排除的是旧任务应用要求共用 YAML 仍选择 astar 的测试；用户当前已主动选择 hybrid_astar，本次保持该选择。
- 安装区 `build_python/path_planner/...` 与 `build_python/usv_navigation/...` 都解析到原 D 盘工作区源码。

WSL 回归命令（先加载 Humble 和当前安装区）：

```bash
PYTHONDONTWRITEBYTECODE=1 ROS_DOMAIN_ID=163 ROS_LOCALHOST_ONLY=1 python3 -m pytest \
  src/core/decision_module/path_planner/test \
  src/core/decision_module/trajectory_planner/test \
  src/core/decision_module/usv_decision/test/test_hybrid_runtime.py \
  src/core/decision_module/usv_decision/test/test_hybrid_ros.py \
  src/sim_env/test/test_configuration.py src/sim_env/test/test_hybrid_configuration.py \
  -q -p no:cacheprovider -k 'not common_planner_yaml'
```

## 复现命令

```text
python scripts/validation/replay_hybrid_snapshot.py docs/validation/hybrid-live-20260916/capture-001 --timeout 1 --heuristic-weight 1 --output validation-results/hybrid-original-order.json
python scripts/validation/replay_hybrid_snapshot.py docs/validation/hybrid-live-20260916/capture-001 --timeout 1 --output validation-results/hybrid-fixed-order.json
```

只读实时采样脚本：`scripts/validation/hybrid_live_probe.py`，没有目标或推进命令发布逻辑。

## 重启后的实际航段验证

用户在后续观测前已重启仿真，决策/控制 PID 为 40387/40389，参数快照确认
`path_planner.heuristic_weight=1.2`。本轮没有重启进程或发送新的导航目标。

| 记录 | 观测结果 |
| --- | --- |
| `capture-002`，约12秒 | 59条全局路径全部非空；179条控制报告全部NAVIGATING且非零桨；速度0.895–0.962 m/s；位移10.958 m；新参考首次发布时间年龄0–0.2 s |
| `capture-003`，约40秒 | 128条非空路径、8条空路径；中段能持续航行，但接近目标时再次出现约1 s搜索超时及WAITING_REFERENCE；总体位移19.230 m |
| `capture-004`，约8秒 | 决策/控制持续GOAL_REACHED，79条控制报告全部零推力；停靠于目标容差内 |

最终目标为 `(-0.315668,115.158844)` m，最终观测船位 `(0.403643,114.823930)` m，
位置误差 **0.79346 m**，满足当前1.5 m目标容差；观测窗口速度低于0.009 m/s。
详见 `live-arrival.json`。这证明原来的持续静止已解除，但不能将中段通过误写为整个航段没有闪烁。

末端数据表明，在严格前进、最小半径4 m、精确终点艏向的模型下，短Dubins连接存在
可行性分界：2610.0秒的船位附近最短连接由约6.564 m变为约31.039 m。
20 m解析连接上限将长候选排除，触发更多离散搜索；这是路径形状跳变与CPU开销叠加，
并非通过放大时间有效期就能解决。后续性能修复保留原半径、末端艏向和20 m上限。

## 全局路径缓存与裁剪（按用户后续要求）

后续改动将全局搜索与局部轨迹更新分开：

- 新目标首次规划；正常周期复检并沿原直线/圆弧投影裁剪缓存，不把实际横向偏差或艏向硬接成折线。
- 新计划仅对切向一致、完整直线通过中心/船体检查的节点做快捷连接。圆弧和必经点保留。
- 路段被障碍阻断、目标变化、偏离参考超过2 m或朝向差超过60°才重新搜索；进度匹配仅搜索剩余路径前10 m。
- 每个局部周期仍用最新state.stamp生成参考并检查预测船体，缓存不延长旧参考有效期。
- 有序航点按各段边界复用，不跨必经点捷径化；局部规划失败可以保留内部全局候选供下次复检。
- 全局path仅实际替换或空显示恢复时发布，反复失败只清空一次；local/reference继续更新。
- 全局path可靠TransientLocal、深度1；源码RViz与只读probe使用匹配持久性，支持晚订阅获取当前航路。

### 固定地图、真实船位序列回放

`cached-near-goal-replay-wsl.json` 使用capture-003记录的79个船位周期：

| 指标 | 结果 |
| --- | --- |
| 全局实际搜索次数 | **1** |
| 缓存复检/裁剪并生成新局部参考 | **78** |
| 规划结果 | **79次SUCCESS** |
| WSL单周期P50 / P95 / 最大耗时 | **6.72 / 9.82 / 82.90 ms** |

该回放固定使用末张地图；当前fake-localization中map/odom恒等已核实。原采集仅保存
速度大小，回放把它置于纵向用于参考生成，不能作为真实水动力闭环数据。没有据此宣称
新缓存策略已经在重启后的Gazebo中完成航行。

### 回归与安装核验

- Windows相关源码回归：**535 passed、10 skipped（ROS）、1 deselected**。
- WSL数值/接入回归：**315 passed、1 deselected**。
- 真实ROS发布/决策/多航点/分布式回归：**34 passed**（隔离域165）。旧分布式报告订阅测试
  在一次中间运行出现观察竞态，最终整组通过，没有修改旧测试以掩盖失败。
- 缓存模块独立40项测试，加上原碰撞65项测试；包括长两点直线持续裁剪、闭环、细障碍、
  未知策略、边界、曲率、直连切线条件和720条固定种子碰撞等价性对照。
- 从当前 `/tmp/usv_ladds_single_leg_20260915/install/setup.bash` 导入新缓存模块和runtime，
  两者路径均解析到原D盘工作区，因此Python功能不需重新colcon构建。

## 首次裁剪显示修复（1 Hz方案，已由下节替代）

此前事件发布策略有显示缺口：`prepare_cached_path` 已更新内部剩余路径，但
`_consume_plan` 仅在 `searched=True` 或空显示恢复时发送global_path，RViz因此一直
显示完整旧路径。已用真实ROS发布测试复现1.1秒后内部路径起点前移而显示起点不变。

当时新增 `navigation.global_path_publish_frequency=1.0`（Hz），仅限制缓存裁剪后的
显示刷新。局部参考/复检仍为5 Hz，全局搜索继续事件触发。新搜索、取消、失效清空、
空显示恢复立即发送；相同路径不重发。A*和Hybrid共用显示策略。

- 新旧行为对照：ROS发布测试先3失败/11通过，修复后14项全部通过。
- 本轮Windows缓存、裁剪、运行层、配置相关回归：110通过。
- WSL实际ROS发布+缓存运行层+配置检查：56通过（隔离域163）。
- 另一次发布/决策/多航点/节点回归：27通过（隔离域165），与上述范围有重叠。
- 固定历史地图的79周期回放仍只有1次全局搜索，79次全部成功。
- 实时采样capture-005没有收到消息；采样前识别的决策PID43966在期间退出，随后
  进程清单不再有该次仿真的Gazebo/决策/控制节点。本轮没有据此宣称实时内部裁剪状态，
  也没有重启或下发目标。显示修复待下一次启动加载。

修改位置：`usv_decision/decision.py` 的 `_consume_plan` 和 `_publish_global_path`、
共享参数DEFAULTS、独立Hybrid配置示例、对应测试与使用说明。没有更改Hybrid搜索、
缓存几何、碰撞约束或轨迹时间戳。

## 裁剪及时性与航点阶段同步（最终策略）

用户进一步反馈“比之前A*迟，都到了还没剪掉”。核对显示规则和多航点阶段后确认：

1. 上节的1 Hz显示限制会让已完成的缓存裁剪多等约1秒。已删除额外显示计时和参数，
   每次已验收的局部更新立即发布发生变化的剩余路径。相同路径去重，原地更新消息，不先清空再重画。
2. 多航点按进入1.5 m到达盘确认上一点完成；几何投影可能仍处于上一点入航段。
   在90°转弯的离线构造状态中，船已在到达盘内侧向转弯，旧点位于船后约0.616 m，
   缓存却让局部预测继续回贴旧入航段。该例证明阶段语义缺口，不能据此断言当前实船正经历相同姿态。
   现 `_update_route_progress` 在确认进度变化、版本递增时清空旧阶段全局缓存一次，
   新阶段从当前实测位姿建立航路；已有验收局部参考保留作过渡，正常后续周期恢复复用。

原A*的最近离散节点裁剪与Hybrid的原圆弧投影仍有几何差别，但不再额外限制显示频率。
当前对照采样 `capture-006-astar` 是LaDDS五航点A*任务，8秒有40次path消息、搜索计数
约每秒增加5；该旧多航点分支每周期分段规划，因此不能把它的快速显示误认为只有裁剪而无搜索。
本次没有为了显示速度恢复Hybrid高频全局重搜，也没有改半径、到达盘、碰撞或时间新鲜度门限。

### 验证

- 显示测试先3失败/11通过，删除限频后14项全过；覆盖A*和Hybrid连续0.2/0.4/0.6秒结果立即显示、搜索计数保持1、重复去重和晚订阅最新裁剪路径。
- 阶段缓存测试先4失败/2通过，加入一次性清理后6项通过；未到点不清、进入盘/侧向切弯/过中心只清一次、单目标不变。
- 实际纯runtime验证新阶段首次搜索成功，后续3次0.2秒更新不搜索（expanded=0）。
- WSL联合回归：66 passed（隔离域163，显示/阶段/多航点/复用/进度/节点）。
- Windows裁剪与配置等回归104通过；另一次阶段相关ROS回归41通过，范围有交集。
- 本轮正在运行的是用户切回的A*，没有自动重启或切换其算法。最终策略待重启Hybrid仿真验证。

显示字段 `navigation.global_path_publish_frequency` 已移除；若使用手写旧配置，应删去该字段。
仓库提供的Hybrid示例已同步移除。现仅 `navigation.planning_frequency` 控制局部更新/缓存复检周期。

## 证据边界与生效方式

- 本轮没有重启或操纵用户正在运行的仿真；已运行的 Python 进程不会自动重载源码。
- 决策和控制共享同一参数快照，需要一起重启；缓存参数与本次移除显示参数的变化不能只由其中一个节点加载。
- 权重修复已通过离线同图回放、回归和上述实际航段到达验证；末端仍存在短时超时，见上节。
- `last_global_path.npy` 与最终地图不是同一源时刻。它在最终地图上的矩形船体复检通过、中心软代价复检失败；不能据此断言原规划漏检，也不能据此无条件复用旧路径。
- 当前源码已经采用严格复检的路径复用；此前每次重搜的行为仅用于上述历史对照。
  新缓存策略尚待用户重启当前仿真加载；不能用刷新旧轨迹时间戳掩盖延迟。
