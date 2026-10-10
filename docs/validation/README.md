# 验证证据索引

本目录保存各轮验证的原始记录。每份证据都绑定当时的源码版本和安装区，**不随代码更新，也不要修改正文和数据**。要证明当前代码通过，需要重新运行验证并另存新的记录。

说明：

- 下表的"标题"取自各目录自己的 README 第一行；没有 README 的数据文件只列文件名，内容未逐份核实。
- 目录名里的 `-001` 是轮次编号，`-20260915` 这类是日期，两种写法都是历史形成的，保持原样。
- 本页只是索引，按阶段分组，不代表证据之间的先后依赖。

## 环境、工具与界面

| 条目 | 标题 |
| --- | --- |
| [environment-changes.md](environment-changes.md) | Gazebo Classic dependency installation |
| [environment.json](environment.json) | Python、NumPy、内核等环境版本数据 |
| [removed-garden-packages.txt](removed-garden-packages.txt) | 被移除的 Gazebo Garden 软件包清单 |
| [gui/](gui/README.md) | WSLg / RViz 验证范围 |
| [otter-gui-fix-001/](otter-gui-fix-001/README.md) | Otter 模型与启动修复，2026-09-11 |
| [wsl-clock-fix-20260915/](wsl-clock-fix-20260915/README.md) | WSL 时钟冲突与 Gazebo 周期性停顿诊断记录 |
| [gazebo-exit-race.md](gazebo-exit-race.md) | Gazebo SIGINT 退出异常：证据与诊断边界 |
| [plot-acceptance-guide.md](plot-acceptance-guide.md) | 验收结果图表导出 |

## 原 C++ 基线与对照

| 条目 | 标题 |
| --- | --- |
| [baseline/](baseline/README.md) | 原 C++ 基线验证摘要 |
| [baseline-grid-interior-001/](baseline-grid-interior-001/README.md) | 栅格内部场景 C++ PID 九例基线 |
| [parity-1a82de4/](parity-1a82de4/report.md) | 原C++ / Python 数值对照（无 README，标题取自 report.md） |
| [original-final-comparison/](original-final-comparison/README.md) | 原协议正式矩阵：完整对照与图表 |
| [ocean-repeatability-audit/](ocean-repeatability-audit/README.md) | Ocean 场景随机性与波相位可复现性核查 |
| [control-timing-source-audit/](control-timing-source-audit/README.md) | 控制与规划时序源码核查 |

## Python 迁移阶段

### 运行前固定的协议

| 条目 | 标题 |
| --- | --- |
| [scenario-mean-release-protocol.md](scenario-mean-release-protocol.md) | 场景均值口径与清理后正式复验协议 |
| [grid-endpoint-source-time-protocol.md](grid-endpoint-source-time-protocol.md) | 栅格终点兼容与源时间跟踪度量：完整复验协议 |
| [grid-interior-protocol.md](grid-interior-protocol.md) | 栅格内部起点的追加配对协议 |
| [pid-course-damping-protocol.md](pid-course-damping-protocol.md) | PID 航向微分增益候选：运行前固定协议 |
| [pid-preview30-protocol.md](pid-preview30-protocol.md) | PID 3.0 s 预瞄候选：运行前固定协议 |
| [soft-escape-release-protocol.md](soft-escape-release-protocol.md) | 软膨胀区驶出修复后的独立复验协议 |

### 正式矩阵、对照与清理

| 条目 | 标题 |
| --- | --- |
| [python-final-matrix-001/](python-final-matrix-001/README.md) | 原场景正式 Python 18+3 矩阵归档 |
| [python-final-matrix-001-process-performance.md](python-final-matrix-001-process-performance.md) | python-final-matrix-001：进程 CPU/RSS 旁路补采 |
| [python-pid-final-001/](python-pid-final-001/README.md) | PID final-001：导航完成与验收门槛分别归档 |
| [pid-final-001-vs-baseline-matrix-002/](pid-final-001-vs-baseline-matrix-002/README.md) | PID final-001 与 baseline matrix-002 对照归档 |
| [python-grid-interior-001/](python-grid-interior-001/README.md) | 格内 Python 矩阵：失败后中断归档 |
| [python-grid-interior-001-process-performance.md](python-grid-interior-001-process-performance.md) | python-grid-interior-001：提前停止矩阵的 OS 旁路记录 |
| [python-release-grid-001/](python-release-grid-001/README.md) | e381832 格内正式矩阵原始验收归档 |
| [python-release-grid-001-process-performance.md](python-release-grid-001-process-performance.md) | python-release-grid-001：正式矩阵 OS 旁路性能记录 |
| [python-release-grid-001-process-performance-receipt.json](python-release-grid-001-process-performance-receipt.json) | 上一条的数据回执 |
| [python-release-grid-comparison/](python-release-grid-comparison/README.md) | 格内正式矩阵：严格对照与图表 |
| [python-grid-endpoint-final-matrix-001/](python-grid-endpoint-final-matrix-001/README.md) | 栅格末点兼容修复：固定正式 18+3 原始验收归档 |
| [python-grid-endpoint-final-matrix-001-process-performance.md](python-grid-endpoint-final-matrix-001-process-performance.md) | python-grid-endpoint-final-matrix-001：正式矩阵 OS 旁路性能记录 |
| [python-grid-endpoint-final-matrix-001-process-performance-receipt.json](python-grid-endpoint-final-matrix-001-process-performance-receipt.json) | 上一条的数据回执 |
| [python-grid-endpoint-final-comparison/](python-grid-endpoint-final-comparison/README.md) | 栅格终点版本正式对照与图表 |
| [m5-cleanup/](m5-cleanup/README.md) | M5 旧 C++ 算法源码清理 |
| [python-clean-final-001/](python-clean-final-001/README.md) | M5 后首次全新构建与安装验收 |
| [python-clean-release-matrix-001/](python-clean-release-matrix-001/README.md) | 全新普通安装：正式 18+3 原始验收归档 |
| [python-clean-release-matrix-001-process-performance.md](python-clean-release-matrix-001-process-performance.md) | Clean-release 正式矩阵进程资源旁路 |
| [python-clean-release-matrix-001-process-performance.json](python-clean-release-matrix-001-process-performance.json) | 上一条的数据 |
| [python-clean-release-comparison-001/](python-clean-release-comparison-001/README.md) | 干净安装版本正式验收：场景均值、逐次结果与图表 |
| [scenario-mean-acceptance/](scenario-mean-acceptance/README.md) | 同场景三次均值验收：独立补充报告 |

### 候选参数试验

| 条目 | 标题 |
| --- | --- |
| [python-preview24-straight-candidate-001/](python-preview24-straight-candidate-001/README.md) | 固定 PID 2.4 s 预瞄候选：straight |
| [python-preview24-turn-candidate-001/](python-preview24-turn-candidate-001/README.md) | 固定 PID 2.4 s 预瞄候选：turn |
| [python-preview24-obstacle-candidate-001/](python-preview24-obstacle-candidate-001/README.md) | 固定 PID 2.4 s 预瞄候选：obstacle |
| [python-preview25-straight-candidate-001/](python-preview25-straight-candidate-001/README.md) | 固定 PID 2.5 s 预瞄候选：straight |
| [python-preview25-turn-candidate-001/](python-preview25-turn-candidate-001/README.md) | 固定 PID 2.5 s 预瞄候选：turn |
| [python-preview25-obstacle-candidate-001/](python-preview25-obstacle-candidate-001/README.md) | 固定 PID 2.5 s 预瞄候选：obstacle |
| [python-pid-preview26-turn-candidate-001/](python-pid-preview26-turn-candidate-001/README.md) | PID 2.6 s 预瞄转弯单例候选 |
| [python-pid-preview30-turn-candidate-001/](python-pid-preview30-turn-candidate-001/README.md) | 固定 PID 3.0 s 预瞄：已完成转弯三次候选 |
| [python-pid-course-d010-turn-candidate-001/](python-pid-course-d010-turn-candidate-001/README.md) | 固定 PID 航向微分增益 0.10：转弯三次候选 |
| [python-lqr-q1-r20-candidate-001/](python-lqr-q1-r20-candidate-001/README.md) | LQR Q1/R20 独立配置候选归档 |
| [python-factor025-candidate-001/](python-factor025-candidate-001/README.md) | factor=0.25 三次候选：首例持续软起点阻断后暂停 |
| [python-single-step-candidate-001/](python-single-step-candidate-001/README.md) | 单步 LOS 候选 273a1fb 的真实结果与暂停原因 |
| [python-soft-escape-candidate-001/](python-soft-escape-candidate-001/README.md) | 软膨胀起点逃逸：三次固定配置闭环候选 |
| [python-grid-endpoint-straight-candidate-001/](python-grid-endpoint-straight-candidate-001/README.md) | 栅格末点兼容修复：PID 直航三次候选 |
| [python-grid-endpoint-turn-candidate-001/](python-grid-endpoint-turn-candidate-001/README.md) | 栅格末点兼容修复：PID 转弯一次候选 |
| [python-grid-endpoint-obstacle-candidate-001/](python-grid-endpoint-obstacle-candidate-001/README.md) | 栅格末点兼容修复：PID 单圆柱一次候选 |
| [grid-endpoint-candidate-comparison-001/](grid-endpoint-candidate-comparison-001/README.md) | 格子中心引导末点修复：五条候选比较 |

### 诊断与审查

| 条目 | 标题 |
| --- | --- |
| [goal-tolerance-boundary/](goal-tolerance-boundary/README.md) | 目标距离容差与 LOS 空间精度边界 |
| [localization-boundary-001/](localization-boundary-001/README.md) | fake_localization 并行数组边界：red 复现与普通安装 green |
| [los-preview-speed-diagnostic/](los-preview-speed-diagnostic/README.md) | LOS 预瞄与参考速度的有界诊断 |
| [los-waypoint-policy-diagnostic.json](los-waypoint-policy-diagnostic.json) | 诊断数据 |
| [lqr-preview2-q1-r20-diagnostic.md](lqr-preview2-q1-r20-diagnostic.md) | LQR 两秒预瞄、低位置权重候选的数值诊断 |
| [pid-course-damping-diagnostic/](pid-course-damping-diagnostic/README.md) | PID 航向微分项的有界离线诊断 |
| [pid-preview-phase-diagnostic.json](pid-preview-phase-diagnostic.json) | 诊断数据 |
| [obstacle-factor-calibration.md](obstacle-factor-calibration.md) | 障碍阈值校准：捕获输入离线诊断 |
| [obstacle-factor-calibration.json](obstacle-factor-calibration.json) | 上一条的数据 |
| [source-time-candidate-diagnostics/](source-time-candidate-diagnostics/README.md) | 源时间度量与 PID 候选诊断 |
| [normal-tf-throughput-001/](normal-tf-throughput-001/README.md) | 正常 TF 条件下的 30 秒建图吞吐补充验证 |
| [mapping-deadline-fix-receipt.json](mapping-deadline-fix-receipt.json) | 数据回执 |
| [scan-queue-deadline-review.md](scan-queue-deadline-review.md) | ScanQueue 200 ms 边界：只读审查与可区分证据 |
| [soft-escape-independent-review.md](soft-escape-independent-review.md) | Soft inflation escape 独立边界审查 |

### 源码测试结果（JUnit XML）

各次源码测试的原始结果文件，文件名后缀是当时的改动主题：

[calibrated-candidate](source-tests-calibrated-candidate.xml)、
[counter-schema-in-progress](source-tests-counter-schema-in-progress.xml)、
[factor025-candidate](source-tests-factor025-candidate.xml)、
[footprint-settling](source-tests-footprint-settling.xml)、
[goal-tolerance](source-tests-goal-tolerance.xml)、
[grid-endpoint](source-tests-grid-endpoint.xml)、
[integration-expanded](source-tests-integration-expanded.xml)、
[los-single-step](source-tests-los-single-step.xml)、
[pending-route](source-tests-pending-route.xml)、
[pid25-source-time](source-tests-pid25-source-time.xml)、
[route-reuse](source-tests-route-reuse.xml)、
[single-step-candidate](source-tests-single-step-candidate.xml)、
[snapshot-process](source-tests-snapshot-process.xml)、
[soft-inflation-escape](source-tests-soft-inflation-escape.xml)、
[static-tf-reset](source-tests-static-tf-reset.xml)、
[supervision](source-tests-supervision.xml)。

## 毫米波雷达（功能已移除）

| 条目 | 标题 |
| --- | --- |
| [mmwave-sim-001/](mmwave-sim-001/README.md) | 虚拟毫米波雷达验证记录 |
| [mmwave-map-001/](mmwave-map-001/README.md) | 毫米波地图显示修正与验证 |
| [mmwave-fov-001/](mmwave-fov-001/README.md) | 毫米波视场边框 TF 修复 |
| [lidar-only-001/](lidar-only-001/README.md) | 移除毫米波雷达后的 LiDAR 导航验证 |

## 三模块拆分与目录整理

| 条目 | 标题 |
| --- | --- |
| [distributed-navigation-001/](distributed-navigation-001/README.md) | 三模块普通导航拆分验证记录 |
| [navigation-startup-001/](navigation-startup-001/README.md) | 拆分版启动故障修复记录 |
| [module-layout-001/](module-layout-001/README.md) | 按业务模块整理功能包的验证记录 |

## 控制器

| 条目 | 标题 |
| --- | --- |
| [control-extraction-001/](control-extraction-001/README.md) | 控制开发工作区提取验证 |
| [adrc-0914-v1/](adrc-0914-v1/README.md) | 0914 PI＋ADRC 接入验证 |
| [speed-tuner-v1/](speed-tuner-v1/README.md) | 本地速度环调参台验证 |

## 路径规划

| 条目 | 标题 |
| --- | --- |
| [hybrid-astar-001/](hybrid-astar-001/README.md) | Python Hybrid A* 接入验证 |
| [hybrid-live-20260916/](hybrid-live-20260916/README.md) | Hybrid A* 路径闪烁、USV 静止：实时诊断与修复 |
| [astar-pruning-001/](astar-pruning-001/README.md) | 普通 A* 直连剪枝验证记录 |
| [astar-startup-fix-20260917/](astar-startup-fix-20260917/README.md) | A* 剪枝后无全局路径：决策节点安装依赖修复 |

## LaDDS 联调

| 条目 | 标题 |
| --- | --- |
| [ladds-communication-20260915/](ladds-communication-20260915/README.md) | LaDDS 0211 与 ROS 2 通信验证 |
| [ladds-single-leg-20260915/](ladds-single-leg-20260915/README.md) | 2026-09-15 单段航渡验证 |
| [ladds-multi-waypoint-20260915/](ladds-multi-waypoint-20260915/README.md) | 0211 多必经航点验证记录 |
| [stop-guard-20260917/](stop-guard-20260917/README.md) | 0331 主动刹停 / 0211 终点值守验证记录 |

## 任务模块（定点值守、阵位保持）

| 条目 | 标题 |
| --- | --- |
| [station-guard-001/](station-guard-001/README.md) | 定点任务交付证据 |
| [station-guard-core-001/](station-guard-core-001/README.md) | core 任务迁移验证证据 |

## 已知的失效链接

以下链接在证据原文中已经失效，按"不改证据"的约定保留原样：

- `module-layout-001/README.md` 第 17 行指向 `build/logger_all.log` 和 `build/events.log`，这两个构建日志没有入库。
- `los-preview-speed-diagnostic/README.md` 第 52 行、`pid-course-damping-diagnostic/README.md` 第 20 行指向模块整理前的源码路径。
