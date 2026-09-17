# 格内正式矩阵：严格对照与图表

**本次总体结论仍为未通过或证据不完整。** Python 的十八项导航及三项故障用例均达到功能阈值；有效基线配对为 7 对，其中 6 对通过、1 对失败，另外 11 对不可比。十八项正常导航性能为 17 例通过、1 例证据不足。原始失败、不可比和 unknown 全部保留，未使用诊断指标替换正式指标。

比较输入是 `/home/usv/usv_migration/results/python-release-grid-001`（`e38183218fce7044e686a0f697ed13f456f49039`）和 `baseline-grid-interior-001`（`632734ba31478bdb833e15949287b53ae9334cf3`）。两者只按相同控制器、场景和重复序号配对。格内场景 SHA-256 为 `13f08a3d6e5d6712f616c3e6e2214d819bca5305c3177579439a5143eb805f18`。Python 使用固定 factor=0.25、PID 预瞄 2.6 s、既有增益及该版本 LQR 配置；本目录没有改变参数或安装。

## 正式逐对结果

主指标仍为 summary 原始 `cross_track_initial_m.rms` 和 `duration_sim_s`，两项 Python/C++ 比例均须不超过 1.10。只有双方真实 completed 且 passed 的记录才可计算比例。

| PID 场景/重复 | RMSE 比例 | 到达时间比例 | 判定 |
|---|---:|---:|---|
| 直航 01 | **1.570461** | 0.880851 | **失败：RMSE 超线** |
| 直航 02 | 1.016634 | 1.009615 | 通过 |
| 直航 03 | 1.012952 | 0.896552 | 通过 |
| 转向 01 | 0.845846 | 0.911504 | 通过 |
| 转向 02 | 0.694743 | 0.887446 | 通过 |
| 转向 03 | 0.808252 | 0.884615 | 通过 |
| 圆柱 01 | 0.412365 | 0.746711 | 通过 |
| 圆柱 02、03 | — | — | C++ 基线裕度失败，不可比 |
| LQR 三场景各三次 | — | — | 没有对应有效 C++ 结果，不可比 |

PID 直航 01 的原始 RMSE 为 0.08645912465925228 m / 0.05505332370087926 m。起始重复采样、源时间和末端几何诊断见 [diagnostics/README.md](diagnostics/README.md)，这些解释不改变该失败，也不把它改成不可比。

完整逐例数值、原始 summary 哈希、来源及排除原因见 [report.json](report.json) 和 [report.md](report.md)。比较器退出码为 1，与报告的 `all_passed=false` 一致，不是报告生成失败。所有输入目录均已结束，`all_sources_finished=true`。

## 正常导航性能

18 例控制计算时延检查均通过，最大已报告窗口 P99 为 **36.326606 ms**，门槛严格小于 100 ms。18 例建图处理时延检查均通过，最大已报告窗口 P95 为 **42.882076 ms**，同窗口观测扫描周期 P50 均为 67 ms。这些数值是窗口分位数的最大值，不是合并调用样本后的总体 P99/P95。

唯一积压证据不足项为 **LQR 直航 01**。四个原始 mapping 状态见 events 第 213、526、784、1080 行：

| 墙钟 monotonic / s | accepted | processed/integrated | expired | pending |
|---:|---:|---:|---:|---:|
| 75230.700103273 | 50 | 50 | 0 | 0 |
| 75235.969139511 | 129 | 128 | 0 | 1 |
| 75241.243211636 | 167 | 166 | 1 | 0 |
| 75246.516137885 | 234 | 229 | 3 | 2 |

三个完整同 epoch 区间跨度为 5.269、5.274、5.273 s；expired 增量分别为 0、1、2。后两个完整区间连续发生 TF 等待过期，触发试验前冻结的 `repeated_tf_expiry` 规则，因此正常 TF 吞吐证据为 unknown。四个 after-drain 窗口均通过自身覆盖/年龄检查，计数守恒，overflow、invalid_consumed 和 resets 均为 0；这些事实只能证明被观测到的剩余队列有界且有积分进展，不能反证没有扫描过期，亦不能直接证明持续计算积压。

本次未放宽 0.2 s 的 TF 截止或任何时延门槛。原始事件没有逐扫描 TF 查询结果记录，不能仅凭周期汇总把过期精确归因于某次 TF 断流、操作系统调度、时钟暂停或算法耗时。完整窗口与 counter 差分均保留在报告中。

独立原事件核查还发现时间重合：导航在第 707、748、793 行报告 SIMULATION_PAUSED（monotonic 75239.272232、75240.326734、75241.380712）；odom 源时刻 11.3→11.4 的接收墙钟间隔为 2.168097 s，11.4→11.5 为 1.008508 s。第一个过期增量 +1 的区间包含暂停的大部分，下一 +2 区间与恢复尾部重合。由于没有原始 /clock 全流和逐扫描 TF 查询日志，这只证明时间重合，不能定位每条过期扫描的原因，也不能据此把 unknown 改成通过或算法性能失败。

## 图表、表格及验证

五组图各含 PNG/PDF：[轨迹](figures/trajectories.png)、[逐次 RMSE/到达时间](figures/navigation_metrics.png)、[逐对比例](figures/pairwise_ratios.png)、[控制耗时](figures/control_timing.png)、[建图耗时](figures/mapping_timing.png)。轨迹按三场景 × PID/LQR 排列；Python 三次分别着色，灰色只表示有效 C++ 基线。圆柱 PID 仅第一例基线有效，另外两例保留在数据表，未画成有效灰线；LQR 图中明确标注缺少有效基线。轨迹表示艇体中心，不单独构成全艇安全证明。

四张源数据表是 `trajectory_samples.csv`、`trial_metrics.csv`、`timing_windows.csv`、`pairwise_ratios.csv`。缺失值保留为空，不补零；耗时图只显示节点已报告窗口，不把话题间隔当计算耗时。

已实际检查全部五张 PNG 的坐标、图例、缺失标记和裁切；五个 PDF 均为单页、文字可提取且未检测到文字越出页面。十个图文件及四张 CSV 均与 `figure_manifest.json` 的 SHA-256 一致，记录见 [export_checks.json](export_checks.json)。未修改绘图脚本或任何原始结果。

## 复现与交付边界

从包含本次验证脚本的仓库根目录执行，输出目录必须是新目录：

```bash
python3 scripts/validation/compare_results.py \
  --python /home/usv/usv_migration/results/python-release-grid-001 \
  --baseline /home/usv/usv_migration/results/baseline-grid-interior-001 \
  --output /home/usv/usv_migration/results/python-release-grid-comparison-NEW

python3 scripts/validation/plot_acceptance.py \
  --report /home/usv/usv_migration/results/python-release-grid-comparison-NEW/report.json \
  --output /home/usv/usv_migration/results/python-release-grid-figures-NEW
```

两条命令独立执行：比较结论非通过时仍需导出真实失败证据，不用 `&&` 隐去图表。

运行、构建、安装指纹与退出健康的原始记录见 [正式运行归档](../python-release-grid-001/README.md)。其中 PID 圆柱 01 的 gzserver self-join 异常和 PID 圆柱 03 的 fake_localization 异常发生于采集后的退出阶段；它们不能被 21/21 功能通过覆盖，也不作为本比较器重新定义功能主分数的依据。进程 CPU/RSS 在 report 中仅为已观测各节点数据，完整进程树资源覆盖由独立旁路报告审核。
