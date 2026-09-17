# 栅格终点版本正式对照与图表

本轮 `3afb25f` 的 18 项导航和 3 项故障功能检查全部通过；固定源时间主度量的 7 对有效 C++ 基线中，6 对通过、1 对失败，另有 11 对不可比。18 项控制耗时和扫描耗时门槛通过，积压证据为 16 项通过、2 项 unknown。因此本归档的总体状态为 **未通过完整验收**，不能用于宣布 M5 清理完成。

原始功能、进程退出和指纹审计见 [正式矩阵归档](../python-grid-endpoint-final-matrix-001/README.md)。机器可读结果是 [report.json](report.json)，逐项中文结果是 [report.md](report.md)。原始记录、门槛、重复编号和既有失败报告均未改写。

## 固定协议与输入

- Python：`/home/usv/usv_migration/results/python-grid-endpoint-final-matrix-001`，安装提交 `3afb25f84232e560c62599a721dc0bfbe1a21657`；PID 预瞄 2.6 s，项目 obstacle_factor 为 0.25。
- C++：`/home/usv/usv_migration/results/baseline-grid-interior-001`，提交 `632734ba31478bdb833e15949287b53ae9334cf3`。9 项 PID 中 7 项完成且通过；圆柱 02/03 的几何裕度检查失败，不能成为有效配对基线。没有有效完成的 LQR 基线。
- 格内场景 SHA-256：`13f08a3d6e5d6712f616c3e6e2214d819bca5305c3177579439a5143eb805f18`。原协议边界场景、早期候选与本轮不混合。
- 运行前后校核的 513 个源码、680 个安装和 88 个验证文件字段一致；完整原始指纹由矩阵归档保存。
- 主度量固定为 `source_time_trapezoid`，显式使用 `--frame-contract otter_fake_localization`；原 `legacy_wall` 同时保留。该契约仅适用于已核查的本项目 map→odom 恒等变换，不能推广至任意同名 frame。新版本证据见 [定位 frame 运行凭据](../localization-boundary-001/normal-grid-endpoint-launch-frame-receipt.json)，基线源码和日志证据随比较方法记录。
- 先按接收顺序冻结第一条非空全局路径。最新的 goal 前 odom 和采集结束前的全部后续源 odom 参与计分；重复源时间戳只有同姿态才能去重。对误差平方作离散梯形积分，以实际首末源时间跨度归一化；最大源间隔、首尾覆盖偏差上限均为预先固定的 0.5 s。
- 积分域为实际首末观测源时间，允许上述有限边界偏差，既不外推边界，也不保证未观测时刻的连续真实误差。报告同时保留实际 gap、覆盖、P95 和最大观测误差。源时间计分降低仿真暂停在墙钟采样中的重复权重，不能抹去 legacy 退化。

18 项 Python 与 9 项 C++ 的源时间数据质量均 valid；最大实际源间隔为 0.10000000000000142 s，最大首部覆盖偏差为 8.881784197001252e-16 s，最大尾部偏差为 0.10000000000000142 s。数据质量 valid 不会将功能失败的 C++ 基线改判为可比。

## 七对有效 PID 配对

下表均为 Python/C++；RMSE 和到达时间各自必须不超过 1.10，才通过对应配对。未挑选重复编号。

| 场景 / 重复 | Python 源 RMSE / m | C++ 源 RMSE / m | 源 RMSE 比值 | legacy RMSE 比值 | 到达时间比值 | 主协议结论 |
|---|---:|---:|---:|---:|---:|---|
| 直航 01 | 0.06075533 | 0.05649795 | 1.075355 | 1.058845 | 0.885106 | 通过 |
| 直航 02 | 0.05693990 | 0.05604784 | 1.015916 | 1.080811 | 0.990385 | 通过 |
| 直航 03 | 0.06126649 | 0.05739899 | 1.067379 | **1.100544** | 0.896552 | 通过；legacy 超限 |
| 转弯 01 | 0.10450016 | 0.09421816 | **1.109130** | 0.969893 | 0.907080 | **源 RMSE 超限** |
| 转弯 02 | 0.08289634 | 0.11547770 | 0.717856 | 0.711897 | 0.883117 | 通过 |
| 转弯 03 | 0.08074213 | 0.07901092 | 1.021911 | 1.045719 | 0.871795 | 通过 |
| 单圆柱 01 | 0.20089837 | 0.48812364 | 0.411573 | 0.579140 | 0.736842 | 通过 |

主度量和 legacy 均为 6/7，但失败项不同，不能互相替换：主度量转弯 01 的完整比值为 1.1091297177599686，legacy 直航 03 为 1.1005440065324548。11 对不可比包括 9 对 LQR 缺少有效 C++ 结果及 2 对 PID 圆柱基线裕度失败；缺失或失败均不计为通过。

## 耗时、积压和异常

18/18 控制计算 P99 小于 100 ms；18/18 扫描处理 P95 不大于同窗观测扫描周期。357 条控制计时窗口中，最大已报告 P99 为 36.627058898739044 ms；78 条建图计时窗口中，最大已报告 P95 为 40.802888750476995 ms，扫描周期 P50 预算均为 67 ms。这些是节点报告窗口分位数的最大值，重叠历史窗口没有合并成池化 P99/P95，消息发布间隔也没有代替算法耗时。

积压检查 16/18 通过，下列两例按冻结的连续 TF 过期规则保留 unknown：

| 试验 | 状态快照 / 完整间隔 | 完整观测跨度 / s | 各间隔 expired 增量 | 各间隔成功积分增量 | 结论 |
|---|---:|---:|---|---|---|
| Python PID 直航 03 | 5 / 4 | 21.304715 | 2, 2, 0, 0 | 31, 64, 80, 79 | unknown |
| Python LQR 直航 03 | 4 / 3 | 15.972575 | 1, 1, 0 | 61, 37, 79 | unknown |

两例计数守恒、覆盖和 after_drain 窗口检查均符合要求，overflow 为 0；但连续区间出现 TF 过期，现有证据不能证明正常 TF 条件下的吞吐稳定，原因码为 `post_drain_observations_do_not_establish_normal_tf_throughput`。该结论只覆盖有限观测窗口。没有逐扫描 TF/clock 因果记录，不能断言过期由某一类算法积压或仿真暂停引起；独立正常 TF benchmark 的通过也不覆盖这两例的 unknown。

累计控制结果计数还记录了 **2 次单次求解超时**：LQR 直航 01 和 LQR 转弯 01 各 `control_result_timeout=1`。直航 01 在约 80.3253 ms 求解后产生零推力，后续恢复并到达；转弯 01 的低频状态没有采到 `CONTROL_ERROR` 名称，但累计计数明确为 1。因此控制 P99 门槛通过不能写成零超时。完整计数见 [run_audit.json](../python-grid-endpoint-final-matrix-001/run_audit.json)。

20 项 Gazebo 试验中，18 项各 8 个子进程正常退出；PID 转弯 02、LQR 直航 01 的 gzserver 在 SIGINT 收尾时发生 boost self-join、退出 -6，其余各 7 个进程正常。所有定位和 joint_state 进程正常。独立 odom 夹具未记录直接返回码，其退出健康为 unknown。上述收尾和控制异常独立保留，不能由功能通过覆盖。

## 图表与核验

五组图各导出 300 dpi PNG 和单页矢量 PDF；轨迹按 3 场景×PID/LQR 分面，Python 的 3 次重复使用不同颜色，仅有效完成且通过的 CPP PID 使用灰色。LQR 面板明确写出无有效完成基线。

| 内容 | PNG | PDF |
|---|---|---|
| 实际轨迹 | [PNG](figures/trajectories.png) | [PDF](figures/trajectories.pdf) |
| 导航 RMSE 与到达时间 | [PNG](figures/navigation_metrics.png) | [PDF](figures/navigation_metrics.pdf) |
| 固定重复的 10% 配对门槛 | [PNG](figures/pairwise_ratios.png) | [PDF](figures/pairwise_ratios.pdf) |
| 控制实际报告 P99 与预算 | [PNG](figures/control_timing.png) | [PDF](figures/control_timing.pdf) |
| 扫描实际报告 P95 与观测周期 | [PNG](figures/mapping_timing.png) | [PDF](figures/mapping_timing.pdf) |

CSV 分别为 [轨迹观测](figures/trajectory_samples.csv)、[逐例指标和源覆盖](figures/trial_metrics.csv)、[逐计时窗口](figures/timing_windows.csv)、[逐对比值](figures/pairwise_ratios.csv)。耗时图标题中的 pass 仅表示对应 timing 子项，不表示积压、进程退出或全部验收通过。

已逐图检查坐标、图例、不可比说明、门槛、文字和边界；五图无裁切问题。所有 PDF 为单页、文字可提取、没有文本跨度超出页面；CSV 保留主度量名称与 legacy 字段。哈希、数据一致性和视觉检查见 [export_checks.json](export_checks.json)，完整输出索引见 [figure_manifest.json](figures/figure_manifest.json)。

`report.json` SHA-256：`c7257f7b756978dcfe6d2e0f2627b05908528bfd0a0bd48c80d6f11a9321539e`。

`figure_manifest.json` SHA-256：`e65223109f4200f9b9da49e69369f6d4b281d7dfdec35a35bb69550b06c8386d`。

## 复算

在本仓库、具备 Python 数值/绘图库的 WSL 环境执行；`--output` 必须指向新的独立目录，不能覆盖原始结果或本归档。比较器返回非零是本轮真实失败/unknown 的预期结果。

```bash
python3 scripts/validation/compare_results.py \
  --python /home/usv/usv_migration/results/python-grid-endpoint-final-matrix-001 \
  --baseline /home/usv/usv_migration/results/baseline-grid-interior-001 \
  --rmse-metric source_time_trapezoid \
  --frame-contract otter_fake_localization \
  --output /home/usv/usv_migration/results/python-grid-endpoint-final-comparison-001

python3 scripts/validation/plot_acceptance.py \
  --report /home/usv/usv_migration/results/python-grid-endpoint-final-comparison-001/report.json \
  --output /home/usv/usv_migration/results/python-grid-endpoint-final-figures-001
```

本归档没有更新算法、安装、判定门槛或原始采样，也没有将事后分段诊断加入正式主分数。
