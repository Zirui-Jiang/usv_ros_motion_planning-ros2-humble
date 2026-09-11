# 干净安装版本正式验收：场景均值、逐次结果与图表

固定 `dfb1fb7` 的新普通安装已完成本轮 18 项导航与 3 项故障采集，功能检查全部通过。按采集前已固定的场景三次均值协议，有完整有效 C++ 基线的 **PID 直航、PID 转弯两组均通过**；PID 圆柱及三个 LQR 场景仍不可比。保留的逐次诊断为 **7/7 有效配对通过，另 11 对不可比**，source 与 legacy 两种逐次 RMSE 均通过。

控制和扫描计算耗时各 18/18 通过，积压证据为 **15 通过、3 unknown**；运行异常另外记录。因此机器字段 `all_passed=false`，比较器退出码为 1。适用场景均值通过不等于六个场景全部有有效基线，也不能替代积压或进程退出证据。

本目录是新的正式结果，不覆盖 [旧 `3afb25f` 逐次 6/7 报告](../python-grid-endpoint-final-comparison/README.md) 或 [用户事后澄清口径的历史均值报告](../scenario-mean-acceptance/README.md)。后续候选未混入，新旧编号未替换。完整运行与退出原件见 [新正式矩阵归档](../python-clean-release-matrix-001/README.md)。

## 固定输入与度量

- 新 Python 原始目录：`/home/usv/usv_migration/results/python-clean-release-matrix-001`；安装工作区 `/home/usv/usv_migration/python-clean-final-001`；被测提交 `dfb1fb7fcb77feb868ee87ac986c70d7c4cab537`。默认 PID 预瞄 2.6 s、Kd_course=0、obstacle_factor=0.25，未使用外部候选控制参数。
- C++ 原始目录：`/home/usv/usv_migration/results/baseline-grid-interior-001`；固定提交 `632734ba31478bdb833e15949287b53ae9334cf3`。PID 圆柱 02/03 的几何裕度失败记录保留，LQR 没有有效完成的基线。
- 格内场景 SHA-256：`13f08a3d6e5d6712f616c3e6e2214d819bca5305c3177579439a5143eb805f18`。新矩阵运行前后的 435 个源码、680 个安装、90 个验证文件字段一致；采集自然退出 0，未因事后计分替补试验。
- [收尾协议](../scenario-mean-release-protocol.md) 在本轮采集前固定。显式选择 `--acceptance-aggregation scenario_mean --rmse-metric source_time_trapezoid --frame-contract otter_fake_localization`，比例阈值仍为 1.10，零基线绝对容差仍为 1e-6。
- 均值门槛要求双方固定编号 1、2、3 全部实际 completed、passed 且源时间计分 valid。比较三次 RMSE 的算术均值之比，以及三次到达时间的算术均值之比；不是合并轨迹点的 RMSE，也不是三次比例的平均。标准差使用 ddof=1；最差值取最大值。缺失或失败的基线不能由有效子集替代。

源时间积分使用 goal 前最新 odom 及采集结束前的后续源 odom，对误差平方作非均匀时间梯形近似；首条非空路径按接收顺序冻结。最大源间隔与首尾覆盖偏移上限仍为 0.5 s，不外推、删启动、删末端或回退墙钟值补缺。本轮 18 项 Python 和 9 项 C++ 的源数据均 valid：最大实际源间隔为 0.10000000000000142 s，最大首部覆盖偏移为 8.881784197001252e-16 s，最大尾部偏移为 0.10000000000000142 s。积分范围是实际首末源观测时刻；有限采样不保证连续真实误差。P95/max 与具体覆盖逐例保存在报告和 CSV 中。

## 两个可比场景的三次均值

| 场景 | Python source RMSE 均值±SD / m | C++ source RMSE 均值±SD / m | 均值 RMSE 比例 | Python / C++ 平均到达时间 / s | 均值时间比例 | 结论 |
| --- | ---: | ---: | ---: | ---: | ---: | --- |
| PID 直航 | 0.05991755 ± 0.00084339 | 0.05664826 ± 0.00068801 | **1.057712** | 20.800000 / 22.500000 | **0.924444** | 通过 |
| PID 转弯 | 0.08569753 ± 0.00952602 | 0.09623559 ± 0.01831691 | **0.890497** | 20.633333 / 23.033333 | **0.895803** | 通过 |

均值组统计为 `eligible_groups=2, passed_groups=2, incomparable_groups=4`。PID 圆柱仅 01 有效配对，不满足双方完整三次功能通过要求，整组均值不可比。三个 LQR 场景没有有效 C++ 基线，也不可比。下面的 Python 自身均值、最差只用于描述其实际三次观测，不构成没有基线的改进结论。

| Python 场景 | source RMSE 三次值 / m | source RMSE 均值 / m | 最差 RMSE / m | 到达时间三次值 / s | 平均时间 / s | 最差时间 / s |
| --- | --- | ---: | ---: | --- | ---: | ---: |
| PID 直航 | 0.06076667 / 0.05908001 / 0.05990598 | 0.05991755 | 0.06076667 | 20.9 / 20.7 / 20.8 | 20.800000 | 20.9 |
| PID 转弯 | 0.08281936 / 0.09633075 / 0.07794248 | 0.08569753 | 0.09633075 | 20.5 / 20.8 / 20.6 | 20.633333 | 20.8 |
| PID 圆柱 | 0.20208073 / 0.20475246 / 0.20352305 | 0.20345208 | 0.20475246 | 22.5 / 22.5 / 22.7 | 22.566667 | 22.7 |
| LQR 直航 | 0.05783209 / 0.05924422 / 0.05972984 | 0.05893539 | 0.05972984 | 16.1 / 16.1 / 16.3 | 16.166667 | 16.3 |
| LQR 转弯 | 0.99736026 / 0.98838413 / 0.99219978 | 0.99264806 | 0.99736026 | 17.9 / 17.9 / 17.8 | 17.866667 | 17.9 |
| LQR 圆柱 | 0.41382340 / 0.36584272 / 0.40890017 | 0.39618877 | 0.41382340 | 18.4 / 18.5 / 18.3 | 18.400000 | 18.5 |

所有实现的三次值、有效数、均值、ddof=1 标准差、最差、legacy 值及不可比原因见 [scenario_statistics.csv](scenario_statistics.csv) 和 [完整中文报告](report.md)。

## 逐次配对全部保留

| PID 场景 / 重复 | source RMSE 比例 | legacy RMSE 比例 | 到达时间比例 | 逐次结论 |
| --- | ---: | ---: | ---: | --- |
| 直航 01 | 1.075555 | 1.038514 | 0.889362 | 通过 |
| 直航 02 | 1.054100 | 0.865123 | 0.995192 | 通过 |
| 直航 03 | 1.043676 | 1.052914 | 0.896552 | 通过 |
| 转弯 01 | 0.879017 | 0.802071 | 0.907080 | 通过 |
| 转弯 02 | 0.834194 | 0.827325 | 0.900433 | 通过 |
| 转弯 03 | 0.986477 | 0.925803 | 0.880342 | 通过 |
| 圆柱 01 | 0.413995 | 0.432142 | 0.740132 | 通过 |

最差有效逐次 source 比例为直航 01 的 **1.0755554424552285**；最差有效时间比例为直航 02 的 **0.9951923076923076**。另 11 对不可比仍写明原因。该新矩阵的 7/7 不重判旧矩阵的 6/7，也不能归因为某个未改动的控制增益改善。所有固定配对键及 source/legacy/时间比例在 [per_repeat_diagnostics.csv](per_repeat_diagnostics.csv) 中。

## 性能、时效与异常

358 条控制计时窗口中，最大已报告 P99 为 **44.8616993436009 ms**（LQR 圆柱 01），全部满足严格小于 100 ms。76 条建图计时窗口中，最大已报告处理 P95 为 **38.743185250496026 ms**（PID 直航 02），同窗扫描时间戳周期 P50 预算均为 67 ms。这里统计的是节点报告窗口分位数的最大值，不合并重叠历史样本，不把话题间隔当作算法耗时。

积压判定为 15 项通过、3 项 unknown：

| 试验 | 完整观测区间中的 expired 增量 | 判定 |
| --- | --- | --- |
| PID 转弯 03 | 0, 2, 2, 0 | unknown |
| LQR 直航 01 | 0, 0, 1, 3 | unknown |
| LQR 转弯 02 | 1, 3, 0 | unknown |

三例均按冻结规则保留 `post_drain_observations_do_not_establish_normal_tf_throughput`：连续区间出现 TF 过期，不能凭幸存扫描的等待分布证明正常 TF 吞吐。计数、窗口覆盖、排队年龄和积分进展的逐项证据见 `report.json.performance`。该结论不等于已证实算法长期积压，也不凭暂停状态推断具体扫描的 TF 失败原因。孤立 TF 过期继续按原规则记录，未因本轮结果调整截止或门槛。

[正常 TF 吞吐实验 002](../normal-tf-throughput-001/README.md) 的 450/450 扫描、处理 P95=23.288 ms≤66.7 ms、expired/overflow/reset=0 和冻结积压通过作为独立补充。源码差异核查确认 `3afb25f→dfb1fb7` 的 mapper 实现、setup.py 和 package.xml 无差异；该实验仍属于不同安装和受控输入环境，不能替代本轮三个 live unknown。

累计控制计数记录 **3 次求解超时**：LQR 直航 03、LQR 圆柱 01、LQR 圆柱 03 各 1 次。1 Hz 状态没有采到 `CONTROL_ERROR` 名称不能把计数写成零。20 项 Gazebo 试验中，18 项各 8 个子进程正常退出；PID 圆柱 01、03 的 gzserver 在 SIGINT 收尾时 boost self-join、退出 -6，其余各 7 个进程正常。所有定位和 joint_state 进程正常；独立 odom 夹具未记录直接返回码，退出健康 unknown。完整核对见 [run_audit.json](../python-clean-release-matrix-001/run_audit.json)，不由到达或耗时分位通过覆盖这些异常。

## 本轮 frame 与来源证据

使用的是本轮普通 `otter` 启动证据，不拿旧矩阵或随机 namespace 冒充：`python_pid_straight_01/launch.log` 第 22 行明确 `otter/map -> otter/odom -> otter/base_link`，日志 SHA-256 为 `8223d931726310d8b565f81018924ef45971a30c8b61fc4c2ced7f65ca67e90d`。

被测 fake localization 源文件 SHA-256 为 `ec5f0d59cc35e6f76066add8cd0e3531d422ae3d7b6f5edfc6566bf800a36068`；干净安装二进制为 `4c9c583ebbf9eeed0db5c05e41587ada7ce2443def653744be8afbda63a58378`。代码中的 map→odom 恒等转换与实际 frame 链共同支持本项目的显式契约；不能推广到任意同名 map/odom 数据。详见 [本轮 frame 凭据](../python-clean-release-matrix-001/frame_receipt.json)。基线仍使用先前已核查的固定源码和原始 frame 日志。

`report.json` SHA-256：`ee36898a8ad47b8cb337c578e8525529066db5f3b320ae2075b139a4d7f4ff6d`。输入 manifest/summary/events、生成器、场景和输出哈希见 [provenance.json](provenance.json)。正常 TF 补充、构建及真实闭环分别引用各自来源，不混合为同一批次。

## 导出图表与核验

| 内容 | PNG | PDF |
| --- | --- | --- |
| 三场景 × PID/LQR 实际轨迹 | [PNG](figures/trajectories.png) | [PDF](figures/trajectories.pdf) |
| 逐次 source RMSE / 到达时间 | [PNG](figures/navigation_metrics.png) | [PDF](figures/navigation_metrics.pdf) |
| 逐次有效配对与 1.10 门槛 | [PNG](figures/pairwise_ratios.png) | [PDF](figures/pairwise_ratios.pdf) |
| 已报告控制 P99 与预算 | [PNG](figures/control_timing.png) | [PDF](figures/control_timing.pdf) |
| 已报告扫描处理 P95 与周期预算 | [PNG](figures/mapping_timing.png) | [PDF](figures/mapping_timing.pdf) |
| 场景均值±样本 SD 与全部三次点 | [PNG](scenario_means/scenario_means.png) | [PDF](scenario_means/scenario_means.pdf) |

原五组图的 CSV 为 [轨迹](figures/trajectory_samples.csv)、[逐例指标及源覆盖](figures/trial_metrics.csv)、[计时窗口](figures/timing_windows.csv)、[逐次比值](figures/pairwise_ratios.csv)。均值图另外导出 [72 行固定位置统计表](scenario_means/scenario_means.csv)，缺少的 C++ 值为空，失败基线点仅用于描述；误差线是样本标准差，不是置信区间。四个不可比场景明确标注，不能把基线缺失画作零。

已检查六组图的文字、坐标、图例、门槛和不可比说明；六份 PDF 均单页、文字可提取，未发现文本超出页面。导航单次图曾有最高点标记上缘裁切，仅在本目录 [渲染副本](plot_acceptance_export.py) 增加顶部显示余量，冻结的 `scripts/validation/plot_acceptance.py` 未变。首次生成目录保留，修正后四份 CSV 逐字节一致；详见 [显示修正记录](rendering_adjustment.json)。最终数据一致性、导出哈希和视觉检查记录见 [export_checks.json](export_checks.json)。耗时图中的 pass 仅表示对应 timing 子项，不代表积压或全部验收通过。

## 复算与重新导出

以下命令只读原始结果，每个输出目录必须不存在。比较器返回 1 是不可比/unknown 的预期结果，绘图命令应独立执行，不因退出码跳过，也不为使退出码为零删除记录。

```bash
python3 scripts/validation/compare_results.py \
  --python /home/usv/usv_migration/results/python-clean-release-matrix-001 \
  --baseline /home/usv/usv_migration/results/baseline-grid-interior-001 \
  --rmse-metric source_time_trapezoid \
  --acceptance-aggregation scenario_mean \
  --frame-contract otter_fake_localization \
  --output /home/usv/usv_migration/results/python-clean-release-comparison-recheck-001

python3 docs/validation/python-clean-release-comparison-001/plot_acceptance_export.py \
  --report /home/usv/usv_migration/results/python-clean-release-comparison-recheck-001/report.json \
  --output /home/usv/usv_migration/results/python-clean-release-figures-recheck-001

python3 docs/validation/scenario-mean-acceptance/plot_scenario_means.py \
  --report /home/usv/usv_migration/results/python-clean-release-comparison-recheck-001/report.json \
  --output /home/usv/usv_migration/results/python-clean-release-means-recheck-001
```
