# 同场景三次均值验收：独立补充报告

2026-09-11（Asia/Shanghai），用户在本轮正式数据采集结束后明确：最终以同控制器、同场景三次试验的**平均 RMSE 和平均到达时间**验收，同时报告每次结果与最差值。本目录按这一澄清新增独立 `scenario_mean` 报告。它不表示该聚合方式在采集前已经固定，不改写原逐次报告，也不把原来的 6/7 改称 7/7。

本次只使用固定 `3afb25f` 的正式 18 项导航与 3 项故障记录，以及对应格内 C++ 基线。可比的两个完整三次场景组均通过均值门槛；另四组不可比。功能检查为 18+3 全部通过；实际运行性能为 16 项通过、2 项证据不足。适用场景均值通过与全证据验收分开表达，机器字段 `all_passed` 保留 `false`，CLI 返回 1。

## 均值、标准差与最差值

每个场景须满足双方固定重复编号 1、2、3 全部实际 `completed`、`passed=true`，且源时间计分均为 `valid`。不能以有效子集、补跑替代编号或最好几次组成均值。主度量仍为 `source_time_trapezoid`，阈值仍为 Python/C++ ≤ 1.10；零基线仍使用既有绝对容差 1e-6。

对每次 RMSE 分别得到 \(E_1,E_2,E_3\)，再计算 \(\bar E=(E_1+E_2+E_3)/3\)。到达时间同样取三次算术均值。判定使用两个实现均值的比值，不合并所有轨迹点计算 RMSE，也不平均三次比例。标准差为样本标准差（ddof=1），不是均值的置信区间；最差值取该指标的最大值。

| 场景 | Python RMSE 均值±SD / m | C++ RMSE 均值±SD / m | 均值 RMSE 比值 | Python / C++ 平均时间 / s | 均值时间比值 | 场景结论 |
| --- | ---: | ---: | ---: | ---: | ---: | --- |
| PID 直航 | 0.05965390 ± 0.00236425 | 0.05664826 ± 0.00068801 | 1.053058 | 20.733333 / 22.500000 | 0.921481 | 通过 |
| PID 转弯 | 0.08937955 ± 0.01313906 | 0.09623559 ± 0.01831691 | 0.928758 | 20.433333 / 23.033333 | 0.887120 | 通过 |
| PID 单圆柱 | 0.20540361 ± 0.00390671 | 0.46027171 ± 0.02430308（仅描述） | 不可比 | 22.400000 / 28.366667（仅描述） | 不可比 | C++ 02/03 裕度失败，不能组成有效三次基线 |
| LQR 直航 | 0.06077053 ± 0.00127341 | 无有效基线 | 不可比 | 16.366667 / 缺失 | 不可比 | 未获得有效 C++ 结果 |
| LQR 转弯 | 0.99186526 ± 0.00552917 | 无有效基线 | 不可比 | 17.800000 / 缺失 | 不可比 | 未获得有效 C++ 结果 |
| LQR 单圆柱 | 0.40525409 ± 0.00359584 | 无有效基线 | 不可比 | 18.433333 / 缺失 | 不可比 | 未获得有效 C++ 结果 |

完整每次值、有效样本数、均值、ddof=1 标准差和最差值在 [report.md](report.md) 和 [scenario_statistics.csv](scenario_statistics.csv) 中。不可比组的已有数值仅作描述，没有计算验收比例；缺失项保留空值。`scenario_acceptance.summary` 为 `eligible_groups=2, passed_groups=2, incomparable_groups=4, all_eligible_passed=true, all_groups_comparable=false`。

## 原逐次结果与最差仍保留

| 场景 | Python 三次 source RMSE / m | C++ 三次 source RMSE / m | 各次 source 比例 | 最差 source 比例 |
| --- | --- | --- | --- | ---: |
| PID 直航 | 0.06075533 / 0.05693990 / 0.06126649 | 0.05649795 / 0.05604784 / 0.05739899 | 1.075355 / 1.015916 / 1.067379 | 1.075355 |
| PID 转弯 | 0.10450016 / 0.08289634 / 0.08074213 | 0.09421816 / 0.11547770 / 0.07901092 | **1.109130** / 0.717856 / 1.021911 | **1.109130** |
| PID 单圆柱 | 0.20089837 / 0.20785476 / 0.20745772 | 0.48812364 / 0.44931939（功能失败）/ 0.44337212（功能失败） | 0.411573 / 不可比 / 不可比 | 0.411573（仅有效 01） |

原主度量逐次判定仍是 **6/7 通过，转弯 01 超过 1.10；另 11 对不可比**。legacy 墙钟逐次结果也保留，直航 03 的 legacy 比例为 **1.1005440065324548**，仍超限；主度量转弯 01 的完整比例为 **1.1091297177599686**。圆柱 01 的有效单对保留，但不能代替完整圆柱三次场景的均值验收。不可比的 11 对分别是 9 对 LQR 基线缺失及 2 对 PID 圆柱基线失败。

[per_repeat_diagnostics.csv](per_repeat_diagnostics.csv) 包含全部 18 个固定配对键、主/legacy 比例、时间比例和原因。逐对象比对已确认新报告的 `comparisons`、`performance` 和 `trials` 与 [原正式逐次报告](../python-grid-endpoint-final-comparison/README.md) **完全相同**，见 [provenance.json](provenance.json) 的检查结果。原报告 JSON SHA-256 仍为 `c7257f7b756978dcfe6d2e0f2627b05908528bfd0a0bd48c80d6f11a9321539e`。

## 功能、性能与运行异常单列

正式记录的 18 项导航、3 项故障功能检查全部通过。18 项控制计算 P99 均小于 100 ms，18 项扫描处理 P95 均不超过同窗扫描周期预算；这些不代表所有单次计算均无超时。积压为 16 项通过、2 项 `unknown`：PID 直航 03、LQR 直航 03 在连续状态区间出现 TF 过期，保留原因 `post_drain_observations_do_not_establish_normal_tf_throughput`。各间隔过期增量分别为 `[2,2,0,0]` 与 `[1,1,0]`。不放宽 TF 0.2 s 截止或积压规则。

[独立正常 TF 吞吐实验 002](../normal-tf-throughput-001/README.md) 在受控源节点环境完成 450/450 扫描积分、450 次扫描前精确 TF 可查验证，expired/overflow/reset 均为 0，全 450 条处理耗时 P95=23.288 ms≤66.7 ms，冻结积压判据通过。这是独立 domain、合成射线负载下的补充性能证据，不替代正式矩阵两例 `unknown`，也不是当前最终干净安装的闭环验收。

运行异常仍单列：LQR 直航 01、LQR 转弯 01 各记录一次 `control_result_timeout`；20 项 Gazebo 试验中 18 项各 8 个进程正常退出，PID 转弯 02、LQR 直航 01 的 gzserver 在收尾时发生 boost self-join、退出 -6，其他进程正常。独立 odom 夹具直接返回码未记录，退出健康为 unknown。完整功能、时间序列和退出证据见 [正式矩阵原始归档](../python-grid-endpoint-final-matrix-001/README.md)。

## 输入、来源与复算

- Python 原始目录：`/home/usv/usv_migration/results/python-grid-endpoint-final-matrix-001`；被测安装提交 `3afb25f84232e560c62599a721dc0bfbe1a21657`，PID 预瞄 2.6 s、Kd_course=0、obstacle_factor=0.25。候选 Kd 或 3.0 s 预瞄数据均未纳入。
- C++ 原始目录：`/home/usv/usv_migration/results/baseline-grid-interior-001`；提交 `632734ba31478bdb833e15949287b53ae9334cf3`。
- 固定格内场景 SHA-256：`13f08a3d6e5d6712f616c3e6e2214d819bca5305c3177579439a5143eb805f18`。Python 原始运行前后的 513 个源码、680 个安装、88 个验证字段一致。本报告不是后续 M5 清理版本的新闭环证据。
- `source_time_trapezoid` 的源间隔、首尾覆盖偏移上限保持预先固定的 0.5 s，使用接收顺序冻结首条非空路径，完整源 odom 对误差平方作梯形近似，不外推、删启动或删末端。实际 27 项源数据均 valid，最大实际源间隔约 0.1 s；数据 valid 不覆盖功能失败。实际 gap/coverage/P95/max 保存在报告中。有限观测梯形近似不保证连续真实误差。
- 显式使用 `otter_fake_localization` frame 契约，仅适用于已核对 map→odom 恒等的本项目。证据见 [真实 frame 运行凭据](../localization-boundary-001/normal-grid-endpoint-launch-frame-receipt.json)；任意数据不能默认同名 frame 等价。
- 新报告 JSON SHA-256：`6160aa47ee2900b16217d8f714aa8b2936060ed88cbfe60082cf499a9600a2f7`。输入 manifest/summary、生成器、方法、输出 CSV 的哈希和完整命令见 [provenance.json](provenance.json)。

默认 CLI 和默认报告仍为 `per_repeat`，不增加场景均值字段。只有显式选项才启用新分支；176 项比较器、源时间和图 CSV 相关定向回归通过，其中新增测试先验证缺失接口为红，再验证完整三次、坏数据、零基线和默认兼容为绿。报表与算法安装版本分开记录。

```bash
python3 scripts/validation/compare_results.py \
  --python /home/usv/usv_migration/results/python-grid-endpoint-final-matrix-001 \
  --baseline /home/usv/usv_migration/results/baseline-grid-interior-001 \
  --rmse-metric source_time_trapezoid \
  --frame-contract otter_fake_localization \
  --acceptance-aggregation scenario_mean \
  --output /home/usv/usv_migration/results/scenario-mean-acceptance-recheck-001
```

输出目录必须是新的独立目录。返回 1 表示仍有不可比组或独立性能证据不完整，不能将退出码改写为六个场景全通过。图表仍可查阅原正式归档；若需要新增均值图，使用本目录的独立绘图脚本，均值±SD与三个单次点同时展示，SD不作置信区间解释。

```bash
python3 docs/validation/scenario-mean-acceptance/plot_scenario_means.py \
  --report docs/validation/scenario-mean-acceptance/report.json \
  --output /home/usv/usv_migration/results/scenario-mean-figures-recheck-001
```

绘图脚本只读取显式均值报告，不重算或选择试验。输出包含 PNG、PDF、带主度量/legacy 标签的 CSV 和来源哈希。代码交付时只完成语法检查，未在并行干净构建期间执行绘图；生成后还需独立视觉核验。新干净安装矩阵如完成，应另建报告和图目录，不能替换此历史数据归档。
