# 验收结果图表导出

`scripts/validation/plot_acceptance.py` 读取 `compare_results.py` 的 `report.json` 及其已经选定的真实 `summary.json`、`events.jsonl`，输出五组 PNG/PDF 和四张 CSV 表。应在正式 Gazebo 试验全部结束后运行，避免绘图与数据读取影响 CPU 性能观测。

本脚本不运行 ROS/Gazebo，不重新判定试验，不修改原始结果。执行前检查被引用 summary 的 SHA-256 和内容是否仍与比较报告一致；输出目录必须不存在，且不能位于任一原始运行目录内。脚本执行成功只表示图表已导出，不能替代静态、单元、ROS 集成、闭环四类验收结论。

## 执行

在能够读取 Linux 原始结果的 WSL Python 环境中，先运行比较器，再导出。以下路径中的 `python-final-comparison-001` 和 `python-final-figures-001` 是独立新目录名称示例，不声明这些结果已经生成。

```bash
python3 scripts/validation/compare_results.py \
  --python /home/usv/usv_migration/results/python-final-matrix-001 \
  --baseline /home/usv/usv_migration/results/baseline-matrix-002 \
  --output /home/usv/usv_migration/results/python-final-comparison-001

python3 scripts/validation/plot_acceptance.py \
  --report /home/usv/usv_migration/results/python-final-comparison-001/report.json \
  --output /home/usv/usv_migration/results/python-final-figures-001
```

比较器可能因为缺少有效 C++ LQR 基线等原因返回非零；保留其判定，仍可单独执行第二条命令展示真实数据。不要通过 shell 的 `&&` 将“未全部通过”变成无法导出失败证据，也不要改报告中的 `all_passed`。

上述命令保留旧 `legacy_wall` 度量。新格心终点复验需在比较命令中显式加入
`--rmse-metric source_time_trapezoid --frame-contract otter_fake_localization`，
并使用对应的格内矩阵及格内 C++ 基线；不能混用原起点矩阵。
该契约只适用于核查过的本项目 map/odom 恒等 TF，完整规则和证据见
[源时间协议](grid-endpoint-source-time-protocol.md)。绘图继承报告的度量选择，
不自行换指标。坐标轴、表格及 manifest 标明主判名称；trial CSV 同时保留两种
RMSE、源观测数量、跨度、最大间隔、首尾覆盖缺口及有效状态。unknown 保持缺失，
不会自动用 legacy 值绘成有效 source-time 分数。

渲染依赖 Matplotlib，使用无显示器的 Agg 后端，不需要 ROS 导入或 GUI。使用环境已经安装的版本并由 `figure_manifest.json` 记录实际版本；正式测量期间不安装依赖。`--help` 和源码语法检查只需要 Python 标准库。

原始目录迁移后可使用可重复的 `--path-map OLD=NEW`，例如：

```bash
python3 scripts/validation/plot_acceptance.py \
  --report /archive/comparison/report.json --output /archive/new-figures \
  --path-map /home/usv/usv_migration/results=/archive/raw-results
```

映射只改变读取位置，仍检查 summary 哈希。映射按参数顺序选择第一个匹配的目录前缀；不会递归搜索或默默选取同名替代实验。

原网格边界矩阵和整体平移后的格内矩阵必须分别生成比较报告与图表目录，不能将较好的新结果替换旧报告中的失败。脚本若发现同一场景的实际 goal 或圆柱几何有多个不同值，会拒绝混合导出。该检查利用真实事件；缺失事件仍会记录缺失，不能据此证明实验几何一致。

## 图与数据口径

| 文件 | 数据与边界 |
| --- | --- |
| `trajectories.png/.pdf` | 三行场景、两列 PID/LQR。Python 三次分别使用颜色、标记、线型；仅 completed 且 passed 的 C++ 航迹为灰色。没有有效 C++ LQR 时明确标注。圆圈为实际起点，星形为发布的目标，圆柱只取实际 `obstacle_spawned` 事件。显示艇体中心轨迹，不能独立证明全艇无碰撞。 |
| `navigation_metrics.png/.pdf` | 每次真实固定初始路径 RMSE 与到达仿真时间。保留 Python 失败记录的可用 RMSE，并标记为叉号；未到达/未完成的持续时间不作为到达时间绘制。无均值、置信区间或补零。 |
| `pairwise_ratios.png/.pdf` | 直接读取比较报告已有的逐对 Python/C++ 比例及门槛。不自行重算、配对或放宽。无有效基线时不画比例；零基线按比较报告的绝对容差保留在表中，不画成 0。 |
| `control_timing.png/.pdf` | 每个实际控制状态窗口报告的计算 P99，预算严格小于 100 ms。按首次 goal 发布后的墙钟时间排列；不将话题间隔当成计算耗时。 |
| `mapping_timing.png/.pdf` | 每个实际积分窗口报告的扫描处理 P95，与同窗口观测扫描周期 P50 预算逐点对照。只显示比较器提取的真实窗口。 |
| `trajectory_samples.csv` | 逐个原始 sample 的位姿、艇体系速度、事件行号、时间和断段编号。保留重复采样，不平滑、不重采样。 |
| `trial_metrics.csv` | 含失败或不可比记录的逐次指标、是否到达、原始持续时间、真正到达时间、最小间隙以及 summary 来源/哈希。 |
| `timing_windows.csv` | 逐个已报告耗时窗口、样本数、观测预算、积分数、性能子项判定和源事件行号。重叠窗口样本数不得相加成为独立样本总数。 |
| `pairwise_ratios.csv` | 逐对比例、双方原数值、绝对容差、是否可比、通过状态与原因；缺失值为空单元格。 |

轨迹默认为首次 `goal_published` 到 `acquisition_finished` 的实际 sample；旧事件只有 `trial_finished` 时会显式记录回退。无完整边界时记录问题并展示现有观测。相邻仿真时刻间隔超过 0.35 s、时间回退或无效位姿会断开线段；可通过 `--max-track-gap-s` 修改并写入来源清单。断段只控制连线，不删除有效观测。图中每个场景两种控制器使用相同坐标范围与等比例坐标，英文图内标注避免缺少中文字体造成乱码。

耗时图的一个点代表节点报告的一个滚动/历史窗口分位数。它不代表单次算法调用，也不是把多个窗口合并得到的整体 P99/P95。耗时图没有替代积压、丢帧、异常或输入失效的证据判定；完整性能结论仍以比较报告的 `performance` 为准。

`figure_manifest.json` 记录脚本、比较报告、原始事件、原始 summary、导出图与 CSV 的 SHA-256、解释口径、缺失问题、路径映射、Matplotlib/Python 版本和图尺寸。默认 PNG 为 300 dpi，PDF 使用嵌入 TrueType 字体；这属于通用工程导出设置，未声明符合特定期刊要求。

## 当前验证状态

准备阶段仅允许轻量语法与 CLI 检查，正式矩阵结束前不渲染。首次真实导出后仍需人工检查各图的图例、坐标范围、字体、缺失标记和数据来源；是否已实际执行、是否完成视觉检查由最终验收状态文档记录，不能从本使用说明推断通过。
