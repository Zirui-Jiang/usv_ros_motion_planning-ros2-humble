# 定点任务交付证据

本目录保存独立 WSL 工作副本的功能源码差异、自动化回归和 Gazebo 闭环证据。功能基线是 `81ab8ed1a228a23f8c1f0a1c071ef4acdfbe34dd`，开发分支为 `function/station-guard-and-keeping`。

最终 `mission-matrix-006` 已完成七场景各三次，**21/21 全部通过**，失败、阻塞和未运行均为 0。矩阵运行前后及各子任务的源码／安装文件指纹一致。对应 `priority-IzMlsY` 的 **713 项源码测试、14 包安装审计通过**，原导航六项回归也已通过。

场景指标及证据边界见 [验证报告](../../station-guard-validation.md)。目录中的早期失败或人工中断记录用于追溯修复，不计入正式 21 次；本轮统计全部来自 matrix006 的独立完整运行。

共享根工作区存在并行雷达开发。源码包与补丁由 `/home/usv/usv_migration/station-guard-001` 的冻结内容生成，只包括定点任务相对基线的差异；它们用于在干净基线上复现本功能，不能直接覆盖正在开发的共享工作区。

## 复现源码

在新的检出目录中，以基线提交创建分支，再应用本目录生成的 `feature.patch`。补丁只包含 `src/` 和 `scripts/` 下的功能及验证代码；文档与试验产物另行保存。

```bash
git checkout -b station-guard-reproduction 81ab8ed1a228a23f8c1f0a1c071ef4acdfbe34dd
git apply --check /absolute/path/to/feature.patch
git apply /absolute/path/to/feature.patch
source /opt/ros/humble/setup.bash
colcon build --parallel-workers 3 --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
bash scripts/validation/run_source_tests.bash
```

随后按照 [使用指南](../../station-guard-guide.md) 启动仿真、提交任务和执行新的验收矩阵。新试验必须采用独立输出目录，保留已有原始日志。

`archive_validation.py` 从已结束的矩阵和明确指定的构建记录生成交付材料；源码和安装指纹以运行时 manifest 为准。归档工具只整理已有测量，不修改试验的原始判定，也不将缺失数据填为通过。

## 交付文件

| 文件 | 内容与用途 |
|---|---|
| [feature.patch](feature.patch) | 相对基线的完整功能差异，已校验可应用且应用后的 Git 树与冻结源码一致 |
| [validated-feature-source.tar.gz](validated-feature-source.tar.gz) | 补丁涉及的新增与修改源码；这是差异包，完整复现应在基线上应用补丁 |
| [metrics-summary.csv](metrics-summary.csv) | 正式矩阵全部 21 次试验的紧凑指标表，可直接导入表格工具 |
| [metrics-summary.json](metrics-summary.json) | 逐次指标和各场景范围统计；只整理原始判定，不重评分 |
| [matrix-trials.csv](matrix-trials.csv) | 全部 trial 状态及完整测量 JSON，便于追溯紧凑表中的数值 |
| [archive-summary.json](archive-summary.json) | 最终计数、版本指纹、证据来源及缺失检查 |
| [source-snapshot.json](source-snapshot.json) | 基线、源码差异、补丁检查结果、运行前后源码及安装文件索引 |
| [final-source-tests.log](final-source-tests.log) / [install-audit.json](install-audit.json) | 与冻结版本对应的源码测试与 14 包安装审计 |
| [figures](figures/) | 正式矩阵中各场景第一次重复的实测图，PNG 与 PDF 同时保存 |
| [raw-evidence.tar.gz](raw-evidence.tar.gz) / [raw](raw/) | 正式矩阵、原导航回归、初测和历史失败记录的原始日志、测量、地图与 manifest |
| [SHA256SUMS](SHA256SUMS) / [sha256-index.json](sha256-index.json) | 交付文件的内容校验；索引自身及运行锁不参与自引用哈希 |

在本目录执行 `sha256sum --check SHA256SUMS` 可核对交付内容。原始证据目录保留原试验层次；同名原始文件若内容变化，归档工具会拒绝覆盖。
