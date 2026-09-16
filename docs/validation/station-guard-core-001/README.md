# core 任务迁移验证证据

来源工作区：`/home/usv/usv_migration/station-guard-core-001`。归档时间：`2026-09-12T16:06:58.975556+00:00`。

所要求三组运行是否全部通过：**True**。发现的全部顶层运行是否都通过：**True**。
归档成功只说明证据已复制并核对哈希；失败、中断、阻塞、未运行或缺少摘要不能解释为验收通过。

| 原运行目录 | 记录通过数 / 计划数（无计划数时用记录数） | 原 result 分布 | 核对后全通过 |
|---|---:|---|---|
| `mission-matrix-001` | 21 / 21 | `{"passed": 21}` | True |
| `navigation-mission-001` | 6 / 6 | `{"completed": 6}` | True |
| `navigation-simulation-001` | 6 / 6 | `{"completed": 6}` | True |

源码测试为 **838 项范围测试通过、846 项全量测试通过**。全量测试使用了针对现有控制导出夹具的 pytest 6.2.5 收集兼容设置，原始失败日志与具体条件见 [测试上下文](raw/validation/source-tests-context.json)，没有修改控制导出代码或放宽测试断言。

本次实测数据和图件：

- [21 次试验指标](trial-metrics.csv)、[九个固定保持窗口](station-windows.csv)、[数值汇总](measured-summary.json)。
- [绕圆避障](figures/orbit_obstacle/mission_plots.png) 与 [受扰动保持](figures/station_disturbance/mission_plots.png)；完整图件位于 `figures`。七场景固定选择第 1 次重复出图，共 10 PNG 与 10 PDF，原始副本同时保存在 `raw`。
- [当前工作区与完整源码快照的核对](receipts/current-checkout-match.json)。
- [迁移说明及新入口](../../station-guard-migration.md)。

- [archive-summary.json](archive-summary.json)：逐运行真实摘要、嵌套摘要清单及记录数核对，不重复累计子运行。
- [raw](raw)：完整保留 validation 全目录（包括较早失败的构建或测试尝试）及 results 全目录；已有图件原样复制。
- [raw-evidence.tar.gz](raw-evidence.tar.gz)：上述 raw 内容的压缩副本。
- [build-source-snapshot.tar.gz](build-source-snapshot.tar.gz)：src、scripts、conftest.py、pytest.ini 全量构建源码。
- [source-snapshot.json](source-snapshot.json)：每个源码文件的 SHA256、大小、权限与总归档 SHA256。
- [receipts/boundary-check.json](receipts/boundary-check.json) 与 [receipts/moved-files.json](receipts/moved-files.json)：独立修改边界与迁移清单证据。
- [SHA256SUMS](SHA256SUMS)、[sha256-index.json](sha256-index.json)：本目录归档时全部交付文件的哈希。

源码快照包含现有依赖和并行雷达源码，以便从源码构建；快照内容不等于本次修改清单，也不表示所有并行功能都经过本轮验收。不包含编译安装产物和缓存；外部 ROS/Gazebo 系统依赖仍需按项目环境安装。

旧 `docs/validation/station-guard-001` 是独立迁移前历史证据，本工具没有读取并合并它的闭环结果，也没有修改该目录。指标表由 `summarize_metrics.py` 读取归档实测值生成，出图流程保存在 `render_migration.py`。若随后增加图表或报告，应重新调用本工具的 `write_checksums(output)` 更新清单。

在当前目录执行 `sha256sum --check SHA256SUMS` 可复核文件完整性；两个 tar 已在制作后逐成员复读校验，没有执行解包。
