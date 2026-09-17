# 控制开发工作区提取验证

日期：2026-09-12。验证对象为当前工作树导出的独立控制开发快照，未修改原控制算法行为。

- [航控接口说明](../../control-autopilot-interface.md)
- [独立工作区使用说明](../../../deliverables/control_workspace/README.md)
- [独立工作区 ZIP](../../../deliverables/control_workspace.zip)
- [来源与哈希清单](../../../deliverables/control_workspace/provenance.json)

## 验证结果

| 检查 | 实际结果 |
| --- | --- |
| 原工程 common、trajectory_planner、controller 源测试 | 144 passed，1.15 s |
| 导出工具测试 | 8 passed，1.51 s |
| 独立工作区隔离导入测试 | 137 passed，7 skipped，1.30 s |
| PID 独立示例 | success；left=0.6390822784810122，right=0.38591772151898773，lateral=0 |
| LQR 独立示例 | success；left=0.5143689040205139，right=0.38098131609473174，lateral=0 |
| 文件一致性 | 45 个清单条目及 provenance.json；逐项导出哈希、源哈希验证通过，ZIP 与目录文件逐字节一致 |
| 文档链接 | 原文与独立快照文档的本地 Markdown 链接均存在；快照中的 sim_env 证据标为原仓库路径 |
| ZIP 大小 | 68,185 bytes |

ZIP SHA256：

```text
a02ea136c0dc12eb519e4a675bc736f6af520a51e1f6f6f160188f629df9f6c6
```

## 复现命令

在原仓库根目录运行：

```powershell
python -m pytest -q src/core/common/test src/core/trajectory_planner/test src/core/controller/controller/test
python -B -m pytest -q scripts/control/test_export_control_workspace.py
```

在独立工作区根目录运行：

```powershell
python -I -B -m pytest --confcutdir=. -q -rs -p no:cacheprovider
python -I -B examples/control_demo.py --controller pid
python -I -B examples/control_demo.py --controller lqr
```

`-I` 与 `--confcutdir=.` 避免继承原仓库导入路径及 conftest，检验使用的是快照源码。示例为一次数值计算，输出无量纲，不是实测推力。

## 验证范围

7 个跳过用例来自原 `common/test/test_geodesy.py` 的两个测试函数：`test_world_reference_matches_current_world_and_example_point`（1 项）与 `test_unsupported_or_incomplete_world_rejected`（参数化 6 项）。它们依赖完整仿真的 `sim_env/worlds/ocean.world`；独立包未携带该夹具。跳过标记由导出的 conftest 精确按文件路径和函数名施加，原测试源码没有修改。这 7 项已在原工程的 144 项测试中通过，其余地理转换数值测试在独立工作区继续执行。

导出工具的 8 项验证覆盖复制范围与哈希、拒绝覆盖文件/目录、ZIP 确定性、归档不能写入输出目录或源包、文档链接转换与来源哈希、跳过规则不会掩盖其他失败。

本次未进行 ROS 2 colcon 构建、Gazebo 闭环、真实航控协议联调或实艇试验。原工程的控制调度仍在 sim_env；此交付是可独立开发、测试和携带的控制器工作区。
