# 既有控制导出测试缺陷与基线环境差异

结论：此项失败可在未改动基线中复现。控制导出器及其测试在新工作树与基线中的 SHA256 完全一致；没有证据表明它由三模块拆分、根 conftest 或 ROS 安装状态引入。按 Q8 单独记录，本次不修改导出工具或其测试。

只复跑同一个测试：

`scripts/control/test_export_control_workspace.py::test_standalone_collection_skips_only_known_world_fixture_cases`

| 源码与安装环境 | PYTEST_ADDOPTS | 外层测试退出码 | 实际结果 |
|---|---|---:|---|
| 新工作树、新 install | 明确 unset | 1 | 该例失败；子 pytest 退出码 4，缺少 src/trajectory_planner/test |
| 基线源码、基线 install | 明确 unset | 1 | 相同失败 |
| 新工作树、新 install | -o testpaths=src/common/test | 0 | 1 passed |
| 基线源码、基线 install | -o testpaths=src/common/test | 0 | 1 passed |

四次均为 pytest 6.2.5；此版本将 `--version` 输出到 stderr，完整版本结果已保存在各运行 JSON 中。每次的完整 shell 命令、工作目录、退出码、stdout、stderr 分别保存在同名前缀的 `.command.txt`、`.json`、`.stdout.log`、`.stderr.log` 中。汇总见 [runs-summary.json](runs-summary.json)，源码比对见 [source-hashes.json](source-hashes.json)。

原因：测试的 `source_tree` 夹具仅创建 common/test，没有创建 trajectory_planner/test 和 controller/test。导出器生成的 pytest.ini 固定列出这三个目录。子进程没有显式测试路径，因而在缺失目录处以 usage error 退出，尚未执行测试希望验证的精确 skip 逻辑。`python -I` 不会清除 PYTEST_ADDOPTS；该环境变量仍影响子 pytest。

此前 **846 passed 的实际覆盖范围确实是 src scripts，而且包含本例**。不能改述为“846 次没有运行 scripts/control”。保存的真实命令是：

```bash
ROS_DOMAIN_ID=215 ROS_LOCALHOST_ONLY=1 PYTEST_ADDOPTS="-o testpaths=src/common/test" bash src/core/usv_mission/validation/run_source_tests.bash src scripts --junitxml=validation/full-source-tests.xml
```

外层显式指定 src scripts，所以该环境覆盖不缩小外层收集范围；内层无位置参数，继承的 testpaths 覆盖使它仅收集实际存在的 src/common/test，避开另外两个缺失目录。原 JUnit 有 846 个 testcase，其中本例存在且没有 failure/error/skip 子节点。证据见 [原始命令](original-baseline-full-source-test-command.txt)、[原始日志](original-baseline-full-source-tests.log)、[原始 JUnit](original-baseline-full-source-tests.xml)、[范围核查](baseline-scope.json)。

因此，基线“846 passed（带上述环境覆盖）”与新版本“不带覆盖时本例失败”不能直接作为回归对照；本次四个单例对照已经隔离了差异来源。另保存的 original-baseline-source-tests.log 是较早一次“Workspace is not built”记录，不是 846 的成功证据。

后续若单独修复此测试，可补全夹具内另外两个包的可导出测试占位文件，使默认收集路径真实存在；仅创建空目录不够，因为导出器只复制文件。也可让这个只验证 common/geodesy skip 的子调用显式选择 src/common/test。两者均未在本次实施，本次也未再次运行全套测试或改动算法。
