# 本次代码交付验证

日期：2026-09-14。本记录仅描述此次打包检查；历史验证报告仍保持原样。

## 已执行检查

| 检查 | 结果 | 证据 |
| --- | --- | --- |
| 当前工作树复制 | 2,107 个来源文件逐字节一致，包括未提交改动 | [来源清单](SOURCE_PROVENANCE.json) |
| ROS 功能包 | 20 个，无重复包名，包声明解析成功 | [结构检查](validation/structure.json) |
| 配置与资源语法 | 26 份 YAML、41 份 world/SDF/URDF/xacro XML 解析成功 | [结构检查](validation/structure.json) |
| Python 语法 | `src/` 与 `scripts/` 共 184 个 Python 文件 AST 解析成功 | [语法检查](validation/python-syntax.json) |
| 源码及 ROS 回归 | **935 通过，0 失败，0 错误，0 跳过**，pytest 退出码 0，耗时 36.78 秒 | [日志](validation/source-tests.log)、[JUnit](validation/source-tests.xml)、[退出码](validation/exit-code.txt) |

测试副本位于独立的 Linux `/tmp/usv-handover-20260914-MBpzUc` 目录，不依赖原仓库 `.git` 或构建缓存来发现源码；复制内容包含源码、测试脚本、文档与根测试配置。测试通过 `--confcutdir=.` 限定配置读取范围，并采用 `ROS_DOMAIN_ID=231` 与 `ROS_LOCALHOST_ONLY=1`。

测试复用本机 `/opt/ros/humble` 和 `/home/usv/module-layout-20260914/install`，后者提供现有已编译消息类型、ROS 安装资源与安装区审计对象。Python 源码测试使用本次复制的目录；需要安装区的检查针对上述既有安装区。本次没有从交付源码重新生成一个全新安装区，因此不能将本次测试结果表述为“交付包已全新编译并完成整套 Gazebo 验收”。

环境记录：WSL2 Ubuntu，Python 3.10.12，NumPy 1.21.5，PyYAML 5.4.1，pytest 6.2.5。实际命令、运行目录和环境变量见 [command.json](validation/command.json)。`PYTEST_ADDOPTS='-o testpaths=src/common/test'` 沿用项目既有导出夹具要求；外层显式收集 `src scripts`，没有以该设置缩减外层收集范围。

## 与历史验证源码的区别

与 `docs/validation/module-layout-001/source-after-manifest.json` 的 679 项历史清单逐项比较，有 674 项匹配，以下 5 项内容不同：

- `README.md`
- `scripts/main.sh`
- `src/core/perception_module/usv_perception/test/test_launch.py`
- `src/core/perception_module/usv_perception/usv_perception/launch_support.py`
- `src/sim_env/worlds/ocean.world`

这些是打包开始前已有的内容，打包时原样保留。历史报告中的 20 包构建、926 项测试与场景回归，只能按其对应源码和环境解释。本次新增回归结果为上表所列 935 项，不能据此反推已重新验证修改后的 Gazebo 场景。比较结果见 [HISTORICAL_COMPARISON.json](HISTORICAL_COMPARISON.json)。

## 完整性与本次未验证项

归档生成时再次核对来源文件与工作树一致性，为全部归档文件（清单自身除外）生成 SHA-256，并核对 ZIP CRC 和归档内文件哈希。压缩包外另有 `.zip.sha256`；接收方解压后运行根目录 `VERIFY_PACKAGE.py`，即可独立检查文件完整性。

本次未重新执行全新 colcon 构建、完整 Gazebo 场景矩阵、GUI/RViz 显示、Docker 构建、跨机部署或实船/真实传感器联调。接收方应按根 README 在自己的环境完成依赖安装、全新构建及相应集成测试。
