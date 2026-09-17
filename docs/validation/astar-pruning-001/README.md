# 普通 A* 直连剪枝验证记录

日期：2026-09-16。分支：`ziruijiang/astar-pruning`。起始快照：`b9cd7fc`。
下列结果对应本轮未提交工作树，不把起始提交本身当作剪枝实现版本。

## 实现范围

- `path_processor.shortcut` 新增纯函数及 `ShortcutPathProcessor`；规划层传入同一地图的碰撞判定与截止时间。
- 单目标：搜索、原路径验证、剪枝、重采样与复检。多航点：先接精确端点，再逐航段处理一次，不跨必经点。
- `enable_shortcut` 仅切换普通 A*。纯 Core 默认 false，普通导航默认 true，旧 mission 的代码默认 false；公共 YAML 接受并开启该布尔键，但算法选择仍为 `hybrid_astar`。
- A* 搜索实现、Hybrid A* 圆弧、LOS 与船体扫掠检查未改。Dijkstra/GBFS 的既有多航点简化行为保留。
- 新依赖方向为 `path_planner → path_processor`、`usv_decision → path_processor`；处理器不反向导入规划包。

使用与开关说明见 [路径规划 README](../../../src/core/decision_module/path_planner/README.python.md#普通-a-路径剪枝)。

## 验证结果

| 检查 | 结果 | 范围 |
| --- | --- | --- |
| 修改前 Windows 全量基线 | 1279 通过、37 跳过、6 失败 | 先记录已有失败，不顺带修复 |
| 修改后 Windows 全量 | 1340 通过、37 跳过、6 失败 | 失败项与基线相同，新增 61 项测试通过 |
| Windows 针对性回归 | 390 通过 | 处理器、普通/Hybrid 规划、多航点、路径复用、配置、旧任务运行层 |
| WSL Python 3.10 / ROS Humble 回归 | 423 通过、0 跳过 | 上述 390 项及 33 项真实 ROS 决策/任务/发布检查，域174、localhost-only |
| 独立安装 | 8 个包构建完成；独立导入与行为检查通过 | 非 symlink 安装，从安装目录导入处理器/规划器/配置/决策及新导航服务 |
| 安装后的空地图对照 | 原路径6点 → 剪枝2点；展开数均为8 | 起点(.5,.5,.3)，终点(5.5,3.5,-.6)，未开启重采样 |
| 源码直跑入口 | 3 个隔离子进程检查通过 | Hybrid 示例及两处离线回放入口，移除外部 PYTHONPATH 并使用 `-I -B` |
| 独立代码审查 / 差异检查 | 无未解决必须修复项；`git diff --check` 通过 | 审查发现的三个入口漏依赖问题已修复并补测试 |

安装检查脚本另外显式拒绝 Python 优化模式，防止 `-O` / `PYTHONOPTIMIZE` 跳过断言后误报通过；在同一安装区验证了优化模式被拒绝、正常模式通过。

各行测试有重叠，不能相加为独立用例总数。Windows 使用 Python 3.12；WSL 使用 Ubuntu 22.04、Python 3.10.12、NumPy 1.21.5、pytest 6.2.5。

测试覆盖无障碍直连、绕墙、未知格和软阈值、短暂穿越单元、对角切角、直线/弯折软起点逃逸、首尾位姿/输入保持、原路径及完整结果复检、搜索预算不能重置、候选检查耗时、精确必经点、单次处理、关闭剪枝、非 A* 行为不变，以及两份历史软起点地图快照和剪枝×重采样组合。

开发顺序记录：新增处理器用例先23项失败（缺少功能），实现后处理器包48项通过；Core/配置用例先13项失败、10项已有契约通过，接入后通过；多航点新增用例先2项失败，再实现接入并通过；三个独立入口先复现导入失败，补依赖路径后通过。

## 复跑

在仓库根目录执行全量源码测试（Windows 有上述已有失败）：

```powershell
$env:PYTHONDONTWRITEBYTECODE='1'
$env:QT_QPA_PLATFORM='offscreen'
python -m pytest -q -p no:cacheprovider --tb=no
```

针对性回归：

```text
python -m pytest -q -p no:cacheprovider src/core/decision_module/path_processor/test src/core/decision_module/path_planner/test src/core/decision_module/usv_decision/test/test_runtime_route.py src/sim_env/test/test_runtime.py src/sim_env/test/test_shortcut_configuration.py src/core/mission_module/usv_mission/test/test_task_runtime.py
```

独立安装验证（在 WSL/Bash 中运行，不启动节点）：

```bash
bash scripts/validation/check_astar_shortcut_install.bash
```

脚本每次创建新的 `/tmp/usv-astar-pruning.XXXXXX`，复制本轮8个包并从声明的依赖闭包构建。不覆盖现有安装、不使用 symlink、不操作运行中的域42。输出目录保留供审计。本轮成功目录为 `/tmp/usv-astar-pruning.YnnLvu`，构建日志在其中的 `log/`。7个 Python 包的 stderr 仅提示禁用字节码编译，并非构建失败。

运行本轮 ROS 回归时，先加载系统 ROS 与脚本打印的新安装区，再设置隔离域；不要把临时示例路径当作其他机器的固定安装位置：

```bash
source /opt/ros/humble/setup.bash
source /tmp/usv-astar-pruning.YnnLvu/install/setup.bash
export PYTHONDONTWRITEBYTECODE=1 OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1
export ROS_DOMAIN_ID=174 ROS_LOCALHOST_ONLY=1
python3 -B -m pytest -q -p no:cacheprovider src/core/decision_module/path_processor/test src/core/decision_module/path_planner/test src/core/decision_module/usv_decision/test/test_runtime_route.py src/sim_env/test/test_runtime.py src/sim_env/test/test_shortcut_configuration.py src/core/mission_module/usv_mission/test/test_task_runtime.py src/core/decision_module/usv_decision/test/test_decision_ros.py src/core/decision_module/usv_decision/test/test_route_decision_ros.py src/core/decision_module/usv_decision/test/test_global_path_publication_ros.py src/core/decision_module/usv_decision/test/test_route_phase_cache_ros.py src/core/decision_module/usv_decision/test/test_hybrid_ros.py
```

## 已有失败及未验证项

Windows 全量的6项已有失败未在本任务中修复：

- `usv_mission/test/test_task_configuration.py` 中2项：旧任务配置不接受现有 ADRC 参数。
- `sim_env/test/test_adrc_configuration.py::test_adrc_defaults_resolve_and_remain_startup_only`：既有力矩限幅默认值与断言不一致。
- `sim_env/test/test_configuration.py::test_original_configuration_loads_and_aliases_work`：既有 obstacle_factor 配置0.05与断言0.25不一致。
- `sim_env/test/test_hybrid_configuration.py::test_common_planner_yaml_remains_usable_by_legacy_mission_app`：旧任务算法枚举不接受现有 Hybrid 配置。
- `scripts/validation/test_harness_options.py::test_safety_boundary_stops_remaining_matrix_and_faults`：Windows 上触发 Linux/WSL 专用验收入口限制。

第一次把独立构建输出放在中文工作区 `build/astar-pruning-package.LSfDlq` 时，Humble `rosidl_adapt_interfaces.cmake` 的 `file(STRINGS)` 未指定 UTF-8，导致7行 `.idls` 元组在中文处被拆开，随后按冒号拆分时报错。换用全新 ASCII 输出/源码目录后成功，未修改系统 ROS、接口定义或用户已有安装。

未重启现有仿真，未进行 Gazebo 闭环跟踪、实船验证、全工作区重新构建或硬实时测量。截止时间是合作式检查；重采样等既有步骤在返回后复核，不能据此宣称整条导航链保证在指定毫秒内中断。中心线剪枝不等于船体或动力学可行；原轨迹船体安全门仍必须保留。
