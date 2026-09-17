# A* 剪枝后无全局路径：决策节点安装依赖修复

日期：2026-09-17。针对本机 WSL Ubuntu 22.04 / ROS 2 Humble 的 LaDDS 单航段仿真安装区。

## 根因与证据

最近一次仿真在 2026-09-16 22:24:50 启动。`decision_node`（PID 52464）在启动后约
0.7 秒以退出码 1 结束；其他仿真进程继续运行，直到用户结束 launch。
原始记录见 [failed-launch.log](failed-launch.log)。本次排查时该仿真已结束，没有可直接观察的在航目标。

按 `scripts/start_ladds_single_leg_sim.bash` 的相同顺序加载系统 ROS、基线安装与
`/tmp/usv_ladds_single_leg_20260915/install`，执行真实的
`from usv_decision.decision import DecisionNode`，稳定复现：

```text
decision.py → runtime.py → path_planner_core.py:9
from path_processor import ShortcutPathProcessor
ImportError: cannot import name 'ShortcutPathProcessor' from 'path_processor'
```

修复前，`path_planner` 和 `usv_decision` 来自单航段 overlay 的 `build_python` 符号链接安装，
会随工作区源码更新；该 overlay 没有安装 `path_processor`，Python 因而加载
`/home/usv/adrc-0914-20260914-mRBVAh/install/path_processor/` 中的旧版本。
模块路径和完整异常见 [before-import.log](before-import.log)。

这发生在算法运行前：决策节点未能启动，自然无法搜索或发布 `global_path`。
启动配置虽选择 `hybrid_astar`，也要先导入同一规划模块，因此同样受影响。
`enable_shortcut` 只控制普通 A* 的后处理，不能修复导入错误。

源码中 `path_planner/package.xml` 与 `usv_decision/package.xml` 已声明新依赖，
但旧单航段构建说明使用 `--packages-select` 并遗漏了 `path_processor`、`path_planner`。
该选项不会递归构建依赖。昨日剪枝验证在全新目录构建了完整依赖闭包，
没有覆盖这套“旧基线 + 部分符号链接安装”的实际组合。

## 本次修复

- 只重建当前源码的 `path_processor`、`path_planner`、`usv_decision`，安装到实际使用的
  `/tmp/usv_ladds_single_leg_20260915/install`，继续使用原 `build_python` 构建目录。
- 补齐 [单航段构建说明](../../../src/core/communication_module/usv_ladds_bridge/README.single-leg.md)
  中的包列表和启动前导入检查。
- 保留算法、剪枝开关、碰撞检查、超时预算、控制器与默认算法选择。

实际执行命令见 [repair-command.bash](repair-command.bash)，构建结果见 [build.log](build.log)：
3 个包构建完成，退出码 0。该命令适用于上面明确列出的本机安装布局。

## 验证与复跑

修复后，在新进程中再次按同一启动顺序加载环境，三个包的 ament prefix 均来自
单航段 overlay，`ShortcutPathProcessor` 与 `DecisionNode` 导入成功，退出码 0。
详见 [after-import.log](after-import.log)。

独立安装验证脚本 [check_installed_navigation.py](check_installed_navigation.py)
不插入源码导入路径，也不加载仓库测试的 `conftest.py`。结果见
[installed-smoke.log](installed-smoke.log)，退出码 0：

| 检查 | 实测结果 |
| --- | --- |
| 普通 A* 剪枝开关对照，关闭重采样 | 同一空地图上，6 点缩为 2 点，搜索展开数均为 8 |
| Hybrid A* 弯曲连接 | 成功，返回 52 个路径点 |
| 真实 DecisionNode + 后台规划进程 + ROS 消息收发 | 收到 17 个全局路径点和 37 个轨迹参考点，节点状态为 `NAVIGATING` |

ROS 检查使用域 177、localhost-only 与独立命名空间，输入为合成空地图、里程计和目标。
脚本在 ROS 初始化前校验隔离环境；运行后销毁节点及 worker。
该检查使用测试参数（普通 A*、3 秒规划预算），验证安装后的规划与发布链路，
不代表原障碍场景、原目标或默认 1 秒预算下的完整航行结果。
未启动 Gazebo、控制器或 LaDDS，也未验证闭环跟踪、到达或实船。

加载下面的安装环境后，可复跑隔离验证：

```bash
ROS_DOMAIN_ID=177 ROS_LOCALHOST_ONLY=1 OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 \
  /usr/bin/python3 -B docs/validation/astar-startup-fix-20260917/check_installed_navigation.py
```

启动前可在 WSL 新终端检查：

```bash
source /opt/ros/humble/setup.bash
source /home/usv/module-layout-20260914/install/setup.bash
source /tmp/usv_ladds_single_leg_20260915/install/setup.bash
/usr/bin/python3 -B -c 'from path_processor import ShortcutPathProcessor; from usv_decision.decision import DecisionNode; print("DecisionNode import OK")'
```

恢复原仿真入口：

```bash
cd '/mnt/d/ROS2轻量仿真系统/usv_ros_motion_planning-ros2-humble'
bash scripts/start_ladds_single_leg_sim.bash --headless false
```

上述命令沿用安装 YAML 的 `hybrid_astar`。若要使用普通 A* 并观察本次剪枝，启动时显式选择：

```bash
bash scripts/start_ladds_single_leg_sim.bash --headless false -- planner_name:=astar
```

节点重启后需要重新下发导航目标。仿真与通信桥继续使用同一 ROS 域；本次验证不向运行中的桥发送任务。
