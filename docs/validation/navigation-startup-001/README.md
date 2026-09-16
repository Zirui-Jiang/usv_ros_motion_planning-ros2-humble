# 拆分版启动故障修复记录

2026-09-14，WSL Ubuntu-22.04，分支 `codex/distributed-navigation`。本记录针对当前拆分版启动后艇不运动的问题。

## 原因与证据

原失败启动记录为 [original-failed-launch.log](original-failed-launch.log)：01:40 的启动使用 `/home/usv/module-layout-20260914/install`，建图入口 `perception_mapping_node` 很快以 code 1 退出，感知监测、决策、航控仍存活。该 launch 日志没有保存 Python traceback。

随后在用户的 `bash -ic` 环境进行同文件导入复现，结果见 [interpreter-probe.json](interpreter-probe.json)：

- PATH 中的 Python 是 `/home/usv/miniconda3/bin/python3`，版本 3.13.13；通过 `/usr/bin/env python3` 导入建图 wrapper，因 `rclpy._rclpy_pybind11` 无法加载而退出 1。
- `/usr/bin/python3` 为 3.10.12；在相同 ROS/安装环境中导入同一 wrapper 成功，退出 0。
- 新 wrapper 使用 `#!/usr/bin/env python3`。建图 Node 缺少解释器 prefix，而其他 Python 业务节点已有 `prefix=[sys.executable]`。

建图退出使感知模块所要求的 `map` 输出缺失，模块无法就绪；决策/航控的就绪检查使其等待并保持零推。日志与可重复的解释器故障共同定位了拆分版的启动回归。

同时，项目根目录不存在 `install/setup.bash`，旧 `scripts/main.sh` 只检查此位置，因此不能直接选择当前独立构建目录。01:42 的另一份现场日志属于旧 C++ `sim_env_node` 入口，不用它判断本次新三模块链路是否正常。

## 修复范围

- `usv_perception/usv_perception/launch_support.py`：建图 Node 增加 `prefix=[sys.executable]`，与感知监测、决策和航控保持一致。该修复作用于 launch 启动路径；直接用 PATH Python 执行 wrapper 的导入探针仍用于展示原问题。
- `scripts/main.sh`：支持 `USV_INSTALL_DIR` 显式选择安装区；默认仍为项目内 `install`；缺失目录时报出完整路径，不猜测其他历史安装区。
- 增加启动回归用例和使用说明。导航算法、控制参数和任务行为保持原样。

## 验证结果

| 检查 | 结果 | 证据 |
|---|---|---|
| 不兼容 PATH 下的 Python 节点启动，修复前 | mapping 失败、monitor 通过 | [perception-launch-red.log](perception-launch-red.log) |
| 脚本安装区选择，修复前 | 4 failed、3 passed | [main-launcher-red.log](main-launcher-red.log) |
| 感知/总体 launch 与脚本回归，修复后 | 38 passed，退出 0 | [launch-green.log](launch-green.log) |
| 单包增量构建安装 | `usv_perception` 成功，退出 0 | [build.log](build.log) |
| 算法、六份 YAML 及既有行为函数保持 | 98/98 通过 | [preservation.json](preservation.json) |
| Conda 环境经真实脚本启动的 PID 直航 | 实际到达，`GOAL_REACHED` | [PID 结果](gazebo-conda-script-001/python_pid_straight_01/summary.json) |
| Conda 环境经真实脚本启动的 LQR 直航 | 实际到达，`GOAL_REACHED` | [LQR 结果](gazebo-conda-script-001/python_lqr_straight_01/summary.json) |

实际导航使用 [run_conda_startup.py](run_conda_startup.py) 调用原 `ros_acceptance.py` 的探针、判据和物理仿真，仅将启动命令导向真实 `scripts/main.sh`，并设置 `USV_INSTALL_DIR`。每次实际启动命令和 Conda PATH 记录在各 trial 的 `launcher-route.json`；ROS domain 为 220，Gazebo 使用测试专属端口。测试以 `gui:=false rviz:=false` 运行，不检验图形界面渲染。

起点 `(23, -10)`，目标 `(35, -10)`，PID/LQR 到达容差分别为原配置的 1.5 m/1.0 m。下发目标后约 0.349 s/0.371 s 首次观测到非零推力；导航测试墙钟用时约 26.84 s/18.23 s。这是各一次故障回归，不是性能统计实验。两次建图状态分别记录处理 354/250 帧扫描，发布统计分别有 48/33 次样本。

测试结束后，launch 日志记录其子节点和 Gazebo 正常退出。本次脚本场景生成只移除了 `ocean.world` 的末尾换行；已核对仅此差异并恢复，场景文件无本轮遗留改动。

补充探针 [observe_running.py](observe_running.py) 执行时两次仿真已结束，返回 0 条地图消息且只有观察节点，退出 1；[其原始记录](live-map-and-processes.json) 不用于证明运行期间的地图连续发布。运行期间的建图恢复证据来自上述两次 trial 的 `mapping_status` 和 launch 日志。此后未因该时序不当的探针重复导航试验。

## 当前机器启动方式

```bash
cd /mnt/d/ROS2轻量仿真系统/usv_ros_motion_planning-ros2-humble
USV_INSTALL_DIR=/home/usv/module-layout-20260914/install bash scripts/main.sh
```

当前安装区已包含修复。等待场景和地图就绪后重新下发导航目标；重启不恢复旧任务。以后使用其他构建目录时，替换 `USV_INSTALL_DIR`，或在项目内正常构建后使用脚本默认目录。
