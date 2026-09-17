# 控制器独立开发工作区

此工作区是当前工作树中控制算法及其依赖的独立开发快照。保持现有 PID、LQR 与驻留控制算法源码和行为，可在不启动 ROS、Gazebo、Radar 或整套仿真系统的条件下进行数值开发、接口检查和单元测试。

`controller` 原本已经是单独的 ROS 2 Python 功能包。本次提取的边界是可单独复制的开发工作区；运行时控制循环、轨迹更新、仿真 ROS 消息转换和推进器发布仍由原工程 `sim_env` 负责。这里没有新建航控节点或实机通信驱动。导出物不会自动替代原工程中的控制器。

## 从原工程重新导出

在原仓库根目录运行：

```powershell
python scripts/control/export_control_workspace.py
```

默认生成 `deliverables/control_workspace/` 和 `deliverables/control_workspace.zip`。两者任一已存在时，脚本会拒绝覆盖，也不会自动删除。需要另一次快照时指定新的路径：

```powershell
python scripts/control/export_control_workspace.py --output deliverables/control_workspace_v2 --archive deliverables/control_workspace_v2.zip
```

复制范围是以下三个包的 Python 模块、源测试、ROS 包元数据和 resource 标记。保留当前文件内容，包括尚未提交的 `common/dynamics.py` 或 station keeping 改动；不从 Git HEAD 还原旧版本。

| 工作区路径 | 原工程来源 | 作用 |
|---|---|---|
| `src/common/` | `src/core/shared/common/` | `VesselState`、`USVModel`、`ControlCommand` 与通用数值运算 |
| `src/trajectory_planner/` | `src/core/decision_module/trajectory_planner/` | 轨迹数据结构、插值和 LOS 参考生成 |
| `src/controller/` | `src/core/control_module/controller/` | PID、LQR、驻留控制器与工厂函数 |
| `examples/control_demo.py` | `scripts/control/control_demo.py` | 一步纯数值调用示例 |
| `docs/control-autopilot-interface.md` | 原工程同名文档（存在时复制） | 航控状态回传与执行接口定义 |

缓存、`.pyc`、`.egg-info`、`.git`、运行日志、传感器采集文件和其他工程功能包不在复制范围。`provenance.json` 逐个记录文件的源相对路径、导出相对路径与 SHA256；同时记录 Git HEAD 和仅针对三个源包的工作树修改状态。生成的辅助文件没有单独源路径，使用 `generated_by` 标明来源脚本。接口文档的源码链接会适配独立目录；未导出的 `sim_env` 证据链接会改为注明“仅原仓库”的纯路径，这一变换记录在 `transformation` 字段，并同时保存 `source_sha256` 与导出 `sha256`。算法与原测试源码逐字节复制。清单不对自身计算哈希。相同输入与 Git 状态重复导出到不同目录，会得到相同 ZIP 字节；归档内部根目录固定为 `control_workspace/`。

## 在独立工作区运行

复制文件夹到其他位置，或解压 ZIP，然后进入 `control_workspace`。建议使用 Python 3.10 或更新版本。纯数值计算依赖 NumPy，测试依赖 pytest：

```powershell
python -m venv .venv
.venv/Scripts/python -m pip install -r requirements.txt
.venv/Scripts/python -I -B examples/control_demo.py --controller pid
.venv/Scripts/python -I -B examples/control_demo.py --controller lqr
.venv/Scripts/python -I -B -m pytest --confcutdir=. -q -rs -p no:cacheprovider
```

Linux 使用 `.venv/bin/python` 替换 `.venv/Scripts/python`。源目录由示例和 `conftest.py` 自动加入导入路径，不需要安装项目到全局 Python 环境。原工程也可直接运行 `python scripts/control/control_demo.py --controller pid`。

测试命令使用隔离导入与 `--confcutdir=.`，即使快照仍在原仓库的 `deliverables` 下，也不会继承原仓库的 `conftest.py`。当前 `src/common/test/test_geodesy.py` 中的 `test_world_reference_matches_current_world_and_example_point` 与 `test_unsupported_or_incomplete_world_rejected`（后者参数化为 6 项）需要完整工程的 `sim_env/worlds/ocean.world`。工作区不携带该夹具，生成的 `conftest.py` 仅对这一个精确文件路径下的这两个函数标记 `skip`，合计 7 项，并显示原因；其余地理转换数值测试照常执行。原测试文件没有改动，这 7 项应在原工程中运行，不能将跳过表述为通过。

`common`、`trajectory_planner`、`controller` 的包级 import 与示例无 ROS 依赖。`common/visualizer.py` 中的可视化转换函数需要 ROS 消息包，只有实际调用这些函数时才加载；该功能不属于纯数值示例。工作区保留原 `package.xml` 和 `setup.py`，在已配置 ROS 2 Humble 的环境中可执行 `colcon build --symlink-install --packages-select common trajectory_planner controller`；此命令构建功能包，不会产生新的运行时控制节点。

## 保持当前控制输出层级

调用入口是 `create_controller('pid'/'lqr', model, config)`，每一控制周期执行 `compute(state, trajectory, dt)`。示例使用 `dt=0.1 s` 与当前仿真 PID 非零增益、LQR 权重和时限。参数针对现有示例模型，实船需要独立辨识和标定。LQR 保留 `0.08 s` 求解时限，若运行设备超时则示例会输出 `timeout` 状态并返回非零退出码。

| 输入或输出 | 当前含义 |
|---|---|
| `VesselState(x, y, psi, u, v, r, stamp, frame_id)` | 位置、艏向、船体系速度、艏摇角速度及统一时刻/坐标系标识 |
| `Trajectory` | 每行 `[t, s, x, y, theta, curv, velocity, acceleration]`，`t` 相对 `stamp`，状态与轨迹必须处于同一坐标系 |
| `ControlCommand.left/right` | 左右推进器归一化命令，取值 `[-1, 1]`，正值正推、负值反推 |
| `ControlCommand.lateral` | 当前双推进器模型中为 `0`，不表示已有第三个执行器 |
| `ControlCommand.status/message` | 算法状态与原因；不等同于航控收到或执行确认 |

示例只输出一步计算的 JSON，没有闭环动力学运行，也没有发送到航控。该 JSON 是演示格式，不是实机报文协议；归一化命令如何映射 PWM、转速或推力，要由航控适配层结合执行器标定定义。不能把 `left/right` 的数值直接当成牛顿。

PID 实例需要跨控制周期复用以保留积分状态，开始新任务时按原调度逻辑调用 `reset()`。纯算法入口不承担反馈超时、控制权限、手动接管、通信心跳或执行器应答检查；这些需要由航控适配和调度层实现，详细定义见随包接口文档。

## 将新控制器接回原系统

1. 保留 `compute(state, trajectory, dt) -> ControlCommand` 契约，先在本工作区新增控制算法及数值测试。
2. 将确认后的算法改动移回原工程 `src/core/control_module/controller/`，有公共模型变更时同步审核 `common` 和 `trajectory_planner` 的兼容性。
3. 在 ROS 2 环境中重新构建受影响包，重新加载 `install/setup.bash`，再进行原工程集成仿真与航控联调。

源数值测试通过只说明这些计算与接口回归通过，不能替代 ROS 集成构建、实船辨识、执行器映射和通信失效验证。
