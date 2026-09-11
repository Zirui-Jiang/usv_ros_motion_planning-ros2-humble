# 目标距离容差与 LOS 空间精度边界

本项修复将 PID、LQR 和 ROS 配置入口统一为：**`goal_distance_tolerance` 必须有限且严格大于 `1e-9 m`**，等于边界也明确拒绝，不默默夹取。默认参数及控制计算未改变。runtime 和 LOS 算法未修改；本项未更新 WSL 安装，也未启动 Gazebo。

## 修复前的实际复现

[red-reproduction.json](red-reproduction.json) 保存修复前源码 SHA、完整路由/轨迹/命令及实际模块结果。复现通过真实 `resolve_parameters` → `plan_navigation(PlanningRequest(...))` → PID/LQR `compute` 调用完成，没有替换或 mock 内部算法。

- `R=1e-10 m`，状态与栅格中心 `(10.5,10.5)`，请求目标 `(10.5000000005,10.5)`；地图为 30×30 的空闲 `uint8` 栅格、分辨率 1 m、原点 0，状态与地图时间 10 s。
- 实际浮点距离为 `5.000000413701855e-10 m`，大于 R。runtime 追加了请求目标，路由最后两点分别为栅格中心和请求目标。
- LOS 将相邻距离不大于 `1e-9 m` 的点去重，返回栅格中心处的两行全零速度轨迹，规划状态为 `goal_reached`。runtime 保存的 `trajectory.goal` 仍是实际请求目标。
- PID 纯算法对象输出 `success`、左右推力均 0，但未报实际目标到达。LQR 纯算法对象在默认 5000 次迭代后未收敛，输出 `timeout`、左右推力均 0，残差约 5.4924。不能把两种控制器的失败状态写成相同。

原始复现脚本检查了“实际距离大于容差、轨迹速度全零、控制器不是 GOAL_REACHED、左右命令全零”，返回 0 表示成功复现缺陷，**不是通过修复验收**。随后新增的期望拒绝测试得到红结果：[red-pytest.log](red-pytest.log) / [JUnit](red-pytest.xml)：**15 failed、4 passed、34 deselected**。其中原 ROS 配置入口已拒绝零容差，但错误只给一般正值要求；新测试同时要求明确参数名和空间精度边界。

## 最小修改范围

| 生产文件 | 修改 |
| --- | --- |
| `src/core/controller/controller/controller/controller.py` | 新增共享边界常量和 `validate_goal_distance_tolerance`，显式拒绝非有限值或 ≤`1e-9` 的值 |
| `src/core/controller/controller/controller/pid_controller.py` | 参数候选全部验证后、配置赋值前调用共享校验，失败更新不改变配置和积分历史 |
| `src/core/controller/controller/controller/lqr_controller.py` | 构造阶段调用同一校验 |
| `src/sim_env/sim_env/configuration.py` | 对 PID/LQR 两个完整参数名均早期校验，覆盖项目 YAML 和 ROS 最终参数快照 |

生产 diff 合计为 21 行新增、2 行替换删除，均为校验函数或导入/调用；没有更改控制计算、路由、LOS 去重或默认配置。测试新增于已有 controller/configuration 两个测试文件。

ROS 报错示例：

```text
usv_simulation.controller.pid_controller.goal_distance_tolerance must be finite and greater than 1e-9 m (LOS waypoint spatial precision)
```

这个下界对应现有 LOS 的空间去重精度，不是建议实际航行使用纳米级容差，也不是对所有地图坐标尺度和浮点误差的精度保证。后续若改变 LOS 空间精度，需要同时审视这一接口约束；本项保留 LOS 原数值算法。

## 绿验证

[green-pytest.log](green-pytest.log) / [JUnit](green-pytest.xml)：**97 passed / 1.44 s**。范围包含 controller、configuration、现有 runtime 和全部 trajectory 测试。新增测试验证：

- PID/LQR 构造拒绝 0、`1e-10`、`1e-9`；PID 的失败参数更新具有原子性。
- ROS 参数快照和项目 YAML 入口分别拒绝两个控制器的过小容差，并指出完整参数名。
- `nextafter(1e-9,+∞)=1.0000000000000003e-9` 原值保留。该已允许容差覆盖复现中的 LOS 去重跨度，实际 PID/LQR 均能报 `GOAL_REACHED`。

真实 rclpy 启动验证在独立 `ROS_DOMAIN_ID=188`、`ROS_LOCALHOST_ONLY=1` 中串行运行六个自有短进程：PID/LQR 各验证低于、等于及刚高于边界。四次无效值由 `Simulator` 读取最终参数后抛出明确 `ValueError`；两次有效值读回原值。**六次 probe 均通过并自然返回 0**，由 probe 捕获并断言预期异常；不把返回 0 解读为无效配置成功创建了导航节点。未 spin、未发送目标或信号，每个 probe 关闭自己的 rclpy context，完整构造的节点正常销毁。

证据见 [ROS 启动 receipt](ros-startup-receipt.json)、六份 `ros-{pid,lqr}-{below,equal,above}.log` 和 `ros-logs/`；receipt SHA256：

`105c4de7efe9c61906526cafd8ae750911666ae9b424aab4bfbb8cab8437b3f0`

这是**源码树的真实 ROS 参数入口验证**，不是新构建 install 的验收。receipt 保存各 probe 的实际模块路径、PID、命令、异常/读回值、返回码和日志 SHA。

修复前后以下文件 SHA 均保持一致：

| 文件 | SHA256 |
| --- | --- |
| `src/sim_env/sim_env/runtime.py` | `0725b4bb4bf8f2648c3c9e27eca6d6c13c69fbe3e4ef1b6942dea9e61ebab1a8` |
| `src/core/trajectory_planner/trajectory_planner/los_trajectory_planner.py` | `cc309fc7eae86d458d846974d3719ee78790ae4a5222cd96c96dfc8619db3f58` |

## 复跑命令

在仓库根目录，下面定向测试不启动 Gazebo：

```bash
PYTHONDONTWRITEBYTECODE=1 python3 -m pytest -q -p no:cacheprovider \
  src/core/controller/controller/test/test_controllers.py \
  src/sim_env/test/test_configuration.py src/sim_env/test/test_runtime.py \
  src/core/trajectory_planner/test
```

实际 ROS 入口的单例复跑可使用本目录保留的 [ros_startup_probe.py](ros_startup_probe.py)，分别替换控制器与边界值；该脚本自身不改日志或生产文件：

```bash
source /opt/ros/humble/setup.bash
ROS_DOMAIN_ID=188 ROS_LOCALHOST_ONLY=1 PYTHONDONTWRITEBYTECODE=1 \
  python3 docs/validation/goal-tolerance-boundary/ros_startup_probe.py \
  --controller pid --tolerance 1e-10 --expected rejected
```

原始红证据不会被上述绿测试覆盖。主任务随后在候选 Gazebo 采集结束的空档执行完整
`run_source_tests.bash`，结果为 **423 passed / 16.57 s，退出码 0**，见
[全仓 JUnit](../source-tests-goal-tolerance.xml)。独立只读审查未发现新增可行动问题；
这一结论只覆盖该边界修复，不是任意超密路径去重正确性的证明。新安装尚待验证，
已有闭环结果、原失败记录与 M5 门槛保持不变。
