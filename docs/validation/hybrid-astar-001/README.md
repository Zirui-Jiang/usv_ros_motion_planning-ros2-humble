# Python Hybrid A* 接入验证

日期：2026-09-16。范围：当前 `path_planner → usv_decision → LOS` 的单目标及有序多航点链路。

## 结果

| 检查 | 结果 |
| --- | --- |
| Windows Python 3.12 / NumPy 2.4.6 / pytest 9.1.1 的相关回归 | **318 passed, 7 skipped**；跳过项缺少 ROS Python 包 |
| WSL Ubuntu 22.04、Python 3.10 / NumPy 1.21.5 / ROS 2 Humble 的数值和真实 launch 参数解析 | **273 passed** |
| 加载新构建 overlay 后的真实 ROS 决策节点初始化、单目标/多航点生命周期回归 | **13 passed** |
| ASCII 验证工作区的 colcon 构建 | **7 packages finished**，约 1 分 8 秒 |
| 从安装目录导入并实际运行 Hybrid A* | **success，102 个路径点**；独立 Hybrid YAML 已安装 |
| 无 ROS 示例，半径 4 m 的四分之一圆 | 成功，102 点；采样折线长度约 6.2831 m，理论圆弧长约 6.2832 m |

不同平台的测试范围有重叠，上表通过数量不能相加作为独立用例数。

## 测试命令

在仓库根目录运行 Windows 数值回归：

```text
python -m pytest src/core/decision_module/path_planner/test src/core/decision_module/trajectory_planner/test src/core/decision_module/usv_decision/test src/sim_env/test/test_configuration.py src/sim_env/test/test_hybrid_configuration.py src/sim_env/test/test_runtime.py -q -rs
```

WSL 的 ROS/launch 检查先加载 `/opt/ros/humble/setup.bash`，在本仓库执行：

```bash
PYTHONDONTWRITEBYTECODE=1 ROS_DOMAIN_ID=163 ROS_LOCALHOST_ONLY=1 python3 -m pytest \
  src/core/decision_module/path_planner/test \
  src/core/decision_module/trajectory_planner/test \
  src/core/decision_module/usv_decision/test/test_hybrid_runtime.py \
  src/core/decision_module/usv_decision/test/test_launch_decision.py \
  src/sim_env/test/test_configuration.py \
  src/sim_env/test/test_hybrid_configuration.py -q -p no:cacheprovider
```

加载下面新构建的 overlay 后，真实节点检查为：

```bash
PYTHONDONTWRITEBYTECODE=1 ROS_DOMAIN_ID=163 ROS_LOCALHOST_ONLY=1 python3 -m pytest \
  src/core/decision_module/usv_decision/test/test_hybrid_ros.py \
  src/core/decision_module/usv_decision/test/test_decision_ros.py \
  src/core/decision_module/usv_decision/test/test_route_decision_ros.py -q -p no:cacheprovider
```

## 安装验证与中文路径

直接在本仓库中文路径构建时，已有 `rosidl_generate_interfaces` 报
`list index: 1 out of range` 和 `Target dependency '/mnt/d/ROS2' does not exist`。
没有修改消息定义或系统 ROS 文件。将相同七个包复制到本机已授权 ASCII 目录后，构建成功。

验证工作区：

```text
C:/Users/Administrator/.codex/visualizations/2026/09/15/01a0a503-748c-7191-a6b4-8186a22cbe5e/hybrid_astar_ros2_build
```

包含：`common`、`usv_navigation_interfaces`、`path_planner`、`trajectory_planner`、
`controller`、`usv_navigation`、`usv_decision`。

在该 ASCII 工作区执行的构建命令：

```bash
source /opt/ros/humble/setup.bash
colcon --log-base log build --base-paths src --packages-up-to usv_decision \
  --executor sequential --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

复制验证不会更新用户另一个正在运行的工作区。部署时应在实际使用的 ASCII ROS
工作区同步源码并重建相关包；纯 Python 示例及数值测试不受上述 rosidl 路径问题影响。

## 审查中修正的问题

- Dubins 三圆弧在圆心几乎重合时，`acos(1-d²/8)` 存在消减误差；换成稳定的
  `2*asin(d/4)`，新增左右镜像及公共规划器回归，避免合法输入被无效短连接拒绝。
- 密集 LOS 路径的到达判断增加剩余路径长度条件，防止靠近终点时截断尚未走完的绕行。
- 单目标 Hybrid 不使用原来的直线接回旧路径；有序航点不对其进行直线捷径化。
- 新参数放到共享默认值和独立 Hybrid YAML，原共用 YAML 保持旧任务应用可读取。

## 已知边界与未验证项

- 额外执行已有 `src/sim_env/test/test_launch_modules.py` 时，两条
  `test_standalone_navigation_starts_only_its_business_module` 断言把控制器写死为
  PID，而当前项目控制器 YAML 为 ADRC，因此失败。没有修改控制器配置或这些旧断言。
  新的真实 main launch 测试已验证 Hybrid 名称及半径/分箱参数传到决策节点。
- 没有启动 Gazebo 做闭环避障、路径跟踪和耗时统计；没有实艇测试。
- 最小回转半径 4 m 是待验证的仿真初值；算法没有水动力响应、速度状态、动态目标或 COLREGs。
- 全局曲线满足设定的几何回转半径；LOS 仍产生自己的时间参考，实船和最终参考轨迹不因此获得严格半径保证。
- 现有控制器/任务完成按位置与速度判定，未增加精确终点艏向完成条件。
- 第一版 Hybrid 严格拒绝超软代价阈值起点，未复制普通 A* 的软膨胀区逃逸。
- 独立旧 `usv_mission` 的值守/绕圈流程不在本轮 Hybrid 接入范围。
