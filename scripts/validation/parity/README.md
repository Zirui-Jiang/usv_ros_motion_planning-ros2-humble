# 原 C++ / Python 正常数值对照

本工具编译一个只包含接口调用的 C++ 小程序，并链接 `baseline-validation` 工作区已经安装的原算法共享库。相同 `fixtures.json` 同时供 C++ 和 Python 读取；C++ 结果保存为 `cpp_golden.tsv`，随后与真实 Python 算法调用对照。没有把 Python 重写的公式当作 C++ 结果。

在 WSL 的新终端中运行，依赖原基线已构建完成；不要先加载 Python install overlay。这里使用现有 ROS Humble、CMake、编译器、yaml-cpp 和 NumPy，不启动 Gazebo，也不启动 costmap 生命周期节点。PID/LQR 原构造函数内部需要 `rclcpp::init` 和可视化 publisher，所以 C++ 程序使用一个独立 ROS domain，随后直接调用公开控制方法，不执行节点 spin。

```bash
source /opt/ros/humble/setup.bash
source /home/usv/usv_migration/baseline/install/setup.bash
export ROS_DOMAIN_ID=198
python3 '/mnt/d/ROS2轻量仿真系统/usv_ros_motion_planning-ros2-humble/scripts/validation/parity/run.py' \
  --baseline /home/usv/usv_migration/baseline \
  --python-source '/mnt/d/ROS2轻量仿真系统/usv_ros_motion_planning-ros2-humble' \
  --output /home/usv/usv_migration/parity_new_run
```

`--output` 必须是尚不存在且位于基线工作区之外的目录，包含独立 `build/`。默认容差为 `atol=1e-6`、`rtol=1e-5`，按 NumPy `allclose(Python, C++, atol, rtol)` 比较，即每项满足 `|Python − C++| ≤ atol + rtol × |C++|`。退出码 0 表示本工具的正常用例全部通过，语义修复探针单独报告；它不表示全部算法或闭环验收通过。编译、调用异常会直接报错，保留已生成日志。

| 范围 | 覆盖与限制 |
| --- | --- |
| 路径重采样 | 原 `PathPlanner::resample` 静态接口，直线与转弯路径；比较全部 x/y/heading |
| RDP | 非空路径、正 max_interval；直线简化和转弯递归加密，比较全部输出 |
| SG | 9 点直线路径的内部 x/y 一致性；全路径另作末端位置差异探针。当前直线探针的输入航向为零，因此它不证明非零航向修复的数值结果 |
| 轨迹插值 | 全部 8 个轨迹字段为仿射序列，测试时间/距离精确节点和区间查询。仿射数据上原错误区间外推恰好等于正确插值，因此这里只证明这个受限子域；非线性数据另作差异探针 |
| PID | 两次连续调用，包含 P/I/D 状态与推力分配；正向速度、无角度跨界，dt=0.1 s |
| LQR | 两个正向、横向速度为零状态的公开控制输出，包含真实 Riccati 求解与推力限制；未独立导出 A/B/K 私有局部矩阵 |
| 未覆盖 | A*/Dijkstra/GBFS 需要真实 Costmap2DROS 生命周期；LOS 完整预测、LiDAR 建图不在本工具范围内 |

原轨迹末区间的非节点查询可能越界，工具避免执行该未定义行为。SG 末端固定及轨迹包围区间的变化作为差异证据保留，不通过放宽容差掩盖。PID 角度差分跨界、LQR 倒车/横向状态等语义修复不在正常用例范围内。

报告包含输入、输出、最大绝对差、baseline/Python 提交、Python 工作区状态、相关源文件 SHA256、C++ 可执行文件及基线共享库 SHA256；`linked_libraries.txt` 验证实际加载了指定基线 install 库。对未提交 Python 源码的运行，要以文件 SHA256 和工作区状态识别，不能只引用提交 SHA。证据副本在 `evidence/`，完整独立 WSL 构建与执行日志路径在副本说明中。
