# 2026-09-14 ADRC 慢速诊断与时钟修复

## 现场证据

本次活动控制进程启动参数为 ADRC，速度 Kp=20、Ki=12，总推力上限 40 N。
现场只读采样见 `live-speed-active-diagnosis.json`：25 秒墙钟内收到 320 条控制报告，
其中 73 条为 SIMULATION_PAUSED；报告包含参考回调和控制周期回调，73/320 不是控制周期占比。
里程计仿真时间从 93.2 s 到 109.5 s，墙钟跨度约 24.86 s，时间倍率约 0.66。
实际纵向速度范围 0.078～0.290 m/s，船正在接近目标。

按对应参考序号、最近已收到里程计及 2.6 s 预瞄插值，参考速度约从 0.59 m/s 降低。
例如仿真时间 95.2 s 的报告附近，实测速度约 0.260 m/s、预瞄速度约 0.587 m/s、
命令对应总推力约 7.32 N。该对齐是订阅端近似重建，不是控制器内部遥测。

## 原因与改动

旧控制节点以 10 Hz 墙钟触发，单次 ROS 时间不变便进入 SIMULATION_PAUSED，发布零推力。
ADRC 的 on_control_hold 同时清空积分和滤波历史。Gazebo 低于实时运行时，重复时钟样本
是正常现象，因此节点反复误停控，Ki 的累计作用被打断。

修改 `src/core/control_module/usv_control/usv_control/control.py`：

- 记录最近一次 ROS 时钟变化对应的 monotonic 时间。
- 所有输入、对端和船体检查通过时，重复时钟在 0.5 s 墙钟窗口内保持上一条命令，
  不调用控制核心，不重复积分或追加滤波样本。
- 连续停钟达到 0.5 s 后在下一控制周期发零并执行原暂停复位逻辑。
- 取消、无效参考、输入缺失、碰撞、对端不可用和时钟倒退仍执行原停止流程。
- 暂停识别窗口作用于共享控制节点；PID/LQR 也不再因短暂重复时钟被误停推。
  三种算法的增益和控制公式未更改。真实暂停的识别延迟现在为约 0.5 s 加一个控制周期。

用户源 YAML 保持 Kp=20、Ki=12，未修改轨迹减速规则或其他参数。

## 验证

- 新增真实 ROS 节点复现：旧代码下 PID/LQR/ADRC 三例均失败于误判暂停，见 `clock-fix-red.xml`。
- 修复后节点及控制器回归：148 passed，见 `clock-fix-green.xml`；包含暂停恢复、取消、
  状态断流、时钟倒退、重复里程计、ADRC 积分/滤波历史等。
- 已同步并增量构建 `/home/usv/adrc-0914-20260914-mRBVAh` 中的 usv_control 包。
  构建成功，安装文件与测试源码 SHA-256 一致：
  `927491891f715308cb733da3e9d6df5a99a386850cc58ab15302bececca8f9db`。
  原 WSL 文件备份于该工作区 `clock-fix-backup-20260914/`。
- 独立 ROS 域 194、独立 Gazebo 端口的直线测试通过：12 m 路程，到达耗时
  22.6 s 仿真时间 / 34.81 s 墙钟，峰值纵向速度 0.7484 m/s，
  最大横向偏差 0.3527 m。见 `clock-fix-gazebo/python_adrc_straight_01/summary.json`
  和 `events.jsonl`。本次为单次验收，未构成同工况多次前后统计对比。

本次未重新跑转弯、障碍及 Gazebo 故障矩阵；用户正在运行的旧进程未停止。
修复不会消除 Gazebo 自身的低实时倍率，也不会取消接近终点时的参考减速。
调参网页验证固定参考下的数值响应，不包含此次 ROS 时钟与进程时序。

## 启用

在用户原启动终端结束旧 launch 后，在 WSL 运行：

```bash
source /opt/ros/humble/setup.bash
source /home/usv/adrc-0914-20260914-mRBVAh/install/setup.bash
ros2 launch sim_env main.launch.py world:=ocean controller_name:=adrc \
  controller_config:="/mnt/d/ROS2轻量仿真系统/usv_ros_motion_planning-ros2-humble/src/core/shared/usv_navigation/config/robots/otter/adrc_controller_params_otter.yaml"
```

已完成构建，无需用户再次构建。重新下达目标后新控制进程才使用修复代码；刷新调参网页不会刷新 ROS 节点。
