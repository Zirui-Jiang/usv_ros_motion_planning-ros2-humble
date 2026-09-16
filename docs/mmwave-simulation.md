# Otter 虚拟毫米波雷达

> 历史记录：当前 `codex/distributed-navigation` 分支已移除毫米波雷达代码、配置及启动入口。下文仅保留此前实现与验证的记录，其中的毫米波命令不适用于当前分支。当前使用说明见[三模块普通导航](distributed-navigation/README.md)。

本功能为当前 ROS 2 Humble + Gazebo Classic Otter 增加一个前向、平面、稀疏的几何雷达传感器，用于验证障碍感知和现有路径规划。无须实物雷达、CAN 或 USB-CAN。

## 使用

在 WSL 中构建当前工作树，source 对应安装区后：

```bash
source /opt/ros/humble/setup.bash
source /home/usv/usv_migration/mmwave-sim-001/install/setup.bash
ros2 launch sim_env main.launch.py obstacle_sensor:=mmwave
```

毫米波模式会加载船艏雷达模型，关闭原 LiDAR 传感器和扫描建图节点，使用毫米波点迹障碍层与原有膨胀层。路径规划、LOS 和控制模块沿用当前工程。`obstacle_sensor:=lidar` 恢复原配置；默认仍为 lidar，避免改变已有验收与任务启动方式。

不覆盖已有安装区的构建方式（输出目录必须尚不存在）：

```bash
bash scripts/validation/build_mmwave.bash /home/usv/usv_migration/mmwave-sim-001
source /home/usv/usv_migration/mmwave-sim-001/install/setup.bash
```

配置位于 `src/sim_env/config/sensors/mmwave.yaml`，支持 `mmwave_config:=绝对路径`。默认前向 120°、0.2–40 m、30 Hz；每 2° 角区间选最近表面回波，每帧最多 64 点。测距量化 0.2 m、距离噪声标准差 0.05 m、检测概率 1、随机种子 73。安装外参相对 base_link 为 (0.8, 0, 0.5) m，yaw=0。这些噪声、角区间和安装位置是可修改的仿真假设，不是设备实测性能。

## 数据链路与接口

```text
Gazebo collision geometry
  → otter/mmwave_link 上的独立 ray 传感器
  → gazebo_ros_ray_sensor（内部 raw_scan）
  → mmwave_perception_node（角区间筛选、量化、可选噪声和漏检）
  → RadarDetectionArray + PointCloud2 + RViz markers + diagnostics
  → usv_radar_costmap::RadarLayer → InflationLayer → 原有 costmap_raw
  → 当前导航节点
```

默认艇命名空间为 otter：

| 话题 | 类型 | 作用 |
|---|---|---|
| `/otter/sensors/mmwave/front/raw_scan` | sensor_msgs/msg/LaserScan | 射线插件的内部几何扫描，不直接输入导航 |
| `/otter/sensors/mmwave/front/detections` | usv_radar_interfaces/msg/RadarDetectionArray | 正式检测列表；保留源时间戳和传感器坐标系 |
| `/otter/sensors/mmwave/front/points` | sensor_msgs/msg/PointCloud2 | xyz 点迹，供障碍层和 RViz 使用 |
| `/otter/sensors/mmwave/front/markers` | visualization_msgs/msg/MarkerArray | 前向覆盖边界 |
| `/otter/sensors/mmwave/front/diagnostics` | diagnostic_msgs/msg/DiagnosticArray | 断流、拒绝帧、仿真时钟重置等状态 |

坐标轴为 x 前、y 左、z 上；平面观测 z=0 表示测量平面，不是测得目标高度。检测 ID 只在该帧内有效，不能当作船级 track ID 或 MMSI。来源标记为 `geometric_simulation_2d`。只输出完整新帧；有效零目标帧可以为空，错误帧和断流不伪造空观测。

RViz 的 `Mmwave field of view` 是固定安装传感器的覆盖边界，采用 `frame_locked=true` 跟随最新船头 TF，避免扫描与 TF 不同到达顺序导致闪烁。检测点云仍按原始采样时刻投影。修复及实际 TF 可用性记录见 [视场显示验证](validation/mmwave-fov-001/README.md)。

RViz 中通过 `Map → Mmwave obstacle map` 查看毫米波障碍代价图：话题为 `/otter/costmap`，Fixed Frame 保持 `otter/map`，地图订阅使用 Reliable / Transient Local。毫米波模式关闭了原 LiDAR 扫描建图节点，因此 `/otter/map` 地图话题没有发布者；这与仍然存在的 `otter/map` 坐标系是两回事。配置已移除旧地图显示项，并扩大默认视角。启动时 Gazebo 尚未生成船体，可能短暂出现等待地图 TF 的日志；本次记录随后恢复且代价图正常绘制，不能将这一条启动日志视为持续定位故障。实际画面和数据验证见 [地图显示验证](validation/mmwave-map-001/README.md)。当前显示的是带短期记忆的雷达障碍代价图，不是持续累积的 SLAM 地图。

## 障碍更新与断流

专用毫米波障碍层将点迹在采样时间经 TF 变换到地图，保留最近 2 s 的已观测点并重绘。完整空帧不会立即清除历史障碍；旧点按时限老化。这是有界记忆策略，不表示未检出区域已被证明无障碍。毫米波层不作 LiDAR 式射线清除。

雷达层使用仿真时间和主机单调接收时间检查新鲜度。导航另订阅毫米波点云的源时间戳；超过默认 0.5 s 无有效新帧则进入 `WAITING_SENSOR` 并发布零推力，即使 Nav2 仍在发布新时间戳的 costmap。仿真暂停同样不能持续保留新鲜状态。原 LiDAR 模式不启用这项雷达门控。

## 模型边界

- 本实现复用已安装的 Gazebo 官方开源射线插件，直接在场景碰撞表面产生命中，可处理 BOX 的局部几何偏移和射线遮挡；不使用模型中心冒充探测点。
- 当前没有 Doppler、电磁散射、RCS、海杂波、多径或毫米波穿透模型。`radial_velocity_valid` 和 `rcs_valid` 始终为 false。字段占位的 0 不能当作真实测量。
- 前向 120° 不提供原 LiDAR 的 360°覆盖。船侧、船后和视场外的障碍没有本传感器观测；既有地图仍采用项目原有未知空间配置，不应将未标障碍理解为已确认自由水域。
- 保留点随时间老化会在高动态场景产生短期痕迹；船级跟踪、速度预测及 COLREGs 不属于本次功能。
- 现有定位是平面真值 TF；波浪导致的三维姿态与扫描平面投影只适用于当前近似。三维雷达与实船误差需独立建模。

## 开源依据

采用系统依赖而非复制第三方雷达源码：

- Gazebo ROS 2 ray sensor：<https://github.com/ros-simulation/gazebo_ros_pkgs/blob/ros2/gazebo_plugins/src/gazebo_ros_ray_sensor.cpp>，源码头部 Apache-2.0。
- Ainstein Gazebo 插件：<https://github.com/AinsteinAI/ainstein_radar/tree/master/ainstein_radar_gazebo_plugins>，ROS 1 / BSD-3-Clause，未采用。消息虽然有 speed，插件没有计算它。
- nhma20 ROS 2 仿真：<https://github.com/nhma20/mmWave_ROS2_PX4_Gazebo>，作者测试 Foxy + Classic，仅 XYZ 点云，未确认明确许可证，未纳入工程。

0.2–40 m / 120°参考用户本地 SR73F 应用手册 4.1；30 Hz 为近似仿真频率。未复制厂家 PDF、DBC、驱动库或示例代码。

## 验证

纯 Python 单测覆盖扫描筛选、量化、固定种子、非法输入和时间门控；ROS 测试覆盖消息、点云、空帧、暂停/重置和导航断流门控；C++ 测试覆盖保留点、TTL、滚动地图和时钟边界。

在 source 新安装区后，独立域运行：

```bash
ROS_DOMAIN_ID=193 ROS_LOCALHOST_ONLY=1 python3 -m pytest -q \
  src/core/mmwave_perception/test src/sim_env/test/test_sensor_configuration.py \
  src/sim_env/test/test_radar_navigation_ros.py
python3 scripts/validation/mmwave_smoke.py --output /tmp/mmwave-smoke-new --domain-id 193
```

仿真 smoke 使用自建场景、独立 Gazebo 端口和进程组，只关闭自己创建的进程。

2026-09-12 已在本机 WSL Ubuntu 22.04 / ROS 2 Humble / Gazebo Classic 11 完成 17 包独立构建、686 项 Python 测试及 20 项 C++ 测试。两次真实 Gazebo 短程验证均通过，最终版本证据为 [smoke-002/summary.json](validation/mmwave-sim-001/smoke-002/summary.json)：检测/点云源时间一致，障碍和膨胀层生效，5 m 目标产生路径和推进指令；暂停毫米波处理节点期间扫描与地图继续更新、点云停止、三路推力为零，恢复后重新进入 NAVIGATING。该测试没有验证长航程任务完成或所有方向的避碰能力。

完整记录见 [毫米波验收说明](validation/mmwave-sim-001/README.md)。原有正在运行的 LiDAR 仿真进程和安装区未被替换；启动新模式前请结束原仿真，再在新 WSL 终端加载上述毫米波安装区。
