# LaDDS 0141 目标运动上行联调

节点 `usv_ladds_radar` 随通信桥一起启动：把仿真激光雷达 `LaserScan` 聚类成 0141 目标列表，
写入 `/ladds/obstacle_in`。`usv_ladds_tx` 发到 DDS `0141` 并回灌 `/ladds/obstacle`，
`usv_ladds_rx` 再解到 `/ladds/targets`。DDS 主题仍是 `0141`。

本项目的 LaDDS 桥可以把 ROS 2 的 `std_msgs/msg/String` 障碍物报文发到外部 LaDDS 主题 `0141`，
再由接收节点订阅 0141 解回 `/ladds/targets`。
0141 与 0211、0411 共用同一个 Windows 中继和 LaDDS 句柄。

## 数据边界

东湖游艇协议文档给出了 0141 的 JSON 外壳：

```json
{
  "count": 1,
  "targets": [
    { "batch": 1, "longitude": 114.3931, "latitude": 30.5453 }
  ]
}
```

桥接层只校验根对象、`count`、`targets` 数组以及数量一致性，并保留目标对象中的其它字段。
仿真侧 `usv_ladds_radar` 用相邻有效回波聚类，把质心从本船 ENU 换到 WGS84，填入
`longitude`/`latitude`/`distance`/`absAzimuth`/`relAzimuth`。这是激光雷达观测目标，
不是 AIS 船舶跟踪；`absSpeed` 固定为 0，`heading` 为 -1。规划仍使用 `costmap_raw`，
0141 不把目标写回栅格。

## 1. 构建

在 WSL 的仓库根目录执行：

```bash
source /opt/ros/humble/setup.bash
colcon build --base-paths src --packages-select usv_ladds_bridge --cmake-args -DBUILD_TESTING=ON
```

## 2. 启动 Windows 中继

在 Windows PowerShell 中保持下面的终端运行。`-ObstacleSendTopic '0141'` 会让 SDK 注册 0141 发布端。

```powershell
.\scripts\start_ladds_relay.ps1 `
  -DomainId 1 `
  -ReceiveTopic '0211' `
  -StopReceiveTopic '0331' `
  -SendTopic '0052' `
  -OwnshipSendTopic '0411' `
  -OwnshipReceiveTopic '0411' `
  -ObstacleSendTopic '0141' `
  -ObstacleReceiveTopic '0141'
```

脚本打印的 TCP 地址需要传给 WSL 侧。域 1 是 LaDDS 域，不能用它替代 ROS_DOMAIN_ID。

## 3. 启动仿真和通信桥

雷达适配器已经包含在 `bridge.launch.py` 里，`route.launch.py` 会一并带上。
仿真真值需发到 `odom_gt`，与 0411 相同。

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=1
ros2 launch sim_env main.launch.py world:=ocean localization_odom_topic:=odom_gt
```

另开终端，同一 ROS 域：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=1
ros2 launch usv_ladds_bridge route.launch.py \
  backend:=tcp relay_host:=<Windows-WSL网卡地址> use_sim_time:=true
```

`ros2 node list` 里应有 `usv_ladds_rx`、`usv_ladds_tx`、`usv_ladds_route`、`usv_ladds_radar`。

雷达节点默认：

```text
scan_topic=/otter/sensors/lidars/lidar/scan
odom_topic=/otter/odom_gt
output_topic=/ladds/obstacle_in
```

发布节点订 `obstacle_input_topic` 并发到 LaDDS `0141`，同时发 `/ladds/obstacle`。
接收节点订 `/ladds/obstacle`（以及 DDS 主题 0141）后解回 `obstacle_output_topic`。

## 4. 报文形态

一帧扫描里聚出障碍时，核心结构如下（坐标随本船位姿变化）：

```json
{
  "count": 2,
  "targets": [
    {
      "batch": 1,
      "longitude": 114.3931,
      "latitude": 30.5453,
      "distance": 12.4,
      "absAzimuth": 90.0,
      "relAzimuth": 0.0,
      "source": 1
    }
  ]
}
```

`count` 等于 `targets` 长度，最多 255；超过时保留距离最近的目标。无回波时发送
`{"count":0,"targets":[]}`。

也可以直接使用仓库中的 [0141 样例报文](examples/ladds-0141-obstacle.json) 向
`/ladds/obstacle_in` 手发一包，用于绕过雷达节点检查桥本身。

## 5. 验收观察

```bash
ros2 topic echo /ladds/obstacle_in --once
ros2 topic hz /otter/sensors/lidars/lidar/scan
ros2 topic hz /ladds/obstacle_in
```

最终验收应在东湖游艇联调工具中订阅域 1、主题 `0141`，确认 JSON 的 `count` 与 `targets` 到达。
桥端发送调用返回成功只说明 SDK 接收了发送请求，不代表对端业务已经解析。

## 常见问题

- 工具收不到 0141：先检查 Windows 中继终端是否打印 `obstacle=0141`，并确认工具订阅的主题保留前导零。
- `usv_ladds_radar` 没有输出：确认仿真在发 `/otter/sensors/lidars/lidar/scan` 和 `/otter/odom_gt`。
- ROS 侧提示 `count must match`：`count` 必须等于 `targets` 数组长度。
- 中继能启动但发送失败：确认 ROS 桥和 Windows 中继来自同一次构建；旧中继没有 0141 注册，会拒绝或把请求发到默认主题。
