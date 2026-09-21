# LaDDS 0141 雷达障碍物上行联调

当前仿真没有独立的目标跟踪雷达消息，障碍物传感器输出是
`sensor_msgs/msg/LaserScan`。0141 的输入链路因此固定为：

```text
/otter/sensors/lidars/lidar/scan
        ↓  usv_ladds_scan（TF 变换 + 有效束聚类）
/ladds/obstacle_in（std_msgs/msg/String，JSON）
        ↓  usv_ladds_tx
LaDDS 主题 "0141"
```

Nav2 仍然把同一份扫描送入 `/otter/costmap_raw` 做规划避障；costmap 只服务导航，
不再参与 0141 报文生成。

## 报文边界

桥接层只检查 `count`、`targets` 及数量一致性。扫描适配器为每一组连续有效束生成一个
临时几何目标，示例结构如下：

```json
{
  "format": "lidar_targets_v1",
  "source": "laser_scan",
  "count": 1,
  "targets": [{
    "batch": 1,
    "absSpeed": -1.0,
    "absCourse": -1.0,
    "heading": -1.0,
    "longitude": 114.3931,
    "latitude": 30.5453,
    "distance": 12.4,
    "absAzimuth": 90.0,
    "relAzimuth": 0.0,
    "sign": 0,
    "type": 0,
    "color": 0,
    "state": 0,
    "threatingRadius": 5.0,
    "code": "",
    "model": 0,
    "name": "",
    "shipLength": -1.0,
    "shipWidth": -1.0,
    "shipHeight": -1.0,
    "MMSI": 0,
    "source": 1,
    "areaType": 1,
    "localX": 12.4,
    "localY": 0.0,
    "pointCount": 4
  }]
}
```

`distance`、`relAzimuth`、`absAzimuth` 和经纬度来自扫描几何与 TF。单帧扫描不能得到目标
速度、航向、MMSI 或船舶尺寸，因此这些字段用 `-1` 或 `0` 表示未知/未提供；这不是正式
目标跟踪结果。`relAzimuth` 是相对船艏、左正的 ROS 角度（度），`absAzimuth` 是真北顺时针
角度（度）。`format`、顶层 `source`、`localX`、`localY` 和 `pointCount` 是本次测试的
扩展元数据，正式协议字段确定后可按对端 schema 收敛。连续束的距离跳变超过
`cluster_range_jump` 时会拆成不同目标。

## 1. 构建

在 WSL 的仓库根目录执行：

```bash
source /opt/ros/humble/setup.bash
colcon build --base-paths src/core/communication_module/usv_ladds_bridge \
  --packages-select usv_ladds_bridge --cmake-args -DBUILD_TESTING=ON
source install/setup.bash
```

修改源代码后必须重新构建并重新 `source install/setup.bash`；否则可能仍运行旧安装区里的
旧适配器。

## 2. 启动 Windows 中继

在 Windows PowerShell 保持中继运行，发送和接收主题都指定为字符串 `0141`：

```powershell
.\scripts\start_ladds_relay.ps1 `
  -DomainId 1 `
  -ReceiveTopic '0211' `
  -SendTopic '0052' `
  -OwnshipSendTopic '0411' `
  -OwnshipReceiveTopic '0411' `
  -ObstacleSendTopic '0141' `
  -ObstacleReceiveTopic '0141'
```

记录脚本输出的 Windows-WSL TCP 地址，下面以 `<relay-host>` 表示。

## 3. 启动 ROS 2 桥

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=196
export ROS_LOCALHOST_ONLY=1
ros2 launch usv_ladds_bridge bridge.launch.py \
  backend:=tcp relay_host:=<relay-host> \
  obstacle_input_topic:=/ladds/obstacle_in \
  obstacle_send_topic:=0141
```

`usv_ladds_tx` 订阅 `/ladds/obstacle_in`，验证 JSON 后原样发送到 LaDDS `0141`，同时把
原文镜像到 `/ladds/obstacle`。接收侧（若启用）把 0141 解回 `/ladds/targets`。

## 4. 直接启动雷达到 0141 的适配器

如果还没有启动全链路仿真，先直接运行项目的组合 launch（不需要额外脚本）：
该命令要求项目的完整 ROS 2 overlay 已构建；只构建通信桥不会安装 `sim_env` 和导航依赖。

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=196
export ROS_LOCALHOST_ONLY=1
ros2 launch sim_env main.launch.py headless:=true gui:=false rviz:=false use_sim_time:=true
```

确认它建立了 `/otter/sensors/lidars/lidar/scan` 和 TF
`otter/map ← otter/lidar_link` 后，再在另一个终端启动适配器：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=196
export ROS_LOCALHOST_ONLY=1
ros2 launch usv_ladds_bridge scan_0141.launch.py \
  input_topic:=/otter/sensors/lidars/lidar/scan \
  target_frame:=otter/map \
  output_topic:=/ladds/obstacle_in \
  use_sim_time:=true
```

默认原点沿用东湖联调配置 `latitude=30.5453, longitude=114.392931`。如果仿真世界使用
其它地理原点，在 launch 参数中同步覆盖 `origin_latitude`、`origin_longitude` 和
`origin_altitude`。适配器找不到扫描时刻的 TF 会暂时丢弃该帧并限频告警，不会把未定位的
点伪装成经纬度。

## 5. 启动顺序与观察

建议顺序是：Windows 中继 → ROS 桥 → 仿真全链路 → `scan_0141.launch.py`。检查输入和输出：

```bash
ros2 topic type /otter/sensors/lidars/lidar/scan
ros2 topic hz /otter/sensors/lidars/lidar/scan
ros2 topic echo /ladds/obstacle_in --once
ros2 topic hz /ladds/obstacle_in
```

第一条应为 `sensor_msgs/msg/LaserScan`，输出应为 `std_msgs/msg/String`，字符串内部应有
`format=lidar_targets_v1`、`count` 和 `targets`。最后在联调软件中订阅域 1、主题 `0141`
核对同一份 JSON；ROS 发布成功或 SDK `send` 返回成功只表示本地提交成功，不能单独证明对端
业务已经解析。

## 常见问题

- **只有 `/otter/costmap_raw`，没有 `/ladds/obstacle_in`**：启动的是 Nav2 感知链路，
  还没有启动 `scan_0141.launch.py`。
- **适配器提示 TF 不存在**：确认仿真使用 `otter` 命名空间、扫描的 `header.frame_id` 是
  `otter/lidar_link`，并检查 `ros2 run tf2_ros tf2_echo otter/map otter/lidar_link`。
- **没有检测到目标**：LaserScan 中 `NaN`、`inf` 或超出 `[range_min, range_max]` 的束会被
  丢弃；仿真障碍物应在雷达量程内。
- **联调软件收不到**：确认中继的 `-ObstacleSendTopic` 保留前导零 `0141`，并确认 ROS 桥
  使用的是本次重新构建后的 install overlay。
