# LaDDS 0141 目标运动上行联调

本项目的 LaDDS 桥可以把 ROS 2 的 `std_msgs/msg/String` 障碍物报文发到外部 LaDDS 主题 `0141`，
再由接收节点订阅 0141 解回 `/otter/ladds_targets`。
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

桥接层只校验根对象、`count`、`targets` 数组以及数量一致性，并保留目标对象中的其它字段。字段的实际来源、状态码、目标类型、速度/航向有效值和尺寸缺省值仍由上游障碍物适配器负责。文档中列出的 0.1 节、0.1 度和 0.1 米等量化说明，暂不在 JSON 桥接层重新编码；样例按现有 0211/0411 的 JSON 习惯填写物理量。若联调工具要求二进制定点字节，需要另加编解码层并以对端实测为准。

当前导航工程发布的是 LiDAR 栅格/代价地图（`costmap_raw`、`map`），不是带批号和经纬度的目标列表。为先验证
“仿真障碍物 → ROS → LaDDS 0141”的发送链路，仓库提供了可选的 `costmap_0141.launch.py`。
它把每一帧 `nav2_msgs/msg/Costmap` 包成一个 `targets` 项，并在该项中保留 frame、时间戳、栅格元数据
和稀疏障碍单元格。这个 JSON 是联调测试格式，不宣称这些单元格已经是船舶目标；后续接入雷达/AIS
跟踪器时，仍可让跟踪器直接向 `obstacle_input_topic` 发布正式目标 JSON。

## 1. 构建

在 WSL 的仓库根目录执行：

```bash
source /opt/ros/humble/setup.bash
colcon build --base-paths src/core/communication_module/usv_ladds_bridge \
  --packages-select usv_ladds_bridge --cmake-args -DBUILD_TESTING=ON
```

## 2. 启动 Windows 中继

在 Windows PowerShell 中保持下面的终端运行。`-ObstacleSendTopic '0141'` 会让 SDK 注册 0141 发布端。

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

脚本打印的 TCP 地址需要传给 WSL 侧。域 1 是 LaDDS 域，不能用它替代 ROS_DOMAIN_ID。
下面示例使用 ROS 域 196；如果当前仿真已经使用其它 ROS 域，桥和发布障碍物的终端必须改成同一个域。

## 3. 启动 ROS 桥

```bash
source /opt/ros/humble/setup.bash
source '/mnt/d/ROS2轻量仿真系统/usv_ros_motion_planning-ros2-humble/install/setup.bash'
export ROS_DOMAIN_ID=196
export ROS_LOCALHOST_ONLY=1
ros2 launch usv_ladds_bridge bridge.launch.py \
  backend:=tcp relay_host:=<Windows-WSL网卡地址> \
  obstacle_input_topic:=/ladds/obstacle_tx obstacle_send_topic:=0141
```

默认参数为：

```text
obstacle_input_topic=/ladds/obstacle_tx
obstacle_send_topic=0141
obstacle_json_topic=/ladds/obstacle_0141
obstacle_output_topic=/otter/ladds_targets
```

发布节点订 `obstacle_input_topic` 并发到 LaDDS `0141`，同时发 `/ladds/obstacle_0141`。
接收节点订 `/ladds/obstacle_0141`（以及 DDS 主题 0141）后解回 `obstacle_output_topic`。
规划仍使用 `costmap_raw`，本入口不把目标列表写进栅格。

实际跟踪器的 ROS 话题不同于 `/ladds/obstacle_tx` 时，在 `bridge.yaml` 或 launch 参数中把
`obstacle_input_topic` 改成跟踪器话题。消息类型目前固定为 `std_msgs/msg/String`，字符串内容是 JSON。

## 4. 启动仿真障碍物适配器

先启动仿真和感知节点，确认它们正在发布 `/otter/costmap_raw`；下面这个适配器只负责转换和转发，
不会自己生成代价地图。

桥启动后，另开一个 WSL 终端，加载同一个 ROS 2 overlay 和同一个 `ROS_DOMAIN_ID`：

```bash
source /opt/ros/humble/setup.bash
source '/mnt/d/ROS2轻量仿真系统/usv_ros_motion_planning-ros2-humble/install/setup.bash'
export ROS_DOMAIN_ID=196
export ROS_LOCALHOST_ONLY=1
ros2 launch usv_ladds_bridge costmap_0141.launch.py \
  input_topic:=/otter/costmap_raw \
  output_topic:=/ladds/obstacle_tx \
  cost_threshold:=254 \
  max_cells:=4096
```

默认只发送代价为 254 的 lethal 单元格；255 是 Nav2 的未知值，默认不当作已观测障碍物。
如果本次测试希望把未知区域也发出，增加 `include_unknown:=true`。`max_cells` 用来限制一帧
JSON 的大小，超过上限时消息会保留 `truncated:true` 和完整的 `obstacle_cell_count`，并发送前
`max_cells` 个单元格。

适配器输出的 `count` 固定为 1，表示“一帧 costmap 快照”；障碍物单元格在
`targets[0].obstacle_cells` 中，单元格式为 `{x, y, cost}`，坐标是 costmap 的列、行索引。
这和 0141 正式的单船目标字段不是一回事，但满足当前“先测试发送功能”的目的。

例如一帧中检测到两个 lethal 单元格时，报文核心结构如下：

```json
{
  "format": "costmap_snapshot_v1",
  "source": "costmap_raw",
  "count": 1,
  "targets": [{
    "kind": "costmap_snapshot",
    "frame_id": "otter/map",
    "obstacle_cell_count": 2,
    "obstacle_cells": [
      {"x": 120, "y": 240, "cost": 254},
      {"x": 121, "y": 240, "cost": 254}
    ],
    "truncated": false
  }]
}
```

## 5. 用样例包验证 0141

先确认桥已经启动，再在同一个 ROS 域的终端发送一包：

```bash
ros2 topic pub --once /ladds/obstacle_tx std_msgs/msg/String \
  '{"data":"{\"count\":1,\"targets\":[{\"batch\":1,\"absSpeed\":0.0,\"absCourse\":0.0,\"heading\":-1.0,\"longitude\":114.3931,\"latitude\":30.5453,\"distance\":25.0,\"absAzimuth\":90.0,\"relAzimuth\":0.0,\"sign\":0,\"type\":0,\"color\":0,\"state\":0,\"threatingRadius\":5.0,\"code\":\"\",\"model\":0,\"name\":\"\",\"shipLength\":-1.0,\"shipWidth\":-1.0,\"shipHeight\":-1.0,\"MMSI\":0,\"source\":1,\"areaType\":1}]}"}'
```

也可以直接使用仓库中的 [0141 样例报文](examples/ladds-0141-obstacle.json) 作为 `data` 字符串来源。
样例中的 `heading=-1`、尺寸为 `-1` 只是“未知”的联调占位，不是东湖协议已经确认的状态码。

## 6. 验收观察

ROS 侧可以看到输入消息和桥的发送日志：

```bash
ros2 topic echo /ladds/obstacle_tx --once
ros2 topic info /ladds/obstacle_tx --verbose
```

仿真运行时还可以分别确认输入和输出频率：

```bash
ros2 topic type /otter/costmap_raw
ros2 topic hz /otter/costmap_raw
ros2 topic hz /ladds/obstacle_tx
```

第一条应显示 `nav2_msgs/msg/Costmap`。如果输入话题存在但类型不同，先把适配器的
`input_topic` 改成实际的 `nav2_msgs/msg/Costmap` 话题。

最终验收应在东湖游艇联调工具中订阅域 1、主题 `0141`，确认 JSON 的 `count` 与 `targets` 到达。
桥端发送调用返回成功只说明 SDK 接收了发送请求，不代表对端业务已经解析；需要以对端订阅显示或抓包作为互通证据。

## 常见问题

- 工具收不到 0141：先检查 Windows 中继终端是否打印 `obstacle=0141`，并确认工具订阅的主题保留前导零。
- ROS 侧提示 `count must match`：`count` 必须等于 `targets` 数组长度。
- 中继能启动但发送失败：确认 ROS 桥和 Windows 中继来自同一次构建；旧中继没有 0141 注册，会拒绝或把请求发到默认主题。
- 只有 costmap 没有目标跟踪数据：本适配器发送的是带栅格单元的测试快照，不会凭空生成批号、经纬度、航向和 MMSI。正式 0141 目标字段仍应由目标检测/跟踪适配器提供。
