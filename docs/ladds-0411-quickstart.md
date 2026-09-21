# LaDDS 0411 本船导航

节点 `usv_ladds_rx`/`usv_ladds_tx`，内部话题 `/ladds/ownship`，DDS 仍是 `0411`。

感知 → 决策的本船导航走协议 `0411`。仿真真值不再直接给决策/航控当 `odom`；
发布节点把它编成 0411 发到 LaDDS，**接收节点订阅 0411 后再解成** `nav_msgs/Odometry`。

```text
fake_localization  /otter/odom_gt
        ↓
usv_ladds_tx   编码 0411 JSON
        ├──────────► LaDDS 主题 0411（岸基/工具）
        └──────────► /ladds/ownship
                         ↓
              usv_ladds_rx 订阅 0411 JSON
                         ↓
                   /otter/odom
              决策 / 航控订阅这份 odom
```

接收节点也会对 LaDDS 主题 `0411` 做 DDS 订阅。同一句柄通常收不到自己刚发的包，
因此仿真回灌走 ROS 话题 `/ladds/ownship`；外部发来的 0411 仍可从 DDS 解进 `/otter/odom`。

0211 / 0331 / 0052 任务链不变。`0052` 与 `0411` 仍共用一个 LaDDS 句柄。ROS 侧桥和
Windows 中继必须使用本分支对应版本。

## 启动参数

Windows 中继注册 0411 发布端和 0411 订阅端：

```powershell
.\scripts\start_ladds_relay.ps1 `
  -DomainId 1 `
  -ReceiveTopic '0211' `
  -StopReceiveTopic '0331' `
  -SendTopic '0052' `
  -OwnshipSendTopic '0411' `
  -OwnshipReceiveTopic '0411'
```

仿真真值改到 `odom_gt`，避免和接收节点解出的 `/otter/odom` 抢发布者：

```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=196
export ROS_LOCALHOST_ONLY=1
ros2 launch sim_env main.launch.py world:=ocean headless:=true \
  localization_odom_topic:=odom_gt
```

ROS 2 侧单腿联调入口：

```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=196
export ROS_LOCALHOST_ONLY=1
ros2 launch usv_ladds_bridge route.launch.py use_sim_time:=true
```

`config/route.yaml` 中已配置：

```yaml
ownship_send_topic: "0411"
ownship_receive_topic: "0411"
ownship_input_topic: "/otter/odom_gt"
ownship_json_topic: "/ladds/ownship"
ownship_output_topic: "/otter/odom"
origin_latitude: 30.5453
origin_longitude: 114.392931
origin_altitude: 0.0
```

实际船舶需要把 `ownship_input_topic` 改成定位模块的 `nav_msgs/msg/Odometry`，
并把原点改成对应 WGS84 基准点。决策和航控仍订相对名 `odom`，由接收节点发布。

## 0411 JSON

桥接层发送的业务对象使用实际物理量，字段含义如下：

```json
{
  "longitude": 114.392931,
  "latitude": 30.5453,
  "speed": 3.89,
  "heading": 90.0,
  "angularZ": 5.73
}
```

- `longitude`、`latitude`：WGS84 十进制度；
- `speed`：节，保留纵向速度的正负号，反向航行时为负值；
- `heading`：真北为 0°，顺时针增加；
- `angularZ`：度每秒。

0411 没有时间戳和横漂速度。接收节点订内部 `/ladds/ownship` 时，用发布节点缓存的源
odom 头（stamp / frame_id / child_frame_id）回填，以便和 `fake_localization` 的 TF
对齐。从 DDS 收到的外部 0411 使用接收节点时钟和配置的 `ownship_frame_id`。
解回的 `twist.linear.y` 固定为 0。

附件中的二进制定点精度暂不在 ROS JSON 层量化。

## 验证重点

先确认 `/otter/odom_gt` 持续有数据，再看 `/ladds/ownship` 和 `/otter/odom`。
决策/航控应订 `/otter/odom` 且该话题的发布者是 `usv_ladds_rx`。
外部工具仍订阅域 1、主题 `0411`。当前代码用替身 LaDDS、TCP 中继和纯转换测试验证了：

- 同一客户端可注册 0411 发布与 0411 订阅；
- 普通 `/ladds/tx` 仍发送到 `0052`；
- 0411 发送请求携带主题 `0411`；
- 本地 ENU、ROS yaw、米每秒和弧度每秒能编成 0411 再解回。
