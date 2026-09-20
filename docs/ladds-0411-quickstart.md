# LaDDS 0411 本船导航上行

本阶段不改变感知、决策和航控三个业务模块之间已有的 ROS 2 通讯。`0411` 由现有
`usv_ladds_bridge` 直接订阅 `/otter/odom`，转换成 JSON 后通过同一个 LaDDS 连接发到
外部主题 `0411`。

外部链路如下：

```text
/otter/odom
    ↓
usv_ladds_bridge（WGS84 / 节 / 真北艏向转换）
    ↓
LaDDS 0411
```

原有任务链保持不变：外部 `0211` 进入 `/ladds/rx`，外部 `0331` 进入
`/ladds/stop_rx`，实际完成反馈 `0052` 由 `/ladds/tx` 发出。`0052` 和 `0411` 共用一个
LaDDS 句柄，只按不同主题注册发布端，不需要再启动第二个 Windows 中继。ROS 侧桥和
Windows 中继必须使用本分支对应版本；旧中继只认识无主题字段的发送帧，会把 0411 请求
误发到默认发送主题。

## 启动参数

Windows 中继使用同一个进程注册两个发布主题：

```powershell
.\scripts\start_ladds_relay.ps1 `
  -DomainId 1 `
  -ReceiveTopic '0211' `
  -StopReceiveTopic '0331' `
  -SendTopic '0052' `
  -OwnshipSendTopic '0411'
```

ROS 2 侧仍使用原来的单腿联调入口：

```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=196
export ROS_LOCALHOST_ONLY=1
ros2 launch usv_ladds_bridge single_leg.launch.py use_sim_time:=true
```

`config/single_leg.yaml` 中已配置：

```yaml
ownship_send_topic: "0411"
ownship_input_topic: "/otter/odom"
origin_latitude: 30.5453
origin_longitude: 114.392931
origin_altitude: 0.0
```

实际船舶或其他仿真场景需要把 `ownship_input_topic` 改成定位模块实际发布的
`nav_msgs/msg/Odometry` 话题，并把原点改成对应的 WGS84 基准点。

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

附件中的二进制定点精度暂不在 ROS JSON 层量化；将来如果外部工具要求定点二进制，
应在 LaDDS 编解码层单独增加量化规则。

## 验证重点

先确认 `/otter/odom` 持续有数据，再让外部 DDS 订阅主题 `0411`。当前代码已经用替身
LaDDS、TCP 中继和纯转换测试验证了：

- `0052` 与 `0411` 可以在同一个客户端上注册为两个发布主题；
- 普通 `/ladds/tx` 仍发送到 `0052`；
- 0411 发送请求携带主题 `0411`；
- 本地 ENU、ROS yaw、米每秒和弧度每秒能够转换成 0411 的字段单位。

真实 Windows SDK 对“同一句柄注册两个发布主题”的现场互通仍需要用外部工具确认。
