# LaDDS 0141 目标运动联调

航路联调配置 `route.yaml` 按"只收不发"处理 0141：

- **收**：`usv_ladds_rx` 订阅 DDS 主题 `0141`，剥掉 `head`/`content` 外壳，把业务对象转到 `/ladds/targets`；
  同时用解 0411 的同一个原点把目标经纬度换成地图坐标，画成障碍栅格 `/otter/ladds/obstacle_grid`。
  感知 costmap 的 `ladds_obstacle_layer` 叠加这张栅格，决策和航控按现有逻辑作废被挡的轨迹并重新规划。
- **发**：暂不由本系统负责。`usv_ladds_radar` 仍把仿真激光聚类成 0141 目标，但 `route.yaml` 把它的输出改到
  `/ladds/lidar_targets`，发布节点不再转发到 LaDDS `0141`；0141 回灌 `/ladds/obstacle` 也已关闭。恢复上行见第 6 节。

0141 与 0211、0411 共用同一个 LaDDS 连接（Windows 中继或 native SDK）。

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
接收侧画障碍只用每个目标的 `latitude`、`longitude` 和 `threatingRadius`；速度、航向等字段目前不做预测。

## 1. 构建

launch 读取安装区里的 YAML，改了 `route.yaml` 或 costmap 配置后都要重新构建。在 WSL 的仓库根目录执行：

```bash
source /opt/ros/humble/setup.bash
colcon build --base-paths src --packages-select usv_ladds_bridge usv_perception --cmake-args -DBUILD_TESTING=ON
```

## 2. 启动 Windows 中继（仅 tcp 后端）

`start_ladds_relay.ps1` 与 `start_ladds_route_relay.ps1` 默认都登记了 0141 的接收。手动指定时：

```powershell
.\scripts\start_ladds_relay.ps1 `
  -DomainId 1 `
  -ReceiveTopic '0211' `
  -StopReceiveTopic '0331' `
  -BerthReceiveTopic '0212' `
  -SendTopic '0052' `
  -StatusSendTopic '0051' `
  -OwnshipSendTopic '0411' `
  -OwnshipReceiveTopic '0411' `
  -ObstacleSendTopic '0141' `
  -ObstacleReceiveTopic '0141'
```

启动行里应有 `obstacle-rx=0141`。域 1 是 LaDDS 域，不能用它替代 ROS_DOMAIN_ID。

## 3. 启动仿真和通信桥

推荐用现成脚本，它们会同时打开感知的 0141 障碍层：

```bash
bash scripts/start_ladds_route_sim.bash   # 仿真和感知/决策/航控，已带 ladds_obstacles:=true
bash scripts/start_ladds_route.bash       # 另一终端：LaDDS 桥和航路适配器
```

双容器联调用 `scripts/start_ladds_split_nav.bash`；启用适配器时，`ladds_split_nav.launch.py` 自动打开 `ladds_obstacles`。

手动启动时，仿真入口要带上 `ladds_obstacles:=true`：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=1
ros2 launch sim_env main.launch.py world:=ocean localization_odom_topic:=odom_gt ladds_obstacles:=true
```

另开终端，同一 ROS 域：

```bash
ros2 launch usv_ladds_bridge route.launch.py \
  backend:=tcp relay_host:=<Windows-WSL网卡地址> use_sim_time:=true
```

启动后应看到：

- 感知日志：`[usv_perception] 0141 obstacle layer: /otter/ladds/obstacle_grid`
- 接收节点日志：`0141 obstacle grid: /otter/ladds/obstacle_grid in otter/map ...`
- `ros2 node list` 里有 `usv_ladds_rx`、`usv_ladds_tx`、`usv_ladds_route`、`usv_ladds_radar`

## 4. 收到 0141 后的处理

每收到一包 0141，接收节点依次：

1. 剥外壳、校验 `count` 与 `targets`。坏包丢弃并警告，已发布的栅格不变。
2. 用 `usv_ladds_rx` 的 `origin_latitude`/`origin_longitude`（与解 0411 为 `/otter/odom` 的原点相同）把目标经纬度换成 `otter/map` 坐标。本项目 map 与 odom 为恒等变换，船和障碍的相对位置由同一套换算保证。
3. 每个目标画成实心圆：半径为 `threatingRadius × obstacle_radius_scale`，缺失或非正时用 `obstacle_default_radius`。
4. 生成覆盖所有圆的栅格，整张替换上一张；`count: 0` 就是清空。以最新 0411 船位为中心、边长 `obstacle_grid_max_size` 的窗口外的目标不画，并记警告。
5. 栅格以 transient_local 发布。costmap 的 `ladds_obstacle_layer`（Nav2 StaticLayer）排在激光障碍层之前叠加，膨胀层再外扩 3 m；决策发现当前轨迹被挡会作废并重新规划，航控每拍检查当前船体占用。

`usv_ladds_rx` 的相关参数（`route.yaml`）：

| 参数 | 默认 | 说明 |
|---|---|---|
| `obstacle_grid_topic` | `/otter/ladds/obstacle_grid` | 为空则不画栅格，只转发 `/ladds/targets` |
| `obstacle_grid_frame` | `otter/map` | 栅格坐标系，与航路适配器的 `map_frame` 相同 |
| `obstacle_grid_resolution` | `1.0` | 米/格 |
| `obstacle_grid_max_size` | `1500.0` | 以船为中心的窗口边长（米） |
| `obstacle_default_radius` | `5.0` | 目标没有有效 `threatingRadius` 时的圆半径（米） |
| `obstacle_radius_scale` | `1.0` | `threatingRadius` 换算成米的系数；对端按 0.1 米整数发送时改为 `0.1` |
| `obstacle_timeout_sec` | `0.0` | 超过该秒数没有新 0141 就清空；0 表示保留到下一包 |

浮点参数在 YAML 里要写成带小数点的形式（如 `5.0`），否则 ROS 参数类型不匹配。

## 5. 用联调工具验证

1. 在工具里订阅 `0411`，读出船当前经纬度。
2. 在航线前方放一个目标：向北 30 m 纬度约加 `0.00027°`；向东 30 m 经度约加 `0.00031°`（按纬度 30.5° 估算）。
3. 在主题 `0141` 发一包，带不带外壳都可以，经纬度换成上一步算出的值：

   ```json
   {"head": {"packageSeq": 1, "packageType": 0, "sendPlatform": 1000, "recvPlatform": 1001},
    "content": {"count": 1, "targets": [{"batch": 1, "longitude": 114.3931, "latitude": 30.5453, "threatingRadius": 5.0}]}}
   ```

4. 观察：

   ```bash
   ros2 topic echo /ladds/targets --once
   ros2 topic echo /otter/ladds/obstacle_grid --once --field info
   ros2 topic echo /otter/navigation_status
   ```

   RViz 中添加 Map 显示 `/otter/ladds/obstacle_grid` 和 `/otter/costmap`，Durability 选 Transient Local。
   目标挡住当前轨迹时，`navigation_status` 会先出现 `Latest map invalidated reference`，随后重新规划绕行。
5. 发 `{"count":0,"targets":[]}` 清除全部障碍。

## 6. 上行自检（默认关闭）

仿真激光 → 0141 → 联调工具的链路仍在代码里。需要时在 `route.yaml` 中：

- 把 `usv_ladds_radar` 的 `output_topic` 改回 `/ladds/obstacle_in`；
- 需要在 ROS 侧看到解回的结果时，把 `usv_ladds_rx` 的 `obstacle_json_topic` 改回 `/ladds/obstacle`。

打开后，本系统发出的激光目标会和对端雷达的 0141 出现在同一主题上；开了回灌时，它们也会画进障碍栅格。

雷达节点默认参数：

```text
scan_topic=/otter/sensors/lidars/lidar/scan
odom_topic=/otter/odom_gt
output_topic=/ladds/lidar_targets（route.yaml；bridge.yaml 为 /ladds/obstacle_in）
```

一帧扫描里聚出障碍时，上行报文核心结构如下（坐标随本船位姿变化）：

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

`count` 等于 `targets` 长度，最多 255；超过时保留距离最近的目标。无回波时发送 `{"count":0,"targets":[]}`。
这是激光雷达观测目标，不是 AIS 船舶跟踪；`absSpeed` 固定为 0，`heading` 为 -1。
也可以用仓库中的 [0141 样例报文](examples/ladds-0141-obstacle.json) 向 `/ladds/obstacle_in` 手发一包检查发送链路。
发给 LaDDS 的字节会再套 `head`（`packageSeq`、`packageType`、`recvPlatform`、`sendPlatform`、`time`）与 `content`；
桥端发送调用返回成功只说明 SDK 接收了发送请求，不代表对端业务已经解析。

## 常见问题

- 地图上没有障碍：先 `ros2 topic echo /ladds/rx_info` 看有没有 `"topic":"0141"` 的包进来；tcp 后端确认中继启动行有 `obstacle-rx=0141`，工具主题保留前导零。
- 接收节点警告 `drop invalid 0141 JSON`：`count` 必须等于 `targets` 个数；带外壳时 `head`、`content` 都必须是对象。
- 警告 `0141 targets not drawn`：目标经纬度缺失或无效，或离船超过 `obstacle_grid_max_size` 的一半。
- costmap 里没有这一层：感知启动时要带 `ladds_obstacles:=true`，日志应有 `0141 obstacle layer`。
- 障碍整体偏移：`usv_ladds_rx` 的原点要与对端换算经纬度时所用的 0411 一致。
- 中继能启动但上行发送失败：确认 ROS 桥和 Windows 中继来自同一次构建；旧中继没有 0141 注册，会拒绝或把请求发到默认主题。
