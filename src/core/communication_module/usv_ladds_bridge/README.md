# LaDDS 与 ROS 2 双向通信桥

0331 主动刹停及 0211/taskType=80 终点定点值守的新协议语义、启动参数和示例见
[停船与终点值守](README.stop-guard.md)。普通 taskType=16 仍按原航路完成流程回 0052；
taskType=80 到终点进入值守时也回一次 0052，回传 `taskType: 80`、`taskFinish: 2`，表示航路终点到达，不表示微控退出。
值守保持活动，周期状态不重复反馈；0331 主动刹停仍不回 0052。
`0212` 靠离泊本阶段按 **单点航路** 执行，到点停稳后回一次 `0052` 并带回任务类型：离泊 `taskType: 3`、`taskFinish: 3`，靠泊 `taskType: 4`、`taskFinish: 4`；
不做靠泊导轨、侧推或锁船。示例报文见
[`docs/examples/ladds-0212-single-waypoint.json`](../../../../docs/examples/ladds-0212-single-waypoint.json)。
航路联调时 Windows 中继需显式加上 `-BerthReceiveTopic '0212'`，只改 ROS YAML 不够。

## 0054 航控推力

航控在唯一推进输出口把归一化左右桨命令经 0924 推力分配器换算为桨角与转速，发布 `/otter/thrusters/vector_cmd`（`usv_navigation_interfaces/ThrusterCommand`）。`route.yaml` 的 `thrust_send_topic: "0054"` 使发布节点订阅它并发到 LaDDS `0054`，外层同样带 `head`，`content` 为暂定格式：

```json
{"mode": 0, "left_angle": 0.0, "left_rpm": 283.4, "right_angle": 0.0, "right_rpm": 301.2}
```

- 桨角单位弧度、0 指向船艏；转速单位 rpm，带符号，正值沿桨角方向推。`mode` 固定为 0（固定桨），全回转横移尚未启用。
- 航控每拍（10 Hz）都发，空闲时就是零推力；命令到达即转发，只保留最新一条，中继回执卡住后不会补发旧命令。
- 看门狗：超过 `thrust_timeout_sec`（0.5 s）收不到有效航控命令，或航控尚未启动时，每 0.1 s 发一次零推力，直到命令恢复。
- 执行端须自行实现 0.5 s 收不到 0054 即归零；桥、中继或网络中断时只有这一层能兜底。
- `start_ladds_relay.ps1` 默认 `-ThrustSendTopic '0054'`，启动行末尾列出已登记的发送主题；中继未登记 0054 时 ROS 端桥启动即报错退出。不发推力时把 `thrust_send_topic` 设为 `""`，或给中继传 `-ThrustSendTopic ''`（两端都关）。
- 同一 DDS 域中订阅 0054 的实船执行端也会收到仿真算出的推力，仿真联调时注意隔离。

## 当前推荐：0211 多航点航路

已支持按 `points` 原顺序执行 1–255 个必经点。中间点在实测进入 1.5 m 范围后连续切换，最终点停稳后发送一次 0052。每点 `speed` 是进入该点的航段速度（节），允许为转弯和变速提前减速。
航点进度变化时发送 `0053`（`travelId` / `taskIndex`，`taskIndex` 为已到达的航点数，0 表示尚未到达第一个点），语义见[多航点快速联调](../../../../docs/ladds-multi-waypoint-quickstart.md)第 5 节。

启动、构建和三点报文见 [多航点快速联调](../../../../docs/ladds-multi-waypoint-quickstart.md)。使用新的 `start_ladds_route_sim.bash` 和 `start_ladds_route.bash`，两端均加载多航点 overlay；新增 ROS 消息需要一起更新决策、控制和适配器。

下面保留通信桥及早期单点联调说明。

本船导航 `0411` 的接入见 [LaDDS 0411 本船导航](../../../../docs/ladds-0411-quickstart.md)。
ROS 图里节点：`/usv_ladds_rx` 接收，`/usv_ladds_tx` 发布，
`/usv_ladds_route` 航路适配，`/usv_ladds_radar` 把仿真激光雷达扫描编成 0141。
`rx`/`tx` 同进程、共用一个 LaDDS 句柄；`tx` 使用 `use_global_arguments=False`，
避免被同一进程的 `__node` 改名。

障碍物/目标运动 `0141` 的接入见 [LaDDS 0141 目标运动联调](../../../../docs/ladds-0141-quickstart.md)。
`usv_ladds_radar` 随 `bridge.launch.py` / `route.launch.py` 一起启动：订阅
`/otter/sensors/lidars/lidar/scan` 和本船里程计，聚类后写 `/ladds/obstacle_in`；
发布节点校验后发到主题 `0141`，接收节点再订 0141 解回 `/ladds/targets`。
规划仍使用 `costmap_raw`，0141 只上报雷达聚类目标。

单段航渡已增加独立入口：见 [0211 单段航渡联调](README.single-leg.md)，包含给定经纬度原点、4节速度、任务执行及0052完成反馈。

当前实测链路：**Windows LaDDS 1.0.3 → 本机 TCP → WSL Ubuntu 22.04 / ROS 2 Humble**，双向可用。

用户联调配置：DDS 数据域 **1**、主题字符串 **`0211`**、**`0212`**、**`0331`**、**`0051`**、**`0052`**、**`0053`**、**`0054`**、**`0411`** 和 **`0141`**（均保留前导零）、QoS **尽力传输 / best_effort**、IP 留空使用活动接口。报文为可按 UTF-8 解码的 JSON，现场样本含 `head` 与 `content`。桥接原样接收 0211/0212/0331；发出的 `0051`、`0052`、`0053`、`0054`、`0411`、`0141` 在 LaDDS 上带 `head`（`sendPlatform` 1000、`recvPlatform` 1001、`packageType` 0、递增 `packageSeq`、发出时刻 `time`）和 `content`。`0411` 由发布节点把 `nav_msgs/Odometry` 编成 JSON，由接收节点再解回 odom；`0141` 由障碍物 JSON 转发并解回 `/ladds/targets`。

感知、决策、航控的心跳从 `0051` 出去。`content.code` 分别为 `perception`、`decision`、`control`；`content.state` 为 0（未启动或未正常工作）或 1（已启动且正常工作）。发布节点订阅 `/otter/navigation/heartbeat`，约每秒每个模块一条，状态变化时立即再发。任务完成仍走 `0052`，报文是 `travelId` / `taskFinish`。

## 为什么有 Windows 中继

当前 Windows SDK 能使用原有授权创建通信句柄并收发。提供的麒麟 Linux SDK 在 Ubuntu 中直接加载报告 `undefined symbol: SSL_CTX_free`；在单独探针进程预加载现有 `libssl.so.3` 后能读取版本，但创建句柄仍报告授权失败。未修改或复制厂商授权。

因此当前默认 `backend=tcp`：Windows 使用有效 SDK，WSL 仅运行 ROS 适配。TCP 是本机部署连接，不是将 ROS 2 底层 RMW 换成 LaDDS。Linux 兼容 SDK/授权齐备后，可使用已保留的 `backend=native`。

## 1. 构建一次

在 WSL 的 Bash 中，项目根目录执行：

```bash
source /opt/ros/humble/setup.bash
colcon build --base-paths src/core/communication_module/usv_ladds_bridge \
  --packages-select usv_ladds_bridge --cmake-args -DBUILD_TESTING=ON
```

构建只需要 ROS 2、ament_cmake_python、标准消息和 Python 标准库，不链接厂商 SDK。厂商 DLL/许可证保留在原目录，不放入仓库。

## 2. Windows 启动 LaDDS 中继

在项目根目录的 PowerShell 中执行，终端保持运行：

```powershell
.\scripts\start_ladds_relay.ps1
```

脚本默认读取本机原有联调工具的 DLL，自动选择 Windows 的 WSL 虚拟网卡作为 TCP 监听地址。2026-09-15 实测地址是 `172.27.128.1:17880`，WSL 地址是 `172.27.138.43`；重启后地址可能改变。

若需指定配置：

```powershell
.\scripts\start_ladds_relay.ps1 -ListenAddress '172.27.128.1' -DomainId 1 -Topic '0211' -Qos best_effort
```

`-LibraryPath`、`-PythonExe` 可指定本机 SDK 和 Python。SDK 需要的工作目录由脚本切换为 DLL 所在目录。

中继只允许一个活动 TCP 客户端，绑定本机 WSL 网卡，不应暴露到公共网络；此初版没有远程认证或 TLS。DDS `--encryption` 与 TCP 加密无关。WSL 连接断开后，中继允许新的客户端连接；ROS 节点需要重新启动，不自动重连。

## 3. WSL 启动 ROS 桥

另开 WSL 终端，在项目根目录执行：

```bash
export ROS_DOMAIN_ID=196
export ROS_LOCALHOST_ONLY=1
bash scripts/start_ladds_bridge.bash
```

脚本默认用 WSL 默认网关作为 Windows 中继 IP，也可以显式指定：

```bash
bash scripts/start_ladds_bridge.bash 172.27.128.1
```

正常启动会显示 `LaDDS bridge started (rx; TCP relay ... version 1.0.3)`。若端口已被占用，先检查是否已有本桥运行，不要重复启动中继。

`ROS_DOMAIN_ID=196` 是本轮隔离测试域，**不要求与 LaDDS 域 1 相同**。需要与现有 ROS 节点同域时，可由你明确选择相同 ROS 域，但本桥仍仅使用 `/ladds/*` 测试话题。

本轮工具验证使用独立安装区 `/tmp/usv_ladds_20260915/install`，没有修改原仿真安装区。直接复用该安装区时，在运行脚本前添加：

```bash
export LADDS_INSTALL_BASE=/tmp/usv_ladds_20260915/install
```

该临时目录若被清理，重新按第 1 步构建即可。也可手动加载 overlay 后直接启动：

```bash
source /opt/ros/humble/setup.bash
source /tmp/usv_ladds_20260915/install/setup.bash
ros2 launch usv_ladds_bridge bridge.launch.py relay_host:=172.27.128.1
```

## 4. 观察收到的 JSON

第三个 WSL 终端：

```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=196
export ROS_LOCALHOST_ONLY=1
ros2 topic echo /ladds/rx
```

此时在联调工具里持续发送到域 1、主题 `0211`，应看到原样 JSON。ROS CLI 输出外层 `data:` 是 `std_msgs/msg/String` 的字段，里面才是原始 JSON。

| ROS 话题 | 类型 | 用途 |
| --- | --- | --- |
| `/ladds/tx` | `std_msgs/msg/String` | UTF-8 文本发到默认 LaDDS 发送主题（单腿配置为 `0052`） |
| `/ladds/progress_tx` | `std_msgs/msg/String` | 航路适配器给出的 0053 裸 JSON，发布节点补 `head` 后发到 `progress_send_topic`（默认 `0053`） |
| `/otter/thrusters/vector_cmd` | `usv_navigation_interfaces/msg/ThrusterCommand` | 航控桨角/转速命令，发布节点编码并补 `head` 后发到 `thrust_send_topic`（航路配置 `0054`） |
| `/ladds/rx` | `std_msgs/msg/String` | LaDDS `0211` 收到的有效 UTF-8 文本 |
| `/ladds/berth_rx` | `std_msgs/msg/String` | LaDDS `0212` 收到的有效 UTF-8 文本 |
| `/ladds/ownship` | `std_msgs/msg/String` | 发布节点编出的 0411 JSON；接收节点订阅后解 odom |
| `/otter/odom` | `nav_msgs/msg/Odometry` | 接收节点把 0411 解回的内部里程计 |
| `/ladds/obstacle` | `std_msgs/msg/String` | 发布节点转发的 0141 JSON；接收节点再订 |
| `/ladds/targets` | `std_msgs/msg/String` | 接收节点解回的 0141 目标列表 |
| `/otter/sensors/lidars/lidar/scan` | `sensor_msgs/msg/LaserScan` | 仿真激光雷达，0141 雷达适配器输入 |
| `/ladds/obstacle_in` | `std_msgs/msg/String` | 雷达适配器输出、发布节点发送到 0141 的 JSON |
| `/ladds/tx_bytes` | `std_msgs/msg/UInt8MultiArray` | 原始字节发送 |
| `/ladds/rx_bytes` | `std_msgs/msg/UInt8MultiArray` | 每包原始字节，包括 NUL 和非 UTF-8 |
| `/ladds/rx_info` | `std_msgs/msg/String` | JSON 格式的域、主题、src/dst 和长度 |

有效文本包会同时出现在 `/ladds/rx` 与 `/ladds/rx_bytes`，表示同一份数据的两种观察形式，不是 DDS 重复收包。非 UTF-8 只出现在字节和元信息话题，不做有损替换。

## 5. ROS 发给联调工具

工具订阅域 1、主题 `0211`、尽力传输。在具有上述 ROS 环境变量的终端执行：

首次启动时应先让双方完成 DDS 发现，并用持续的低频测试消息确认已能接收。注册主题成功不等于远端已匹配；本轮同时启动时的少量早期测试消息没有被新 DDS 进程收到，先启动独立 DDS 端、再发测试标记后往返通过。Best Effort 不会补发发现完成前丢失的首包。

```bash
ros2 topic pub --once /ladds/tx std_msgs/msg/String \
  "{data: '{\"bridge_test\":\"hello_from_ros2\"}'}"
```

在工具中检查 `bridge_test` 标记。这不是航行任务报文。桥不会自动发送或把 RX 转发回 TX；若收发使用同一 DDS 主题，自己发送的消息也可能被本地订阅者收到，不能仅用这种回环证明另一端已收到。

ROS 发布成功只代表提交给 ROS；TCP 的 `sent` 响应代表 SDK 的 `LaDDS_SendEx` 返回成功，不代表业务对端执行成功。应用级 ACK、任务重试和去重不在本阶段范围内。正式验收应像本轮一样用独立 DDS 订阅端核对唯一标记。

## 停止与排错

- 分别在 ROS 桥、中继所在终端按 Ctrl+C。只停止本桥，不执行 `killall` 等全局进程清理。
- `Connection refused`：Windows 中继未运行，或 IP/端口不匹配。
- `relay disconnected`：检查中继终端错误，修复后重启 ROS 桥。
- 如果工具暂停发送，`ros2 topic echo` 保持等待属于正常现象；不要用历史包中不变的时间戳判断链路是否断开。
- `LaDDS_Create returned 0`：核对厂商授权、平台、工作目录和绑定 IP；不要修改许可证内容。
- 收不到数据：先确认工具确实正在发送，域为 1、主题是字符串 `0211`、QoS 尽力传输。工具显示“添加订阅”本身不表示它在发布。
- TCP 模式的 DDS 域、主题和 QoS 以 **Windows 中继参数**为准；ROS 同名参数仅对 native 后端生效。
- 队列默认最多 100 包，满时丢弃新包并计数，适用于本轮低频通信检查；不是可靠任务队列，也不提供状态新鲜度保证。

## 代码和测试

- `ladds_client.py`：ctypes 参数、句柄和回调生命周期，按长度复制原始字节。
- `wire.py` / `relay.py` / `tcp_client.py`：本机 TCP 长度前缀报文、SDK 收发结果及连接管理。
- `bridge_node.py` / `config.py`：ROS 消息适配与参数校验。

从项目根目录执行单元测试：

```bash
python3 -m pytest -q src/core/communication_module/usv_ladds_bridge/test
```

单元测试使用 SDK 替身及真实本机 socket，不能替代厂商互通测试。真实抓包、ROS 接收以及独立 DDS 订阅端证据见项目 `docs/validation/ladds-communication-20260915/`。
