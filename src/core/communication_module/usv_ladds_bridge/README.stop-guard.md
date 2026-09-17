# 0331 主动刹停与 0211 航路终点值守

本页描述此次用户确认的业务约定，不宣称已完成实艇验证。历史的单段联调数据不能
用作这两项新功能的刹停距离、位置保持精度或海况适用性证据。

## 消息语义

| DDS 主题 | 业务内容 | 行为 |
| --- | --- | --- |
| `0211` | `taskType: 16`、1–255 个航点 | 正常航路；最终点到达且低速后回一次 `0052` |
| `0211` | `taskType: 80`、1–255 个航点 | 到最终点进入值守时回一次 `0052`，随后持续定点微控 |
| `0331` | `{}` 或 `{"head":{},"content":{}}` | 立即打断航路/值守，主动刹停；停稳后零输出，不保持原地 |

0331 不要求经纬度、艏向或时长。为兼容旧工具，旧 `longitude`、`latitude`、
`heading`、`duration` 字段允许出现，但全部忽略并记录警告，不会据此驶向某个位置。
空字节串、`null`、数组和带其他业务字段的报文无效；无效报文不打断正在执行的任务。

任务类型 80 的值守中心是 `points[-1]` 对应的固定地图坐标，不是到达瞬间的船位。
中间点仍正常连续航行；`speed` 单位仍是节，表示驶向该航点的航段速度，不表示
值守纠偏速度。定点微控持续运行，实际位置误差、艏向误差和抗流能力受控制参数、
定位质量及推进器布局限制，不能保证数学意义上的绝对不动。

类型 80 的 `0052` 表示**航路终点已到达并进入值守**，不表示定点微控任务退出。
反馈格式不变：`{"travelId": 53, "taskFinish": 1}`（travelId 使用本次任务的实际值）。
每次执行只发布一次到达反馈；本轮未新增上位机确认或断线补发机制。
反馈仍使用原业务格式 `{"travelId": 53, "taskFinish": 1}`，其中 `travelId` 回传本次
航线编号。同一次执行只反馈一次，后续周期性的 `GUARDING` 状态不重复发送；
只有再次收到合法 0211 开始新执行并再次到点，才产生该新执行的到达反馈。

### 控制初值与能力边界

- 0331 使用带符号速度阻尼主动反推，归一化左右桨限幅均为 `0.4`；反向航行时用正推制动，不积累速度积分、不返回停止前位置。
- 平面速度不超过 `0.05 m/s`、偏航角速度不超过 `0.02 rad/s`，且连续源观测满足 `1 s`，才确认 `STOPPED`。相邻源观测间隔超过 `0.5 s` 或时间倒退会重新计时，重复时间戳不累计。接收一次零速数据或收到服务接受回复均不能代替停稳确认。
- 定点控制初始位置容差 `0.5 m`、艏向容差 `5°`。这些是 `StationKeepingController.DEFAULTS` 的仿真初值，不是实船精度承诺，也不来自 `guardingRadius`。
- 当前双纵向桨模型没有侧推。较大横偏需要先转向回点，再对准指定艏向；在任意横流下同时维持位置和艏向绝对不变并不具有硬件保证。
- 制动和微控均检查状态/地图/命令新鲜度及完整船体预测扫掠。缺数据或所有候选不安全时会停推并报告等待/受阻，不能声称任何工况都能完成主动刹停。软件命令不替代独立硬急停。
- 控制或感知模块在刹停期间重启，会保留停止意图，以新的内部任务编号隔离旧报告，等新实例的数据有效后继续制动。普通航渡和值守仍沿用重启后需重发任务的策略。

`guardingHeading` 按用户确认的 JSON **真北顺时针角度（度）**解释，允许 `[0, 360]`，
转换为地图 ROS yaw（弧度）：北 `0° → π/2`，东 `90° → 0`。
`null` 或缺失表示不约束固定艏向，`0` 是明确朝北，不能当作空值。
不约束固定艏向不等于控制器绝不转向：纠正位置偏差仍可能需要转向。
这与历史 0331 二进制艏向表中的 0.1° 单位不同，不能直接混用。

区域此次不适配：仅类型 80 允许携带 `areas`/`areasCount`，存在非空区域时输出
“未应用区域约束”警告，且每条 `/ladds/task_status` 中保留 `warnings`。
这些区域不自动成为禁航区、允许区或值守区。类型 16 仍保持原有拒绝区域的行为。
非空 `guardingRadius` 也只给未适配警告，不作为允许漂移半径或定点精度。

## 工具发送示例

在 DDS 字符串主题 `0211`（保留前导零）发送一次：

```json
{
  "head": {"sendPlatform": 1000, "recvPlatform": 1001, "packageSeq": 5},
  "content": {
    "travelId": 53,
    "taskType": 80,
    "count": 2,
    "points": [
      {"latitude": 30.546127, "longitude": 114.3938, "speed": 2, "type": 1},
      {"latitude": 30.546227, "longitude": 114.3938, "speed": 2, "type": 1}
    ],
    "guardingHeading": 90,
    "guardingRadius": null,
    "areasCount": 0,
    "areas": []
  }
}
```

上例到第二点后持续守点并朝东。只守点、不固定艏向时，将 `guardingHeading` 改为
`null`。恢复普通航路时改为类型 16，并将值守参数置空。每次收到合法 0211 都是
新执行；不要使用工具的定时循环发送。

需要主动刹停时，在另一个 **DDS 主题 `0331`** 发送：

```json
{}
```

不要把这个空对象发到 0211，也不要依赖消息体猜协议号。0331 停船过程中再次收到
0331 会合并到同一停止执行，并清除尚未提交的新航路；停止结束后新一次 0331 才
创建新的停止执行。旧航路不会自动恢复。

## 启动与观察

新增 `StopNavigation.srv` 以及 `NavigateRoute` 的值守字段后，必须同时重编译并加载
同一 overlay 的 `usv_navigation_interfaces`、`usv_decision`、`usv_control` 和
`usv_ladds_bridge`，不能混用旧生成接口。控制与决策的依赖包也需按主项目构建说明更新。

Windows 中继需要额外订阅 0331，原通用脚本默认仍保持单主题兼容：

```powershell
.\scripts\start_ladds_relay.ps1 -DomainId 1 -ReceiveTopic '0211' -StopReceiveTopic '0331' -SendTopic '0052'
```

ROS 使用包内 `config/single_leg.yaml`（已经配置 `stop_receive_topic: "0331"`）。
TCP 模式只改 ROS YAML 不会改变 Windows 已经启动的 DDS 订阅；需要两侧都启用。
Linux native 后端则直接按此 YAML 在同一 SDK 句柄注册两个订阅。

桥按 SDK/TCP 元数据中的原始 topic 分流：0211 文本只发布到 `/ladds/rx`，0331 文本
只发布到 `/ladds/stop_rx`。原始字节和 topic 元数据仍可分别在 `/ladds/rx_bytes`、
`/ladds/rx_info` 观察；不复制停船报文到普通航路入口。

```bash
# source 本轮重新构建的 overlay，并沿用仿真的 ROS_DOMAIN_ID / ROS_LOCALHOST_ONLY
ros2 topic echo /ladds/task_status std_msgs/msg/String
ros2 topic echo /otter/navigation_status std_msgs/msg/String
```

需要绕过 DDS 做已隔离仿真的本地协议测试时，可以使用：

```bash
ros2 topic pub --once /ladds/stop_rx std_msgs/msg/String "{data: '{}'}"
```

该命令会真实触发当前 ROS 域内的停止控制；仅对明确隔离的仿真使用，不要向实艇
所在 ROS 域随意发布。本文命令未在真实 DDS 或实艇链路执行。

## 状态与竞态约定

- `STOP_REQUESTED`：已经提交停止服务；不代表已停止。
- `STOPPING`：决策已接受主动刹停；仍可能有船速。
- `STOPPED`：收到匹配停止 request_id、任务编号和决策会话的低速确认。
- `GUARDING`：持续值守中；本次执行首次确认到点进入此状态时发送一次 0052，后续周期状态不重复发送，微控任务继续保持活动。
- `WAITING_STOP` / `STOP_UNKNOWN`：服务不可用、回包不明或尚未停稳时，后续航路留在等待状态。

收到合法 0331 后先撤销等待航路、中止适配器里的旧执行，再立即调用
`/otter/navigation/stop`，不等待旧 NavigateRoute 回包，也不等待新的定位/状态。
停止请求携带旧执行 ID 列表，决策封禁这些 ID，防止迟到的航路请求重新启动。
内部超时重试沿用同一个停止 ID，最多重试三次；新 0211 必须等待匹配 STOPPED 和
此前全部停止/取消回包处理结束后才允许提交。

旧 `cancel_navigation` 是没有任务 ID 的 Trigger 接口。本适配器会等待它自己发出的
旧取消回包，但不提供对任意外部客户端延迟取消的全局隔离；其他客户端不应同时争用任务控制权。

`/otter/cancel_navigation` 是既有普通取消接口，不能把它当作 0331 反推刹停接口。
0331 的服务接受和停稳确认、取消任务均不产生 0052。类型 80 只有实际到达终点并
进入值守时反馈一次 0052；不能把服务接受或规划成功当作到达，也不能用该反馈退出微控。
DDS best_effort 和软件通信链路不构成独立硬件急停保证，操作端必须观察实际接收和
停船状态；定位、速度反馈失效以及执行机构能力需要在隔离仿真和实船安全流程中验证。

## 本地回归

在 WSL / Ubuntu 22.04 中从仓库根目录运行完整隔离构建和测试：

```bash
bash scripts/validation/run_stop_guard_checks.bash all
```

脚本把依赖包复制到新的 ASCII `/tmp/usv-stop-guard.*` 目录，避免 Humble rosidl 的
中文源码路径问题，再构建独立安装区；主测试使用 `ROS_DOMAIN_ID=86`，末尾的航路适配器集成测试
单独使用域 `85` 和唯一话题名，两者均限制本机发现并使用人工输入。
不会覆盖现有导航安装区、启动 Gazebo 或连接真实 LaDDS。结果路径会输出为 `VERIFY_ROOT`，
JUnit 记录在该目录中。测试完成的安装区是源码快照，后续源码修改需要重新构建。

仅运行通信测试：

```bash
PYTHONPATH=src/core/shared/common python3 -B -m pytest -q -p no:cacheprovider \
  src/core/communication_module/usv_ladds_bridge/test
```

纯测试使用 SDK 替身和 loopback TCP，不接真实 DDS；节点竞态测试依赖已安装的 ROS
及本轮新接口，缺少 `rclpy` 时该文件会跳过。只有纯测试通过不能替代 ROS 联调、
闭环仿真及实艇测试。
