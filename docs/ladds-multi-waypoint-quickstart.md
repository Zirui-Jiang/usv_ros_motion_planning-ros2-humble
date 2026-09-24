# LaDDS 0211 多航点快速联调

## 结论

一条 `0211` 报文中的 `points` 是完整有序航路。系统先原子校验全部点，再按原顺序执行；每个中间点都必须由实际里程计轨迹进入半径 `1.5 m` 的圆，最终点还必须满足距目标不超过 `1.5 m`、平面速度不超过 `0.25 m/s`。整条航路完成后才发布一次 `0052`。

每个点的 `speed` 单位为节，表示“当前位置或前一点进入本点”的航段速度。规划器可为转弯和最终停车提前减速，因此该值不是航点处的瞬时保速保证。`4 kn = 2.0577777778 m/s`，多航点仿真 profile 的速度上限为 `2.1 m/s`。

## 1. 构建独立 overlay

在 WSL 的仓库根目录执行：

```bash
bash scripts/build_ladds_route.bash
```

只改了桥接时不要重编整条 overlay，只选这一个包：

```bash
bash scripts/build_ladds_route.bash --packages usv_ladds_bridge
```

默认从当前仓库的 `src/core` 构建以下 10 个包：`common`、`controller`、`path_planner`、`path_processor`、`trajectory_planner`、`usv_navigation_interfaces`、`usv_navigation`、`usv_decision`、`usv_control`、`usv_ladds_bridge`。基线为 `/home/usv/.local/share/usv_ros_motion_planning/base/install`，独立输出为：

```text
/tmp/usv_ladds_multi_waypoint_20260915/build
/tmp/usv_ladds_multi_waypoint_20260915/log
/tmp/usv_ladds_multi_waypoint_20260915/install
```

输出目录已存在时执行标准的 colcon 增量构建，不删除旧结果。可通过 `--source`、`--base-install` 和 `--output-root` 显式选择其它路径。

## 2. 启动 Windows LaDDS 中继

在 Windows PowerShell 中运行现有中继：

```powershell
.\scripts\start_ladds_relay.ps1 -DomainId 1 -ReceiveTopic '0211' -StopReceiveTopic '0331' -BerthReceiveTopic '0212' -SendTopic '0052' -StatusSendTopic '0051' -ProgressSendTopic '0053'
```

`-ProgressSendTopic` 默认即 `0053`；中继只在启动时注册发布主题，更新本脚本后须重启中继，0053 才能发出。记录脚本显示的 WSL 虚拟网卡 IPv4。LaDDS 使用 DDS 域 `1`；ROS 仿真、适配器和观察终端使用 ROS 域 `42`。两者是不同中间件的独立域号，无需相等。

## 3. 启动仿真和航路适配器

如果旧的仿真和适配器仍在运行，先分别回到原来的两个 WSL 终端按 `Ctrl+C`，等待它们正常退出，再启动下面的新入口。这样可避免 ROS 域 `42` 中出现同名节点，也可避免新消息接口与旧 overlay 混用。不要用新的终端批量结束进程。

仿真终端：

```bash
cd '/mnt/e/usv_ros_motion_planning-ros2-humble'
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=1
export GAZEBO_MASTER_URI=http://127.0.0.1:11347
bash scripts/start_ladds_route_sim.bash --headless false
```

该 wrapper 复用现有普通导航主 launch 和单航段仿真 profile，默认加载 `/home/usv/.local/share/usv_ros_motion_planning/base/install` 后再加载 `/tmp/usv_ladds_multi_waypoint_20260915/install`。它保留 `ocean` 的地图、障碍和艇初始位姿 `(23,-10)`，只在运行时副本中使用 WGS84/ENU 原点 `(30.5453,114.392931)`。

适配器终端使用同一个 ROS 域，并把地址替换为 Windows 中继实际显示值：

```bash
cd '/mnt/e/usv_ros_motion_planning-ros2-humble'
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=1
bash scripts/start_ladds_route.bash --relay-host 172.27.128.1
```

安装区内 launch 文件为 `route.launch.py`。`ros2 node list` 里是 `usv_ladds_rx`、`usv_ladds_tx`、`usv_ladds_route`、`usv_ladds_radar`。`usv_ladds_route` 会把完整 `points` 数组提交到 `NavigateRoute`，并非只取首点。`usv_ladds_radar` 随桥启动，把激光雷达扫描编成 0141。旧入口 `start_ladds_single_leg.bash` / `single_leg.launch.py` 仍转发到本脚本。

观察终端也必须显式使用相同 ROS 环境：

```bash
source /opt/ros/humble/setup.bash
source /home/usv/.local/share/usv_ros_motion_planning/base/install/setup.bash
source /tmp/usv_ladds_multi_waypoint_20260915/install/setup.bash
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=1
ros2 topic echo /ladds/task_status
```

另开一个同环境终端可观察 `/otter/navigation_status`。如果任一终端继承了 `ROS_LOCALHOST_ONLY=0`，wrapper 会尊重这个显式值；联调时应像上面一样统一显式设为 `1`。

## 4. 0211 报文

支持带 `head`/`content` 的完整报文，也支持直接发送 `content` 对象。核心约束如下：

- `travelId`、`taskType`、`count` 和每个 `type` 必须是 JSON 整数，不能写成浮点数或布尔值；`taskType` 必须为 `16`。
- `count` 范围为 `1..255`，必须与 `points` 数组长度相等。
- 每一点必须包含 `type: 1`、WGS84 `latitude`、`longitude` 和正数 `speed`。
- 当前仅支持无区域和无守卫约束：`areasCount` 省略或为 `0`，`areas` 省略、`null` 或空数组；`guardingHeading`、`guardingRadius` 省略或为 `null`。
- 任一后续点非法会拒绝整包，不执行前面的合法点。

下面三点从默认艇初始位置 `(23,-10)` 出发，已在默认 Gazebo 场景中完成整条航路实测。本轮验证使用现有 ADRC 配置，最终导航状态中的 `controller` 为 `adrc`。

| 序号 | ENU `(x,y)` m | 纬度 | 经度 | 入点航段速度 |
|---:|---:|---:|---:|---:|
| 1 | `(35,-10)` | `30.545209796997472` | `114.39329575621676` | `2 kn` |
| 2 | `(35,2)` | `30.54531803998755` | `114.39329575662137` | `4 kn` |
| 3 | `(47,2)` | `30.545318039577303` | `114.39342081603291` | `2 kn` |

示例报文：

```json
{
  "head": {
    "packageSeq": 1,
    "packageType": 0,
    "sendPlatform": 1000,
    "recvPlatform": 1001,
    "time": "2026-09-15 00:00:00.000"
  },
  "content": {
    "travelId": 60,
    "taskType": 16,
    "count": 3,
    "points": [
      {"type": 1, "latitude": 30.545209796997472, "longitude": 114.39329575621676, "speed": 2},
      {"type": 1, "latitude": 30.54531803998755, "longitude": 114.39329575662137, "speed": 4},
      {"type": 1, "latitude": 30.545318039577303, "longitude": 114.39342081603291, "speed": 2}
    ],
    "guardingRadius": null,
    "guardingHeading": null,
    "areasCount": 0,
    "areas": []
  }
}
```

同一报文也保存在 [`docs/examples/ladds-0211-three-waypoint.json`](examples/ladds-0211-three-waypoint.json)，可直接复制到发送工具。这是以默认初始位置为起点的实测路线；如果艇已经移动，应从实际位置重新检查和规划航路，不能假设它仍在 `(23,-10)`。

## 5. 执行与反馈语义

在工具中关闭定时或循环发送，每次执行只发送一包 `0211`。每次收到合法 `0211` 都代表一次新执行：即使 `travelId` 和全部内容相同，也会取消当前执行并从头重新执行该航路。`travelId` 是航线编号，可以复用；内部执行使用新的随机请求 ID。

中间点进入 1.5 m 圆后继续下一点，不要求低速停车。最终点同时满足位置和速度条件后，工具订阅的 `0052` 才收到。业务字段在 `content` 里，外层另有 `head`：

```json
{
  "head": {
    "packageSeq": 1,
    "packageType": 0,
    "recvPlatform": 1001,
    "sendPlatform": 1000,
    "time": "2026-09-01 09:48:43.950"
  },
  "content": {"travelId": 60, "taskFinish": 1}
}
```

被新 `0211` 替换、取消、模块重启、时钟重置、无路或超时均不会发送成功 `0052`。协议当前没有定义失败 DDS 报文，应从 `/ladds/task_status` 和 `/otter/navigation_status` 查看原因及 `waypoint_index`、`passed_waypoints`。

执行中另发 `0053` 任务执行情况，`content` 为 `{"travelId": 60, "taskIndex": N}`：`travelId` 取自本次 `0211`，`taskIndex` 取决策确认的 `waypoint_index`。
航路被接受后的首个导航状态发 `0`（尚未到达第一个点），每进入一个必经点圆后发一次递增值；最终点到达（类型 80 为进入值守）时发 `count`，并先于 `0052` 发出。
同一值不重复发送，导航状态约 1 Hz，故最多滞后约 1 s，过点过快时序号可能跳变。被替换、取消或 0331 刹停的执行不再发 0053；`0212` 靠离泊不发 0053。
ROS 侧可 `ros2 topic echo /ladds/progress_tx` 查看裸 JSON，LaDDS 工具订阅主题 `0053` 查看带 `head` 的报文。
