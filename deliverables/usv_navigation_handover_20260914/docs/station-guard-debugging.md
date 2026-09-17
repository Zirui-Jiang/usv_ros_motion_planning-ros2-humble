# 定点值守绕圆避障调试记录

> **迁移前历史调试记录。** 以下现象、代码位置、原始日志与数值对照属于原 `sim_env` 集成阶段，按各自记录版本解释；保留全部失败及修复证据，不改写历史数据。当前任务协调已迁入 `core/usv_mission`，迁移范围和迁移后验证见 [任务模块迁移记录](station-guard-migration.md)，当前操作见 [使用指南](station-guard-guide.md)。

本记录覆盖 2026-09-12 的 `orbit_obstacle-smoke-002` 与 `safety-smoke-003`。目的在于保存失败证据、区分实测与离线对照，并说明后续重测依据。任务规划速度保持原请求值。第 1–5 节记录增加制动后转艏退路之前的诊断；更新后的实现及其边界见第 6 节。旧版本的纯合成结果可能随实现更新而改变，不能以当前安装重放输出冒充旧版本结果。

## 1. 实测失败与证据范围

隔离试验使用 ROS domain `205`。WSL 工作区与原始记录位置如下：

- 工作区：`/home/usv/usv_migration/station-guard-001`
- 原始事件：`/home/usv/usv_migration/station-guard-001/results/orbit_obstacle-smoke-002/orbit_obstacle_01/events.jsonl`
- 结果：同目录 `metrics.json`
- 初始预检地图：同目录 `costmap_input_8359caf1.npz`，此时尚未生成受控障碍。
- 后期独立诊断快照：同目录 `planner-diagnostic-1ad0170e/`。

绕圆请求为圆心 `(23, -10)` m、半径 `8` m、顺时针、航速 `0.5` m/s、位置精度 `0.5` m、`1` 圈。受控圆柱半径 `0.5` m，中心为 `(23.5233542525, -17.9828629154)` m。

| 事件或样本 | 仿真时间 / s | 实测或日志含义 |
| --- | ---: | --- |
| 圆柱生成服务成功 | 58.2 | 生成前检查的完整船体间隙为 `1.4304200915` m；该检查使用生成操作开始时的最新艇位。 |
| 生成后的 odom 样本 | 58.2 | 艇位 `(26.380992, -17.109961, -2.818285)`；记录的扫掠间隙为 `1.386908` m。 |
| 首次记录到 `AVOIDANCE_BLOCKED` 的 odom 样本 | 59.6 | 速度约 `0.3628` m/s，扫掠间隙约 `0.7808` m。状态标签来自最近收到的状态消息，不等同于控制器相位的精确切换时刻。 |
| 首次记录到 `AVOIDING` 的 odom 样本 | 81.2 | 艇位 `(25.218242, -17.588302, -2.913516)`，速度约 `0.00645` m/s，扫掠间隙约 `0.13929` m。此时已滑入近障区域。 |
| 独立诊断地图与状态 | 253.8 | 地图仍含受控圆柱对应的 lethal 单元；详见下文。 |
| 删除圆柱 | 261.5 | 删除原因为 `cleanup_after_failed_recovery`，属于失败清理。 |

试验最终为 **failed**：未在规定时间内观察到保留障碍条件下的实测绕圆回归。失败状态中的规划原因为 `no candidate passed global route and LOS swept hull validation`。不能将此试验解释为成功绕障。

原始日志中早期相位已出现规划阻塞，而艇体继续靠近障碍，支持优先验证“无可用退出参考时进行经过安全检查的主动制动”。本记录不将后续主动制动改动视为已经通过闭环验证；需要用下一轮 smoke 数据确认。

## 2. 后期真实地图的来源

独立诊断只订阅 odom、栅格和状态，没有发布推进指令，也没有调用模型位置、生成或删除服务。快照保存为：

- `planner-diagnostic-1ad0170e/costmap_input_979c3ff6.npz`
- `planner-diagnostic-1ad0170e/state_input_979c3ff6.json`
- `planner-diagnostic-1ad0170e/navigation_context_979c3ff6.json`
- `planner-diagnostic-1ad0170e/events.jsonl`

地图尺寸为 `800 × 800`，分辨率 `0.5` m，原点 `(-174.5, -217.0)` m，frame 为 `otter/map`，时间戳 `253.8` s。状态时间戳同为 `253.8` s，艇位为 `(25.134737, -17.602886, -2.913367)`，艇体速度为 `(0.002532, 0.000619, 0.0000626)`。

受控圆柱位置周围的 `5 × 5` 原始代价格如下，确认抓取发生在障碍仍存在时，而不是障碍移除后的地图：

```text
253 253 253 162  64
253 254 253 206  75
253 254 253 206  75
253 253 253 162  64
162 206 162  90  41
```

诊断上下文记录的地图接收年龄约 `0.040` s、odom 接收年龄约 `0.138` s。各订阅分别采集，**不是原子 worker 输入快照**；本次没有采到最初失败请求所使用的栅格。后期地图与历史艇位组合只能用于带明确来源标记的反事实重放。

## 3. 后期真实地图与历史姿态的离线对照

对照使用上节后期真实地图，以及原始事件中的历史 odom 姿态和速度。为满足纯任务状态机的时效检查，输入状态的时间戳在内存中设为地图时间戳；历史时间单独输出，未改写原始文件。参数采用 `resolve_parameters({})` 的源码默认值，因此也不能冒充当时 worker 的完整参数快照。

两组对照分别把诊断任务航速设为 `0.5` 和 `0.2` m/s。这样 LOS 参考速度以及候选切入所用的 `speed / yaw_rate_max` 半径均一致变化；原始实测任务和生产代码均未改变。

| 历史 odom 时间 / s | 后期图上候选数 | 0.5 m/s | 0.2 m/s |
| ---: | ---: | --- | --- |
| 58.2 | 2 | SUCCESS | SUCCESS |
| 58.5 | 2 | SUCCESS | SUCCESS |
| 59.0 | 2 | SUCCESS | SUCCESS |
| 59.6 | 3 | SUCCESS | SUCCESS |
| 81.2 | 3 | NO_PATH | NO_PATH |

`81.2` s 姿态的三个候选均进入普通 LOS 验证，并被 `LOS swept footprint intersects lethal or unknown map cells` 拒绝。此时船体已经过于靠近障碍；单独把参考航速降至 `0.2` m/s没有使这些前进候选可行。

这些结果**不支持直接认定“首次失败必然由 0.5 m/s 转弯半径过大导致”**：初期历史姿态在后期地图上，两种速度均可规划成功。实际初始地图、候选集合、异步参考采用时序及控制状态仍需新试验的同步证据。

以下命令在 WSL 中只读重放现有证据，标准输出中明确保留数据来源限制。应先核对工作区源码与安装指纹；如果已刷新安装，其结果属于当前实现对旧证据的重放，不是旧版本原样复现。

```bash
source /home/usv/usv_migration/station-guard-001/install/setup.bash
python3 - <<'PY'
import json
from pathlib import Path
import numpy as np
from common.models import GridSnapshot, VesselState
from sim_env.configuration import resolve_parameters
from sim_env.missions import MissionManager, MissionSpec
from sim_env.runtime import PlanningRequest
import sim_env.runtime as runtime
from sim_env.missions.planning import plan_mission

trial = Path('/home/usv/usv_migration/station-guard-001/results/orbit_obstacle-smoke-002/orbit_obstacle_01')
raw = np.load(trial / 'planner-diagnostic-1ad0170e/costmap_input_979c3ff6.npz')
grid = GridSnapshot(raw['data'], float(raw['resolution']), float(raw['origin_x']),
                    float(raw['origin_y']), float(raw['stamp']))
events = [json.loads(line) for line in (trial / 'events.jsonl').read_text().splitlines()]
samples = {row['sim_time_s']: row for row in events if row['event'] == 'mission_sample'}
config = resolve_parameters({})
ordinary = runtime.plan_navigation
traces = []

def traced(request):
    response = ordinary(request)
    traces.append({'status': response.status.value, 'message': response.message,
                   'goal': request.goal})
    return response

runtime.plan_navigation = traced
print('Counterfactual replay: historical odometry on later actual obstacle map; not an atomic worker snapshot.')
for historical_time in (58.2, 58.5, 59.0, 59.6, 81.2):
    row = samples[historical_time]
    state = VesselState(*row['pose'], *row['body_velocity'], stamp=grid.stamp)
    for speed in (0.5, 0.2):
        spec = MissionSpec('orbit', 0., 0., speed=speed, orbit_radius=8., watch_radius=4., laps=1)
        manager = MissionManager(spec, (23., -10., 0.), grid.stamp, config=config)
        manager.request_escape('offline counterfactual diagnostic')
        intent = manager.tick(state, grid, grid.stamp)
        traces.clear()
        response = plan_mission(PlanningRequest(1, None, state, grid, np.empty((0, 3)), config, intent))
        print(json.dumps({'historical_time': historical_time, 'map_stamp': grid.stamp,
                          'speed': speed, 'candidate_count': len(intent.candidates),
                          'result': response.status.value, 'selected_goal': response.selected_goal,
                          'metadata': response.metadata, 'ordinary_trials': traces}))
PY
```

## 4. 纯合成几何对照

另做一个纯数值构造场景，用于检查低速是否可能改善某些近障前进转弯。该场景**不是实测地图，也不是上述实测失败的复现**：

- 空白已知栅格，`240 × 240`、分辨率 `0.25` m、原点 `(-30, -30)` m。
- 艇位 `(0, 0, 0)`，初始速度为零。
- 原圆圆心 `(0, 8)` m、半径 `8` m、逆时针。
- 合成圆柱中心 `(2.3, 0)` m、半径 `0.5` m；按栅格中心是否落入圆内标记 lethal，未仿真 lidar 与 Nav2 膨胀过程。
- 其余为默认规划参数，保留完整船体扫掠校验。

对照结果为 `0.5 m/s → NO_PATH`，`0.2 m/s → SUCCESS`，每次最多验证四个退出候选。它说明在特定几何、静止起步条件下，低速参考可以改善 LOS 转弯可行性；**不能推出本次初始实测失败的根因**，也不能证明实测艇速仍为约 `0.43` m/s时可以立刻安全执行低速参考。

可在已加载当前 ROS Python 安装的环境中重放：

```bash
python3 - <<'PY'
import numpy as np
from common.models import GridSnapshot, VesselState
from sim_env.configuration import resolve_parameters
from sim_env.missions import MissionManager, MissionSpec
from sim_env.runtime import PlanningRequest, plan_navigation

config = resolve_parameters({})
data = np.zeros((240, 240), np.uint8)
axis = (np.arange(240) + 0.5) * 0.25 - 30.
x, y = np.meshgrid(axis, axis)
data[(x - 2.3)**2 + y**2 <= 0.5**2] = 254
grid = GridSnapshot(data, 0.25, -30., -30.)
state = VesselState()
print('Synthetic numerical fixture only; not measured Gazebo data.')
for speed in (0.5, 0.2):
    spec = MissionSpec('orbit', 0., 0., speed=speed, orbit_radius=8., clockwise=False)
    manager = MissionManager(spec, (0., 8., 0.), 0., config=config)
    manager.request_escape('synthetic numerical diagnostic')
    intent = manager.tick(state, grid, 0.)
    result = plan_navigation(PlanningRequest(1, None, state, grid, np.empty((0, 3)), config, intent))
    print(speed, result.status.value, result.metadata, result.selected_goal)
PY
```

## 5. 本轮冻结内容与下一轮证据

本轮冻结 `src/` 与验收脚本后，优先重测已有明确依据的主动制动／安全接管修复；未实施低速退出修改，也未放宽完整船体碰撞条件。

验收脚本另补充了两项证据与时序处理：

1. 地图明确观察到受控障碍后，调用 `capture_inputs('controlled_obstacle_snapshot')`；任务失败时调用 `capture_inputs('mission_failure_snapshot')`。它们保留独立地图、状态及上下文时间戳，仍不宣称是原子 worker 快照。
2. 速度初值扰动记录请求与应答仿真时间，观测区间为请求时刻至应答后 `0.5` s。服务延迟单独限制为 `2` 个仿真秒，超限明确记为 blocked；请求之前的速度样本不参与识别。验收单元回归结果为 **30 passed**，包括延迟有效、未观察到脉冲及延迟超限反例。

下一轮若仍失败，应首先使用新增的初始障碍快照、同时段 odom、worker 请求与返回元信息以及控制安全门状态，定位失败发生在候选净空、A* 路径、LOS 扫掠、结果采用或控制执行哪一层。保留实际地图与合成对照的明确边界，不根据单个后期姿态推断初始规划原因。

## 6. smoke-003 初始实测原图与制动后转艏边界

第三次试验保存了首次观察到障碍后的真实栅格，目录为：

```text
/home/usv/usv_migration/station-guard-001/results/safety-smoke-003/orbit_obstacle_01/orbit_obstacle_01/
```

关键输入为 `costmap_input_883edcae.npz`、`state_input_883edcae.json`、`navigation_context_883edcae.json`。栅格与艇体状态的记录时间均为 `57.2` s，分辨率为 `0.5` m，栅格原点为 `(-173.5, -217.0)` m。状态为：

```text
pose = (26.0393734918, -17.2608382243, -2.8143967717)
(u, v, r) = (0.4861631622, 0.0346587458, -0.0835119774)
```

受控圆柱中心为 `(23.6052469360, -17.9770719030)` m、半径 `0.5` m。上下文记录的地图与 odom 接收年龄分别约为 `0.020` 和 `0.101` s。虽然二者消息时间相同，这仍是分别订阅的最新数据，不能宣称是运行中 worker 的原子请求快照；重放使用源码默认参数。

原始前进规划在 `0.5` 和 `0.2` m/s 下均失败。两个回圆候选的 A*、中心线路径及全船体全局路线检查通过，但两条 LOS 轨迹均被扫掠船体校验拒绝。`0.5` m/s 的第一段不安全轨迹位于预测 `0.5–1.0` s，`0.2` m/s 则位于 `1.5–2.0` s。降速延后预测碰撞，却未提供可行退出路径。详细记录为 `initial_obstacle_planning_diagnostic.json`。

在原始艇位直接把速度置零，再尝试转艏，部分方向的 LOS 确实可行；但实际制动会向障碍移动。该静止反事实记录在 `initial_obstacle_alignment_diagnostic.json`，不能用作可执行的安全证明。

新增 worker 退路先从实测六维状态预测制动，再在预测停车点检查带 `0.1` m 裕量的完整船体旋转，最后从对齐后的零速度状态验证 LOS。正常制动及两个增强首步命令的停车结果如下；增强首步后的尾迹仍采用普通制动模型，未把更强制动假定为持续控制。

| 首步制动候选 | 停车位置 / m | 停车艏向 / rad | 带裕量制动尾迹 | 8 个 45° 方向的带裕量旋转 | 可旋转方向的 0.5 m/s LOS |
| --- | --- | ---: | --- | --- | --- |
| normal | `(25.751406, -17.409745)` | `-2.844171` | 通过 | 全部拒绝 | 未进入 |
| gain 60 / force 40 | `(25.779688, -17.398788)` | `-2.844157` | 通过 | 仅 `180°` 通过 | 两个回圆候选均拒绝 |
| gain 120 / force 40 | `(25.796048, -17.393147)` | `-2.844300` | 通过 | 仅 `180°` 通过 | 两个回圆候选均拒绝 |

表中额外两个增强候选通过内存中逐个强制候选的诊断调用检验，并非声称生产 worker 会遍历所有已通过停车检查的候选。实际 worker 按顺序采用第一个通过制动尾迹检查的停车预测，再尝试转艏方向。

`initial_braking_alignment_diagnostic.json` 保存逐候选结果。更新后的 worker 对 `57.2` s 原始输入和 `57.3` s 历史 odom 配合此图的重放仍为 `NO_PATH`，耗时约 `0.04–0.05` s，未耗尽 `1` s 预算；记录为 `initial_alignment_worker_replay.json`。这是停车后缺乏安全转艏与前进空间的边界，不能记录成原图修复成功，也不能通过放宽船体裕量强行通过。

新增退路的纯数值回归验证了：存在安全制动、旋转与 LOS 组合时返回 `departure_pose`；动态停车尾迹不安全时不返回条件参考；已经对齐时直接使用正常 LOS；测量完成后按原目标重新规划。返回的轨迹只表示预测停车状态下存在继续航行的几何条件。执行侧必须先制动到实测低速，冻结实测停车点，再完成转艏并重新规划，不能直接执行预测停车点的旧路线。

后续闭环试验需要证明主动制动足够及时地保留转艏空间，再验证回到原绕圆轨迹。进入 `ALIGN_ESCAPE` 是中间状态，不能单独作为绕障恢复或完成一圈的验收依据。

## 7. matrix001 第三次绕障的解除区包络缺陷

正式矩阵 `mission-matrix-001` 的三个绕障试次均观察到了受控障碍后的恢复，但第三次 `orbit_obstacle_03` 随后未完成返回中心，在临时点持续进入 `WAITING_CLEARANCE`。该矩阵已中断，**不能标记为全项通过，也不能把此前的绕障恢复等同于完整任务完成**。

原始证据目录为：

```text
/home/usv/usv_migration/station-guard-001/results/mission-matrix-001/orbit_obstacle_03/orbit_obstacle_01/
```

`events.jsonl` 中的临时目标为 `(17.2642284377, -4.1072351888, 1.1123192732)`。例如 `387.9` s 实测状态为 `(17.0172862790, -4.4711287262, 1.0756853418)`，速度约 `0.000645` m/s，距离原目标中心约 `8.1462` m。该样本已经低速到达临时目标的位置容差内，但仍处于 `WAITING_CLEARANCE`。

只读核验使用两张此前已保存的真实地图，没有更改运行中的模型、地图或消息：

| 地图文件 | 地图时间 / s | 旧候选方形检查 | 临时目标处 `pose_is_clear(margin=5)` | 修正后的任意艏向包络 |
| --- | ---: | --- | --- | --- |
| `costmap_input_da39aa08.npz` | 2.7 | 通过 | 不通过 | 不通过 |
| `costmap_input_99f0e407.npz` | 61.2 | 通过 | 不通过 | 不通过 |

前者为预检地图，后者为受控障碍快照。它们是历史地图上的离线几何核验，不是 `387.9` s 的实时地图或原子 worker 快照。两张图均足以独立复现同一静态几何缺陷：旧候选检查通过，而候选自身的解除区检查已失败。

静态 `BOX_0` 在世界配置中的平面范围为 `x∈[0,10]`、`y∈[-12,8]`，保存地图中其附近包含中心坐标 `x=9.75` 或 `10.25` 的 lethal 单元。临时目标处旧方形的西边界为 `x=11.011232`，避开了这些单元；但按目标艏向旋转后的 5 m 解除矩形西边界为 `x=9.542798`，已经与该障碍相交。因此该问题是候选筛选与解除条件的数学定义不一致，不是缺少等待时间。

令艇长、艇宽分别为 `L,W`，解除裕量为 `m`。现有 `pose_is_clear` 将长宽扩展为 `L+2m,W+2m`，再随艇体艏向旋转。旧候选方形却使用半边长

\[
r_{\mathrm{old}}=\sqrt{(L/2)^2+(W/2)^2}+m,
\]

不能覆盖该扩展矩形的所有旋转。保持现有解除裕量语义时，正确的任意艏向包络半径为

\[
r_{\mathrm{release}}=\sqrt{(L/2+m)^2+(W/2+m)^2}.
\]

此外，任务允许在目标位置精度 \(\varepsilon_p\) 范围内判定到达，实际艇位可能朝障碍偏移。候选方形最终采用半边长

\[
r_{\mathrm{candidate}}=r_{\mathrm{release}}+\varepsilon_p.
\]

默认 `L=2.2 m`、`W=1.2 m`、`m=5 m` 时，旧半边长为 `6.252996 m`，正确解除包络半径为 `8.280700 m`；再加入 `0.5 m` 到达精度后，候选半边长为 `8.780700 m`、边长为 `17.561401 m`。由此同时覆盖任意艏向和到达盘内的位置偏差，未缩小原有 5 m 解除区。

本地修复新增 `arrival_tolerance` 参数，由状态机的普通退出、绕圆前向回归候选以及 worker 的退出候选复验统一传入 `spec.position_tolerance`。新增的五个数值回归分别覆盖：旋转扩展矩形的角部遗漏、到达盘内偏移、普通退出候选、绕圆退出候选，以及 worker 对候选的再次安全检查。五个测试在旧实现上全部失败；修复后任务状态机与任务规划测试为 **66 passed**，扩展回归为 **280 passed、3 skipped**，命令为：

```text
python -m pytest src/sim_env/test src/core/common/test src/core/controller/controller/test src/core/trajectory_planner/test -q
```

上述测试证明数值层的几何契约及相关回归通过，不代表中断的 matrix001 已被追认为成功。修复后使用新构建启动 `mission-matrix-002`；该轮随后因条件轨迹接管复审发现而手动中断，保留0项passed、0项failed、3项blocked、18项not_run。对应源码版本为 `envelope-en3qIF`、653项源码测试通过，运行结果不改写为后续修复版本的结论。

## 8. 条件轨迹接管修复与 matrix003 结果

matrix002运行期间发现，重复携带相同 `departure_pose` metadata 的条件规划响应可能被ROS适配层直接接管轨迹，违背条件路径仅作可行性证明的合同。修复后的适配层无条件禁止直接adopt带此metadata的条件trajectory；重复条件响应使当前地图进入NO_PATH并执行受验证安全制动，新地图到达后重新选择安全目标。新增三项回归在旧实现均失败、修复后均通过，`validation/handoff-fNJc74` 构建、14包安装审计和完整656项源码测试通过。

在该冻结版本上运行的matrix003已自然结束：passed=2、failed=1、blocked=0、not_run=18。orbit_obstacle的trial01/03完整通过，trial02长期NO_SAFE_EXIT，在180s受控绕障恢复期限内未满足实测回圆驻留条件，后续18项未派发。两次通过不能覆盖该次失败，基础回归通过不等于完整闭环矩阵通过。

## 9. matrix003 trial02：直接LOS与原地转艏均不可行

使用sim221保存的现场输入及失败运行输入逐层核验，真实消息时间如下。状态和地图分别保存，时间不同，不称为同时刻原子worker快照。

| 状态时间 / s | 地图时间 / s | 半圆恢复合法候选 | 整圆恢复合法候选 | 普通退出合法候选 |
|---:|---:|---:|---:|---:|
| 221.4 | 220.5 | 1 | 7 | 1 |
| 243.8 | 243.0 | 1 | 7 | 1 |

两组输入中，全局折线的完整船体检查通过，但所有直接LOS均失败；停车点处以0.1m裕量检查原地转艏，完整船体扫掠也均失败。扩大恢复候选到整圆确实增加了合法目标，却没有产生可执行LOS或原地转艏退路，因此半圆候选遗漏不是此次不能恢复的主要原因。

在两组输入上，沿当前psi反方向后退0.5m后，再转艏并对原goal规划LOS均通过数值检查。后退距离检查集合为 `[0.5, 1, 2, 3] m`；0.5m是其中最短被测可行值，不是理论最短值。本证据只支持离线条件可行性，不证明实艇已后退到位，也不证明后退控制过程或最终回圆已在Gazebo新运行中通过。

已实现并冻结的执行前奏为 `BRAKING_ESCAPE → BACKING_ESCAPE → ALIGN_ESCAPE → AVOIDING`：

1. 先实测制动至平面速度≤0.04m/s且|r|≤0.02rad/s，在此时冻结实际XY和psi。
2. BACKING_ESCAPE以kind=hold持续固定冻结艏向，后退到经过规划检查的临时后退目标。
3. 后退目标位置误差≤min(0.05m, position_tolerance/10)、平面速度≤0.04m/s、|r|≤0.02rad/s、艏向误差≤5°，连续新odom证明稳定0.5s后，才在实测位置转入ALIGN_ESCAPE。
4. 实测对齐后对原goal重新规划，不能直接执行假设后退点生成的旧LOS轨迹；原任务及目标保留。

后退规划与manager合同已新增29项回归。独立WSL的 `validation/backoff-T510NA` 构建、14包安装审计通过，完整685项源码测试在11.09s内通过。新版对上述两张原图输入的数值重放均返回success，均选择backoff=0.5m且原目标不变，耗时分别为31.9ms、26.5ms；这属于原输入重放，不是新仿真，也没有新增独立闭环次数。

本次修复保持0.1m控制扫掠裕量、5m解除区裕量和180s受控绕障恢复期限不变。最终交叉接口只读审查后执行的matrix004已自然结束，2passed、1failed、18not_run；两个通过trial实际使用了BACKING_ESCAPE。它们提供后退前奏的闭环证据，但完整矩阵仍未通过，不覆盖matrix003的真实失败，也不把数值重放计作新仿真。

## 10. matrix004 trial02：后期候选遗漏与越障评分误拒

本轮同一失败trial包含两个独立问题，需要按时间和证据分别解释。

在圆柱尚未删除的state254.6s/map253.9s真实输入中，原15°–180°恢复范围没有合法候选。扩展为沿规定方向15°–345°后有11个合法点，保持既有距离排序和四候选worker预算，数值重放成功选中 `(30.763496, -8.069164, -1.327035)`，单位为m、m、rad，耗时51.2ms。候选修复新增四项回归在旧实现均失败、修复后均通过，其他裕量和预算不变。本次证实的后期半圆遗漏，与第9节matrix003两组输入中“整圆仍不能解决LOS/原地转艏不可行”是不同快照和不同约束，不能互相替代。

更早的绕障恢复评分存在独立误拒。A*离圆避障轨迹经过圆心附近，最小径向距离为0.01748m，所有离圆段累计净极角为负。圆心附近极角变化不能度量是否越过局部障碍，因此该净角不能作为越障判据。原始日志137.4–138.4s窗口已满足下游回圆条件，但旧评分拒绝恢复，没有删除圆柱，后续又触发上述候选遗漏。

recovery-iKvKeO冻结版本的新版harness保存“完整船体曾进入障碍切向下游半平面”事件。障碍相位处的顺行单位切向为t，艇中心对障碍的投影为q=(p-p_obstacle)·t，完整矩形船体在该方向的支撑半径为h；从上游初态观测到q-h>0后保留越障事件。对默认2.2×1.2m船体，h=1.1|cos(psi-psi_t)|+0.6|sin(psi-psi_t)|。不能只要求艇中心过线。

事件出现后，当前还需原圆精度带内、切向误差≤15°、速度≥0.1m/s、ORBITING连续驻留1s；从障碍生成开始的历史odom间隔≤0.5s，完整船体全程扫掠间隙≥0.2m。恢复期限180s和控制/解除安全裕量不变。旧离圆净极角仅作诊断，正常ORBITING精度带内的独立有效计圈完全保留原算法。

137.4–138.4s的partial rejoin rescore验证了局部回圆窗口与评分修正；旧trial后续有效圈数和返中心未完成，禁止把整trial rescore成pass。原failed判定、日志、指标和局部重评分别保留，该操作不构成新仿真或新增试验次数。

算法候选和评分器两项修复已定向同步至WSL并冻结。`validation/recovery-iKvKeO`构建、14包安装审计全部通过，705项源码测试在12.88s内通过；随后matrix005在三次反复绕障后人工中断，详见下一节。matrix004的原失败不升级。

## 11. matrix005：全圆距离混排导致上游恢复循环

matrix005三次已启动运行均观察到选中上游恢复点并反复绕障。向唯一自有矩阵父进程发送SIGINT后正常结束，摘要为0passed、0failed、3blocked、18not_run。实际循环日志保留，blocked表示主动中断类别，不解释为通过或三个自然超时失败。

三张初始地图各有7个通过几何检查的候选，原前半圆只有180°点合法。将全圆候选按欧氏距离混排后，worker前四个候选为270°/255°/240°/225°，实际选中270°点，其全艇下游投影约−8.4m。只传入原180°点时也返回plan SUCCESS，全艇下游投影范围为+1.36..+1.67m。前半圆可行目标存在，后半圆却因较短路径抢先入选，这是新增混排策略带来的闭环回归。

已实现并冻结的最小修复采用几何检查之后的分组回退：15°–180°前半圆集合非空时只提供该组候选，沿用原有距离排序；只有整组为空才启用195°–345°后半圆。前半圆判断上界为π+1e-9rad，保留浮点误差范围内的180°点。这样保留matrix004圆东侧没有前半圆合法点时的扩展恢复能力，同时阻止可行前半圆被后半圆短路替代。最多四候选、1s规划预算、0.1m控制裕量、5m解除区及180s恢复期限均不修改。

分组修复新增八项回归，在旧实现均失败、修复后均通过。三张005初始图均选180°下游点并成功规划，004圆东点的后半圆fallback仍成功；原图数值重放不计作新仿真。`validation/priority-IzMlsY`构建、14包安装审计通过，713项源码测试在13.15s内通过。705项与recovery-iKvKeO仍是matrix005对应的修复前版本。

matrix006在priority-IzMlsY源码／安装区冻结后完成全部21项，退出码0，21passed、0failed、0blocked、0not_run、running=0、all_passed=true，源码与安装前后一致。旧循环与中断记录不覆盖、不重新标为通过。

## 12. 最终闭环收口与证据边界

冻结版本最终通过713项源码测试及14包构建／安装审计，另保留按自身manifest确定版本的原导航六项历史闭环通过记录。matrix006七场景各三次全部通过，其九个阵位评分窗口均为600样本、联合达标率100%；三次orbit_obstacle的径向P95为0.141474–0.142828m、独立净圈数1.004023–1.008407、完整船体最小间隙0.465323–0.584453m，均完成原中心返回。trial01/03实际经历BACKING_ESCAPE，trial02没有调用该前奏但同样完成。该结果来自新版本完整真实仿真，不是旧失败重评分。

[归档摘要](validation/station-guard-001/archive-summary.json) 与 [简明指标](validation/station-guard-001/metrics-summary.json) 均记录source_unchanged=true、installation_unchanged=true。最终源码SHA256为 `31edc633a7461d36b7bba2af97a8d69cde6cef18a1eb4fc5fe3c90b71f8bb3aa`，安装区为 `7fe5b8257cb8b66f0efb78769d950714d670a8e781b638ea6c038a0c49846f65`。矩阵原摘要SHA256为 `3b549f2e454d63e71325aa73850bbc6de00d650d294aa0388922b56d66e354cc`。

上述结论限于冻结副本及 [已验证源码补丁](validation/station-guard-001/feature.patch)，不包含共享根目录的并行雷达改动，不延伸为真实海试或动态CPA/COLREGs性能。完整七场景表和图件链接见 [验证报告](station-guard-validation.md)；[交付入口](validation/station-guard-001/README.md) 保留matrix001–006、原导航、smokes和各次构建记录。smoke004的整轮原日志重评分与matrix004的137.4–138.4s局部回圆窗口重评仍各自保持原有范围，后者不能升级未完成圈数／返中心的整次trial。
