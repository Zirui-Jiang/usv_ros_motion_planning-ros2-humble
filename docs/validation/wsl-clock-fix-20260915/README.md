# WSL 时钟冲突与 Gazebo 周期性停顿诊断记录

2026-09-15。环境为 Windows 上的 WSL 2 / Ubuntu-22.04，运行 ROS 2 Humble、Gazebo Classic 和 Otter ADRC 控制链路。

**已确认并处理的原因是两套校时服务争用同一个 WSL 内核时钟。** 保留 WSL 根命名空间的 `chronyd`，执行 `systemctl disable --now systemd-timesyncd.service` 停用 Ubuntu 内重复运行的校时服务后，连续 170.04543 秒未检测到墙钟突跳，`CLOCK_MONOTONIC` 相对 Hyper-V PTP 的平均速率偏差由约 +7.79% 降至 −0.369872 ppm。时钟源仍为 `tsc`，本轮没有再次手动设置 tick。

**Gazebo 修复后 150 秒专项验收通过。** 五类观测消息均没有超过 0.5 秒的接收间隔，`/clock` 和里程计最大间隔约 0.130 秒，实时因子约 0.9953。本次覆盖了时钟层单变量验证和 headless Gazebo 消息持续性验证；不能将其扩展为所有场景、所有来源的卡顿均已解决。

## 现象与原因

现场现象为仿真偶发停顿约 1～2 秒，关闭 Gazebo GUI 后仍出现，停顿期间显示界面仍能拖动。此前现场诊断观察到 Gazebo 物理步进等待与墙钟回拨相对应：`/clock` 和里程计暂停更新，而控制进程仍可产生心跳。Gazebo Classic 的 `World::Step` 用墙钟差计算步进等待；墙钟向后跳会使等待量增大。这说明界面能够响应并不代表物理步进仍在进行。[Gazebo Classic 11 的 World.cc](https://github.com/gazebosim/gazebo-classic/blob/gazebo11/gazebo/physics/World.cc)

继续取证发现：

1. Ubuntu 的 `systemd-timesyncd` 处于 `active/enabled`，NTP 状态报告轮询周期为 32 秒，并反复出现约 −2 秒的校正量。
2. 全内核 syscall trace 同时出现 `chronyd-238` 和 `systemd-timesyn-595` 对 `CLOCK_REALTIME` 的 `clock_adjtime` 调用。Ubuntu 的普通进程视图未显示前者；通过 WSL 根调试环境确认它是 `/sbin/chronyd`，实际参考源配置为 `refclock PHC /dev/ptp0 poll 3 dpoll -2 offset 0`。
3. 原始 TSC 对 Hyper-V PTP 的速率偏差很小，而经过校时调整的 `CLOCK_MONOTONIC` 明显偏快。问题因此指向软件校时层，不能由内核的 TSC 调整警告直接推断硬件时钟已损坏。
4. 仅将 tick 从 10833 临时改回 10000 没有成功：170 秒内它再次变为 10833，并发生 5 次墙钟回拨。去除重复校时服务后，未继续干预 tick，时钟自行恢复稳定。

WSL 的多个发行版共享同一 Linux 内核，不同 PID 视图不会隔离内核校时。微软 WSLg 文档说明了共享内核及系统发行版的隔离方式；NixOS-WSL 维护者也已针对根命名空间 chronyd 与发行版校时服务冲突，合并移除重复服务的修复。[微软 WSLg 架构说明](https://github.com/microsoft/wslg#user-distro)、[NixOS-WSL 修复 #957](https://github.com/nix-community/NixOS-WSL/pull/957)

与全部证据吻合的过程是：timesyncd 根据网络 NTP 调整墙钟，chronyd 根据宿主侧 PHC 再通过渐进校时追赶，形成持续干扰。此处不判断两者哪个时间源的绝对 UTC 时间更准确；本次消除的是对同一时钟的争用。

## tick=10833 的含义

本机 `USER_HZ=100` 时，未补偿的标称 tick 为 10000 微秒。chrony 的 Linux 实现会同时使用 `ADJ_TICK` 和 `ADJ_FREQUENCY`，将较大的速率调整分配给 tick、将剩余部分分配给 freq。基线快照的组合为：

```text
tick = 10833 µs，freq = +33.651428 ppm
组合速率调整 ≈ (10833 − 10000) × 100 + 33.651428
             ≈ +83333.651428 ppm ≈ +8.333%
```

这与 chrony 默认最大渐进校时速率 83333.333 ppm，即 1/12，高度吻合。10833 是动态校时过程的表现，不能据此认定某个配置文件将 tick 永久写错。源码说明与数值对应支持高速渐进校时的解释；本目录 syscall trace 只记录调用者与参数地址，没有逐次保存 `timex` 内容，不能仅靠该文件还原每次调整的完整参数。[chrony maxslewrate 文档](https://chrony-project.org/doc/4.6/chrony.conf.html#maxslewrate)、[chrony Linux 频率调整实现](https://github.com/mlichvar/chrony/blob/master/sys_linux.c#L116)

修复后 tick 为 9999 也不表示仍有同类错误：结束快照的 freq 为 +99.983856 ppm，二者合计约为 `−100 + 99.983856 = −0.016144 ppm`。应联合观察 tick、freq 和实际时钟速率，不能只要求 tick 恒等于 10000。该结束快照值与整个 170 秒区间的平均速率偏差属于不同指标。

## 时钟采样方法与结果

使用本目录 `sample_clocks.py`，以只读方式打开 `/dev/ptp0`，通过动态 clock ID 读取 Hyper-V PTP；约每 50 毫秒采集一次 `CLOCK_REALTIME`、`CLOCK_MONOTONIC`、`CLOCK_MONOTONIC_RAW`。每组读取前后各取一次 PTP，以两次 PTP 的中点作为参考，并记录读取跨度。`adjtimex` 使用 `modes=0`，只查询状态。

速率偏差计算为 `(被测时钟增量 / PTP 增量 − 1) × 10⁶`，单位 ppm。墙钟突跳按相邻样本的 `ΔREALTIME − ΔMONOTONIC` 检测，绝对值超过 **20 毫秒**才记入 `wall_jumps`。因此“零次突跳”是此阈值及采样窗口下的结果，不表示证明了任意微小调整均不存在。PTP 是宿主时间参考，不是独立外部 UTC 标准。

| 指标 | 双校时基线 | 仅临时归一 tick：失败试验 | 仅保留 WSL chronyd |
| --- | ---: | ---: | ---: |
| 证据文件 | `before-tsc.json` | `after-tick-normalized.json` | `after-single-timesync.json` |
| PTP 计时长度 / s | 35.001769 | 170.035420 | 170.045430 |
| 样本数 | 710 | 3513 | 3394 |
| RAW 相对 PTP / ppm | +0.033027 | +0.020707 | +0.048922 |
| MONOTONIC 相对 PTP / ppm | +77906.889831 | +79392.681843 | −0.369872 |
| 墙钟突跳次数，阈值 20 ms | 1 | 5 | 0 |
| 墙钟跳变量 / s | −2.153334 | −1.768002 ～ −2.509411 | 无检测事件 |
| tick，起点 → 终点 / µs | 10833 → 10833 | 10000 → 10833 | 9999 → 9999 |
| freq，起点 → 终点 / ppm | +33.651428 → +33.651428 | +33.651428 → +33.651428 | +109.582825 → +99.983856 |
| 最大单组 PTP 读取跨度 / ms | 0.0564 | 0.0622 | 0.0756 |
| clocksource，起点与终点 | tsc | tsc | tsc |

最终干预只有停用 Ubuntu 重复校时服务；没有切换 clocksource，没有添加 tick 纠偏定时任务，也没有修改 ADRC 参数。`timesyncd-change.json` 保存了成功返回码 0，以及 `active/enabled → inactive/disabled` 的状态变化。

## 复现检查、修复与回滚

下面的服务命令在 **Ubuntu-22.04 的终端**执行。此方案适用于已确认 WSL 根命名空间 chronyd 正在运行的本机环境。

```bash
# 只读查看 Ubuntu 服务状态。
systemctl show systemd-timesyncd.service -p ActiveState -p UnitFileState

# 本次实际采用的修复：立即停止，并移除发行版内的开机启用关系。
sudo systemctl disable --now systemd-timesyncd.service

# 预期为 ActiveState=inactive、UnitFileState=disabled。
systemctl show systemd-timesyncd.service -p ActiveState -p UnitFileState
```

在 **Windows PowerShell** 中，只读查询 WSL 管理的 chronyd：

```powershell
wsl --system --user root --exec chronyc tracking
wsl --system --user root --exec chronyc sources -v
```

Ubuntu 的 `timedatectl` 可能不再报告发行版 NTP 服务运行，这不能用来判断根命名空间 chronyd 是否停止。结合上述 chronyc 查询与时钟采样检查同步状态。

复测时，在 Ubuntu 中运行以下命令会新建一个临时 JSON 证据文件，不调整时钟。脚本拒绝覆盖已有输出：

```bash
sudo python3 '/mnt/d/ROS2轻量仿真系统/usv_ros_motion_planning-ros2-humble/docs/validation/wsl-clock-fix-20260915/sample_clocks.py' \
  --duration 170 \
  --output "/tmp/wsl-clock-check-$(date +%Y%m%d-%H%M%S).json"
```

该修复立即作用于系统服务，**无需 `colcon build`，无需为使修复生效而重启 WSL**。本次没有修改 Ubuntu 服务以外的启动配置，也没有编辑系统发行版的 chrony 配置。

如需恢复本次记录中的原始 `enabled/active` 状态，在 Ubuntu 执行：

```bash
sudo systemctl enable --now systemd-timesyncd.service
```

这会重新启用已确认的第二套校时服务，可能恢复本次冲突；它是回滚命令，不是常规启动仿真的步骤。

## Gazebo 验证状态

`gazebo-after/` 中的三次到达发生在两套校时服务仍同时工作的阶段，只能说明当时任务到达，**不能证明时钟冲突或周期性停顿已经修复**。短任务到达也可能跨过一次暂停后继续完成。

`gazebo-single-timesync/` 是第一次修复后专项探针尝试。探针错误地等待至少两个 goal 订阅者，而自身没有订阅 goal，导致未进入正式观察窗口；该次记录不记为通过，也不作为产品故障。现有 `probe_gazebo.py` 已按实际需要等待至少一个 goal 订阅者。

**`gazebo-single-timesync-v2/` 已完成并通过专项判定，`passed=true`。** 探针在独立 ROS domain 197 和独立 Gazebo master 下运行 `world:=ocean`、`gui:=false`、`rviz:=false`、`controller_name:=adrc`；用 `CLOCK_MONOTONIC_RAW` 记录消息接收间隔，实际观察 150.000951328 秒。运行结束后已正常关闭本次探针自己启动的进程。

| 观测 topic | 样本数 | 最大接收间隔 / s | 超过 0.5 s 的间隔数，含窗口首尾 |
| --- | ---: | ---: | ---: |
| `/clock` | 1492 | 0.129709408 | 0 |
| `/otter/odom` | 1493 | 0.129788120 | 0 |
| `/otter/sensors/lidars/lidar/scan` | 2228 | 0.099764122 | 0 |
| `/otter/thrusters/left_thrust_cmd` | 1517 | 0.100526107 | 0 |
| `/otter/navigation/control_report` | 2131 | 0.100889709 | 0 |

实时因子为 **0.995310401**；非零左推力样本 **1161** 条。探针约在 0、40、80、120 秒发送四个目标，坐标依次为 `(43, −10)`、`(43, 10)`、`(23, 10)`、`(23, −10)`，单位米。该次加载配置的 SHA-256 为 `f1f791015fb8151dda116c7dda00a3fa0430d130d5e4301228bad7ac9b21de4b`，完整启动命令、实际目标发送时刻和原始消息记录见 `gazebo-single-timesync-v2/result.json`，进程日志见同目录 `launch.log`。

对原始数据进一步核对，`/clock`、里程计和激光的源时间均无回退或重复，源时间持续推进的最长等待分别约为 0.130、0.130、0.100 秒。这排除了本轮消息持续到达只是反复收到旧源时间的情况，计算结果见 `gazebo-single-timesync-v2/source-continuity.json`。结束后还通过进程查询确认没有残留本次 Gazebo 或探针进程。

该探针的判定是每个观测 topic 样本数大于 100 且最大间隔小于 0.5 秒，最大间隔包含观察窗口两端。它验证消息流是否持续以及是否再现秒级停顿，不以发送目标的数量证明到达，也不替代路径跟踪误差或控制器性能验收。

试验末段的 `final-service-status.json` 再次确认 Ubuntu timesyncd 为 `inactive/disabled`，clocksource 仍为 `tsc`，tick 为 9999、freq 为 +100.066025 ppm。全部仿真退出后又保存了 `post-validation-service-status.json`，确认该服务仍为 `inactive/disabled`。`chrony-after.txt` 显示根 chronyd 的参考源为 `PHC0`、更新间隔为 8 秒、Leap status 为 `Normal`。这验证了停用 Ubuntu 重复服务后，WSL 管理的校时服务仍在工作。

## 证据索引

| 文件或目录 | 用途与解释 |
| --- | --- |
| [sample_clocks.py](sample_clocks.py) | 只读时钟采样、速率与突跳统计实现 |
| [before-tsc.json](before-tsc.json) | 双校时基线、NTP 状态、原始时钟样本 |
| [tick-adjustment.json](tick-adjustment.json) | 早期仅写 `ADJ_TICK=10000` 的诊断试验记录，不是最终修复 |
| [after-tick-normalized.json](after-tick-normalized.json) | 上述试验失败：tick 再次变为 10833，5 次回拨 |
| [clock-syscalls.txt](clock-syscalls.txt) | 全内核 trace，显示两个命名空间来源的校时进程调用 |
| [timesyncd-change.json](timesyncd-change.json) | 最终服务变更命令、返回码与前后状态 |
| [after-single-timesync.json](after-single-timesync.json) | 单一校时服务下的 170 秒时钟层验证 |
| [final-service-status.json](final-service-status.json) | Gazebo 试验末段再次确认服务停用状态、tick/freq 与 clocksource |
| [post-validation-service-status.json](post-validation-service-status.json) | 全部仿真退出后的最终服务状态复核 |
| [chrony-after.txt](chrony-after.txt) | 根 chronyd 的 PHC0 参考源与正常同步状态 |
| [gazebo-after/summary.json](gazebo-after/summary.json) | 较早的三次到达；不作为修复完成证据 |
| [probe_gazebo.py](probe_gazebo.py) | Gazebo 专项消息间隔探针 |
| [gazebo-single-timesync/result.json](gazebo-single-timesync/result.json) | 首次探针未进入观察窗口的记录 |
| [gazebo-single-timesync-v2/result.json](gazebo-single-timesync-v2/result.json) | 修正探针后的 150 秒专项通过结果、启动命令与原始数据 |
| [gazebo-single-timesync-v2/source-continuity.json](gazebo-single-timesync-v2/source-continuity.json) | clock、odom、scan 源时间持续推进检查，无回退或重复 |
| [gazebo-single-timesync-v2/launch.log](gazebo-single-timesync-v2/launch.log) | 专项运行的进程日志 |

## 验证边界

- 170 秒时钟采样覆盖约五个原 32 秒轮询周期，支持当前连续运行期间的修复效果；没有验证长时间运行中的全部负载组合。
- WSL 完整关闭并重新启动后的服务状态、Windows 睡眠与唤醒后的时钟表现均未验证。停用关系已写入 Ubuntu，但不能把配置持久化等同于重启验收完成。
- RAW 相对 PHC 的测量排除了本次大幅速率异常主要来自原始 TSC；没有验证宿主 Windows 相对外部 UTC 的绝对误差。
- 本次修复针对有直接证据的周期性校时停顿，不覆盖独立的 CPU、磁盘、驱动、渲染或算法负载问题。
- NixOS-WSL 的维护者修复是同类问题的参考证据，不代替本机测量；微软公开资料说明共享内核与系统发行版结构，没有据此声称已核实本机版本的具体 chronyd 启动源码调用点。[同类问题的 PHC 配置与进程记录](https://github.com/nix-community/NixOS-WSL/issues/956)
