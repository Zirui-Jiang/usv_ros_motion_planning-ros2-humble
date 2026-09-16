# 关闭 Gazebo GUI 后的秒级卡顿：系统校时回拨

> 2026-09-15 更新：已进一步定位为 WSL 根命名空间 chronyd 与 Ubuntu
> systemd-timesyncd 争用同一内核时钟，已停用 Ubuntu 的重复校时。
> 下文保留首次只读取证过程；当前修复及复测见
> [WSL 双校时冲突修复记录](../wsl-clock-fix-20260915/README.md)。

## 结论

本次秒级停顿的直接机制已定位：WSL 系统日历时间发生约两秒的向后跳变，
Gazebo Classic 的物理步进限速使用日历时间差，进而请求约两秒的休眠。
RViz 事件循环以及 ROS 控制/心跳仍可运行，因此卡顿时视角可以拖动。
此前确认的软件渲染和日志洪泛是额外性能问题，但不能作为本次周期性停顿的完整解释。

只进行了只读诊断，未修改系统时钟、时钟源、NTP、驱动或仿真配置，未附加会暂停进程的调试器。

## 本轮运行与采样

用户先运行 gui:=false rviz:=true（gzserver PID 27159），后正常 Ctrl+C 关闭，
日志明确记录 SIGINT；新实例 gzserver PID 27584，物理活跃线程 TID 27809。
第一次针对已退出实例的采样没有得到有效数据。随后以普通用户收集 ROS 消息，
独立特权只读探针收集内核等待点，二者按同一个 monotonic 时间轴关联。

有效数据：

- `headless-topics.json`：55 秒 ROS 采样。
- `headless-thread-waits.json`：同期线程等待点及等待时的栈内存。
- `headless-sleep-clock.json`：后续 45 秒系统日历时间/monotonic 对照和休眠请求采样。
- `headless-sleep-symbols.json`：保存栈中代码地址的离线符号解析。

55 秒窗口内：

- clock/odom 同步出现 2.540、1.594、0.646、2.172 s 的接收间隔。
- scan 最大接收间隔 3.575 s。
- 控制报告最大间隔 0.110 s，各模块心跳最大间隔约 0.218 s。
- 平均实时倍率 0.868，gzserver 平均占用约 48.6% 单个逻辑 CPU，写盘约 3.16 MB/s。
- 停顿期间 TID 27809 的内核等待点连续为 hrtimer_nanosleep；不是控制器计算超时。

## 回拨与长休眠的直接对应

后续 45 秒探针每约 20 ms 同时读取 time.time() 和 time.monotonic()，以两者增量之差识别
日历时间跳变，并读取 clock_nanosleep 请求的 timespec：

| 探针相对时间 | 日历时间相对 monotonic 的跳变量 | 随后物理线程休眠请求 |
|---|---:|---:|
| 约 7.32 s | −2.311185 s | 2.334164824 s |
| 约 39.55 s | −2.177700 s | 2.199350397 s |

相邻事件约相隔 32.23 s。timedatectl timesync-status 同时显示：

```text
Server: 185.125.190.56 (ntp.ubuntu.com)
Poll interval: 32s
Offset: -2.177699s
Delay: 206.637ms
Frequency: +33.270ppm
```

注意：远程读取运行中栈会有竞争，原始 timespec 样本中存在已被复用的无效值。
分析仅使用秒数合理、纳秒在 [0, 10^9) 且连续重复观测到的有效长休眠值。
栈地址扫描属于辅助证据，不是完整展开的调用栈；匹配到了安装版本 11.10.2 的
World::RunLoop、World::Step 和 Time::Sleep。

## Gazebo 源码对应

Gazebo Classic 的 World::Step 用上一步墙钟时间加更新周期、减去当前墙钟和补偿项计算
sleepTime；若系统时间倒退约两秒，该差值会额外增加约两秒，然后调用 Time::Sleep。
见 [Gazebo World.cc](https://github.com/gazebosim/gazebo-classic/blob/gazebo11/gazebo/physics/World.cc#L735-L766)。

Linux 下 GetWallTime 使用 clock_gettime(0)，而 Time::Sleep 使用 CLOCK_MONOTONIC 的
相对 clock_nanosleep。休眠一旦按错误的墙钟差算出，便真实等待这段时长。
见 [Gazebo Time.cc](https://github.com/gazebosim/gazebo-classic/blob/gazebo11/gazebo/common/Time.cc)。

## 首次诊断时的候选方向（后续已收敛）

直接机制已定位，但系统时间为何需要每 32 秒纠正约两秒，还需进一步验证宿主与 WSL 时钟关系。
当前 clocksource 为 tsc，可用源包括 hyperv_clocksource_tsc_page、hyperv_clocksource_msr、acpi_pm。
内核日志还存在 `Adjusting tsc more than 11%` 的记录，是继续核查时钟源的线索，不能单凭此认定硬件故障。

后续比较发现 CLOCK_MONOTONIC_RAW 与 Hyper-V PHC 基本一致，异常速率出现在软件校时层。
内核 syscall 跟踪抓到根命名空间 chronyd 和 Ubuntu timesyncd 同时调节时钟。
停用 Ubuntu timesyncd、保留根 chronyd 后，170 秒未发生回拨，clocksource 仍为 tsc。
因此没有执行切换 Hyper-V clocksource，也没有修改 .wslconfig；详见上述更新记录。
不应通过增大 Kp/Ki、放宽控制暂停门限或永久停用时间同步来掩盖问题。
软件渲染及热循环日志可以另行优化，但不能替代上述校时问题的修复。
