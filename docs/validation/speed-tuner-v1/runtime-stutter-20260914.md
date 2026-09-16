# 2026-09-14 运行中 Gazebo 间歇卡顿诊断

后续无 GUI 对照已定位秒级停顿的直接机制：WSL 系统校时回拨触发 Gazebo 长休眠。
见 [后续根因记录](headless-clock-root-cause.md)。下文保留首次采样时已确认的性能问题及当时的判断边界。

本轮为只读诊断，未停止用户仿真、修改控制参数、改变图形驱动或删除日志。
原始 40 秒同步采样：`runtime-stutter-20260914.json`。

## 确认的停顿位置

- 当前 launch PID 26501，gzserver 26503，gzclient 26505，RViz 26515，控制节点 26527。
- `/clock` 收到 339 条，里程计 338 条；两者在采样墙钟约 13.77 s 处同时出现 2.181 s 接收间隔。
- 此时仿真时钟停在 150.6 s。控制报告仍持续（全窗口最大间隔 0.119 s），
  决策、控制、感知心跳最大间隔分别约 0.226、0.227、0.221 s。
- 控制节点在停钟持续 0.5 s 后进入 SIMULATION_PAUSED，共观察到 16 条对应报告。
  这次是新看门狗对真实时钟停止的响应，不是上次短暂重复时钟的误判。
- 全窗口平均实时倍率 0.849。控制节点累计统计 P99 1.033 ms、最大 6.119 ms，
  远小于 100 ms 控制周期。这里的耗时统计来自进程累计窗口，不仅包含本次 40 秒。
- 停顿期间 gzserver 的单秒 CPU 占用从约 70%～80% 降到约 3%，
  因此不能仅凭总 CPU 较高就认定具体阻塞函数。未获得该时刻的调用栈。

## 已确认的性能问题

### CPU 软件渲染

`glxinfo -B` 显示 `llvmpipe (LLVM 15.0.7, 256 bits)`、`Accelerated: no`。
Gazebo 客户端实际存在 llvmpipe-0 至 llvmpipe-11 工作线程，采样平均 CPU 为 285.5%
（100% 表示占用一个逻辑核心）。RViz 为 32.8%，gzserver 为 67.2%，
建图节点为 53.7%，决策为 10.6%，控制为 7.7%。

Windows CPU 的另一次瞬时采样为 97%，与上述 40 秒平均窗口不同。
WSL 分配 12 个逻辑 CPU，MemAvailable 约 21 GB，memory pressure 为零，
本次未见内存不足证据。

Windows 枚举 RTX 5060 状态 OK，AMD 核显状态 Error、ConfigManagerErrorCode=22，
还存在远程桌面虚拟显示适配器；这些是图形环境线索，未证明哪一项造成 WSL 回退。
`/dev/dxg` 存在。单独诊断子进程中设置 NVIDIA/Intel adapter 或 loader override
仍得到 llvmpipe；强制 `GALLIUM_DRIVER=d3d12` 则创建 OpenGL 上下文失败。
这些环境变量测试只作用于 glxinfo 子进程，未改变运行中的仿真。

### 动力学插件高频调试日志

`/home/usv/.gazebo/server-11345/default.log` 检查时为 1,000,904,249 字节。
文件尾部连续来自 `usv_gazebo_dynamics_plugin.cc` 的 gzdbg：速度、加速度、
stateDot、附加质量、Cmat、Dmat、阻力、合力和四个浮力分段。
该插件在物理更新回调中无条件进行这些输出；世界配置 max_step_size=0.001 s、
real_time_update_rate=1000。进程 I/O 采样显示正常运行区间约 3 MB/s 的磁盘写入。
日志洪泛带来格式化和写入开销，但尚未用开/关对照证明它单独造成 2.181 s 的停顿。

## 建议顺序及验证边界

1. 下次启动在原命令增加 `gui:=false rviz:=true`，保留 RViz、关闭 Gazebo 三维客户端，
   对照相同航行场景的时钟停顿和实时倍率。这可去掉当前最大的已观测 CPU 使用者。
2. 将动力学热循环详细日志改成默认关闭、按需启用或限频；保留必要警告和错误。
   只删除历史日志不能停止继续写入。此轮尚未修改或构建该 C++ 插件。
3. 单独修复 WSL 图形加速，检查显卡/WSLg/OpenGL 驱动链路，直到渲染器不再显示 llvmpipe。
   本轮未安装驱动、启停设备或重启 WSL。
4. 用同样的采样复验；若仍出现秒级停钟，再对 gzserver 做阻塞调用采样。

注意：探针最初订阅的 `/otter/scan` 不是本系统实际雷达话题，所以没有直接激光消息样本；
本报告没有使用该空订阅推断雷达断流。建图状态来自实际 `/otter/mapping_status`。
目前结论是 Gazebo 侧真实停顿及两个明确性能问题，尚不能声称已经查明唯一阻塞调用或修复卡顿。
