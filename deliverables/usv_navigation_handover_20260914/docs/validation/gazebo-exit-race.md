# Gazebo SIGINT 退出异常：证据与诊断边界

归档日期：2026-09-11。状态：已确认一次真实退出异常，尚未取得崩溃栈；没有修改 Gazebo 插件、基础仿真或 harness 终止逻辑。

## 试验与结论

本记录对应 `python-pid-final-001/python_pid_turn_01`。manifest 记录 Python 源码提交为 `1a82de47a21238b7c5b8e1c3794db5eb5ae7235a`，ROS domain 为 182，场景为转弯，控制器为 PID。

该例导航到达判据通过，但 gzserver 在退出时抛出 `boost::wrapexcept<boost::thread_resource_error>`，信息为 `boost thread: trying joining itself: Resource deadlock avoided`，进程退出码为 `-6`（SIGABRT）。导航结果与进程退出健康必须分列，不能据导航通过宣称全生命周期通过。其余节点，包括 Python 导航、建图及 C++ costmap 支撑节点，均有正常退出记录。

目前优先排查 Gazebo 依赖的退出竞态，但不能据此认定某个插件或具体析构函数为根因，也未确认该异常的发生概率。本轮只读归档，不附加正式性能试验进程；后续再次出现时再收集栈，不为这一退出问题扩大导航迁移范围。

## 原始证据与校验

完整原始文件保留在 WSL Ubuntu-22.04，未复制大日志：

| 文件 | 完整路径 | SHA256 |
| --- | --- | --- |
| launch.log，31,502 bytes | `/home/usv/usv_migration/results/python-pid-final-001/python_pid_turn_01/launch.log` | `0bdc344235a86f63286ba75a8fb07f67a1037be8787668bde0637f79f019b990` |
| events.jsonl | `/home/usv/usv_migration/results/python-pid-final-001/python_pid_turn_01/events.jsonl` | `faf1139fd89accade34d88607dd1f09986e7fc1aa82e5fd770e9d7d5817e973e` |

试验清单：`/home/usv/usv_migration/results/python-pid-final-001/manifest.json`。

下列摘录为 launch.log 第 145–158 行原文，包含末尾 LF，14 行摘录自身 SHA256 为 `960a14ef144e8e898ab3585034fad058a280de57816851d871325b60156123a0`：

```text
[fake_localization_node-5] [INFO] [1789088998.712543826] [rclcpp]: signal_handler(SIGINT/SIGTERM)
[robot_state_publisher-2] [INFO] [1789088998.713934837] [rclcpp]: signal_handler(SIGINT/SIGTERM)
[INFO] [fake_localization_node-5]: process has finished cleanly [pid 83719]
[INFO] [robot_state_publisher-2]: process has finished cleanly [pid 83713]
[INFO] [joint_state_publisher-3]: process has finished cleanly [pid 83715]
[INFO] [perception_mapping_node-6]: process has finished cleanly [pid 83721]
[INFO] [sim_env_node-8]: process has finished cleanly [pid 83725]
[costmap_support_node-7] [INFO] [1789088998.979986132] [otter.costmap]: Cleaning up
[costmap_support_node-7] [INFO] [1789088998.981914125] [otter.costmap]: Destroying bond (costmap) to lifecycle manager.
[costmap_support_node-7] [INFO] [1789088998.985314218] [otter.costmap]: Destroying
[INFO] [costmap_support_node-7]: process has finished cleanly [pid 83723]
[gzserver-1] terminate called after throwing an instance of 'boost::wrapexcept<boost::thread_resource_error>'
[gzserver-1]   what():  boost thread: trying joining itself: Resource deadlock avoided
[ERROR] [gzserver-1]: process has died [pid 83711, exit code -6, cmd 'gzserver /home/usv/usv_migration/python/install/sim_env/share/sim_env/worlds/ocean.world -slibgazebo_ros_init.so -slibgazebo_ros_factory.so -slibgazebo_ros_force_system.so'].
```

可在 WSL 复核原文，不改写任何文件：

```bash
trial=/home/usv/usv_migration/results/python-pid-final-001/python_pid_turn_01
sha256sum "$trial/launch.log" "$trial/events.jsonl"
sed -n '145,158p' "$trial/launch.log" | sha256sum
nl -ba "$trial/launch.log" | tail -n 25
```

## 信号顺序与时间边界

launch.log 第 134 行记录收到 SIGINT，第 135–141 行分别向自己的子进程转发 SIGINT；第 141 行对 gzserver 的转发只出现一次。第 158 行的异常退出由仍存活的 launch 父进程记录。

当前受审阅的 `scripts/validation/ros_acceptance.py:73–101` 中，OwnedProcess.close 首先只对自己创建的 launch PID 发送 SIGINT，等待最多 20 秒；随后才针对自身进程组处理残留进程的 SIGTERM/SIGKILL。该子进程通过 `start_new_session=True` 创建（第 64 行）。本次事件的单调墙钟数值为：

| 事件 | wall_monotonic_s |
| --- | ---: |
| acquisition_finished | 67851.518150903 |
| trial_finished | 67855.228764355 |
| 两事件间隔 | 3.710613452 s |

这一间隔覆盖采集结束至最终记录，包含清理及记录开销，**不是单独测得的 gzserver 析构耗时**。间隔小于 20 秒，且 launch 已记录 gzserver 自身的 `-6` 退出；结合信号代码与日志，本次没有重复 SIGINT 或超时强杀导致异常的证据。此结论不等同于系统调用级信号追踪，也不能排除 Gazebo 内部退出回调之间的竞态。

## 已完成的只读定位

- 本机 `/usr/include/boost/thread/detail/thread.hpp:732–737` 在 `this_thread::get_id() == get_id()` 时抛出上述 self-join 信息。因此本条异常确实指向线程尝试 join 自己，不能解释为普通“等待过久”。
- 原项目 Gazebo 插件源码未发现显式线程创建/join；实际安装的 8 个项目插件共享库也未发现 Boost/std 线程 join 的动态导入。这不能排除它们调用依赖清理函数后间接进入该路径。
- 本机 Gazebo transport 库导出 `ConnectionManager::Stop/Fini`、`Node::Fini`，并导入 `boost::thread::join_noexcept()`。这仅支持优先调查依赖生命周期，不能单凭符号确定异常调用链。
- 环境版本：Gazebo/libgazebo11 `11.10.2+dfsg-1`，Boost thread `1.74.0-14ubuntu3`，ros-humble-gazebo-ros `3.9.0-1jammy.20260726.134039`。
- `/proc/sys/kernel/core_pattern` 为 `|/wsl-capture-crash %t %E %p %s`；本次试验目录及 `/var/crash` 中未找到可用于此次 gzserver 的 core。未获得堆栈、线程身份或具体 join 调用者。

因此尚未实施代码修复。取得栈后，若证实 Stop/Fini 从待回收线程自身进入，最小修复方向是把停止请求与最终 join 分开，由所属控制线程完成回收。不能用忽略异常、无条件 detach 或跳过 join 来伪造退出通过；具体补丁必须由实际栈决定。

## 可选复现诊断：仅用于自己启动的非性能试验

以下命令是准备材料，本轮未执行。不要附加正式性能矩阵。发生同类异常后，在独立诊断运行中沿用原 world、插件与 harness 信号顺序，只增加调试器；调试结果不能并入正式性能数据。

从自己刚启动的诊断进程取得 `OWN_LAUNCH_PID`、`OWN_GZSERVER_PID`，并设置该次 world 的绝对路径 `OWN_WORLD_PATH` 与尚不存在的输出目录 `DEBUG_DIRECTORY`。**不能复用本报告中的历史 PID 83711。** 下列前置检查要求当前用户、gzserver 可执行文件、world 路径以及到自有 ROS launch 的祖先关系全部匹配；失败即退出，不能用按名称 pkill/killall 代替。

```bash
set -eo pipefail
: "${OWN_LAUNCH_PID:?set the PID of the diagnostic ros2 launch you started}"
: "${OWN_GZSERVER_PID:?set its current gzserver PID}"
: "${OWN_WORLD_PATH:?set the exact diagnostic world absolute path}"
: "${DEBUG_DIRECTORY:?set a new absolute output directory}"
python3 - "$OWN_LAUNCH_PID" "$OWN_GZSERVER_PID" "$OWN_WORLD_PATH" <<'PY'
import os
from pathlib import Path
import sys

launch_pid, gz_pid = map(int, sys.argv[1:3])
world = Path(sys.argv[3]).resolve(strict=True)
assert launch_pid > 1 and gz_pid > 1 and launch_pid != gz_pid

def command(pid):
    proc = Path('/proc') / str(pid)
    assert proc.stat().st_uid == os.getuid(), 'process belongs to another user'
    return [v.decode() for v in (proc / 'cmdline').read_bytes().split(b'\0') if v]

launch_args = command(launch_pid)
assert 'launch' in launch_args and 'sim_env' in launch_args, launch_args
gz_args = command(gz_pid)
assert Path(gz_args[0]).name == 'gzserver', gz_args
assert str(world) in gz_args, gz_args
assert Path(f'/proc/{gz_pid}/exe').resolve().name == 'gzserver'
ancestor = gz_pid
seen = set()
while ancestor != launch_pid:
    assert ancestor > 1 and ancestor not in seen, 'not a child of the owned launch'
    seen.add(ancestor)
    stat = Path(f'/proc/{ancestor}/stat').read_text()
    ancestor = int(stat.rsplit(')', 1)[1].split()[1])
print({'verified_launch_pid': launch_pid, 'verified_gzserver_pid': gz_pid,
       'world': str(world), 'gzserver_cmdline': gz_args})
PY
[[ "$DEBUG_DIRECTORY" == /* && ! -e "$DEBUG_DIRECTORY" && ! -L "$DEBUG_DIRECTORY" ]]
mkdir -- "$DEBUG_DIRECTORY"
gdb -q -batch -p "$OWN_GZSERVER_PID" \
    -ex 'set pagination off' \
    -ex 'handle SIGINT nostop noprint pass' \
    -ex 'handle SIGTERM nostop noprint pass' \
    -ex 'handle SIGABRT stop print pass' \
    -ex continue \
    -ex 'thread apply all bt full' \
    -ex continue 2>&1 | tee "$DEBUG_DIRECTORY/gzserver-exit-gdb.log"
```

调试器等待期间，让该诊断运行原有的 harness 按原顺序清理；不要额外向 Gazebo 发送另一轮 SIGINT。在 SIGABRT 停住时记录所有线程，最后的 continue 放行原信号。若本次正常退出，GDB 可能提示进程已退出、无栈可读；这只说明本次未捕获异常，不证明问题消失。若 ptrace 权限限制附加，可在原有授权范围内仅对已核对的自有 PID 使用提权 GDB，无须修改全局 ptrace 配置。若匹配检查与附加之间目标已退出，停止本次诊断，不搜索其他 Gazebo 进程替代。

M5 清理与干净构建仍受迁移验收门槛约束；本归档不执行删除、构建或仿真。
