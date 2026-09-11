# fake_localization 并行数组边界：red 复现与普通安装 green

原 `e381832` 安装收到一条 `name=['otter']`、`pose=[]`、`twist=[]` 的 `ModelStates` 后发生 SIGSEGV，GDB 栈顶位于 `FakeLocalizationNode::modelStatesCallback`。本次证实输入长度缺陷，**不等于** 已确定正式 PID 圆柱第 3 次 SIGINT 后退出 `-11` 的原因。

## 3afb25f 普通安装 green：已执行并通过

在 control 完成固定 `3afb25f84232e560c62599a721dc0bfbe1a21657` 的 13 包构建和安装审计后，独立 domain 188 执行 `localization_smoke.py --mode acceptance`。本次使用普通 `/home/usv/usv_migration/python/install`，**不是** 后续待完成的全新 clean workspace 构建。

缺失 pose/缺失 twist 均被拒绝；yaw=π/2 时世界速度 `(3,4)` 转为体轴 `(4,-3)`，角速度保持 0.2；重复 clock 未重复输出；clock 从 10 回退到 5、再到 5.1 后继续输出；合法输入流期间仅给自有 PID 112410（starttime 7887932 ticks）一次 SIGINT，节点退出码为 0，随后只读确认该 PID 已不存在。测试脚本返回 0，没有启动 Gazebo 或 OS 采样器。

- 实际安装源文件 SHA：`ec5f0d59cc35e6f76066add8cd0e3531d422ae3d7b6f5edfc6566bf800a36068`，identity TF 逻辑位于 97–102 行。
- 实际二进制 SHA：`85999f583f4da714a83bc5650d7bdc449770f30e5985ee691784b53f6b8eead0`。
- 本次随机命名空间下实际日志第 1 行为 `localization_smoke_62066bd65379/map → localization_smoke_62066bd65379/odom → localization_smoke_62066bd65379/base_link`，与命名空间派生代码及 odom frame 检查一致。普通 launch 的 `otter/map → otter/odom → otter/base_link` 链须另引用候选实际日志，不能由这个随机命名空间测试冒充。
- 原结果目录：`/home/usv/usv_migration/results/localization-green-grid-endpoint-preflight-001`；仓库保留 [green receipt](green-grid-endpoint-receipt.json)、[源码/安装与 frame 核查](green-grid-endpoint-environment-receipt.json)、[完整短日志](green-grid-endpoint-node.log)。log SHA 为 `0ea8cd95a152e931bcebc1f6ecf5377302f4cb4d18dcaa38f304ebec3539abd4`。
- 后续普通 launch 已另行核对：完成的 `python-grid-endpoint-straight-candidate-001/python_pid_straight_01/launch.log` 第 16 行明确记录 `otter/map → otter/odom → otter/base_link`，最终 log SHA 为 `74ec72f657edb9f09beab1600af7965c3ba1f52e74f84f0780746a3cda6bc52a`。路径、完成标记、源码/二进制关联见 [普通 launch frame 补充 receipt](normal-grid-endpoint-launch-frame-receipt.json)。早先 environment receipt 的 pending 字段保留其采样时刻状态，由本条补充完成证据。

这份 green 验证输入边界和一次合法流退出；没有据此声称原先低频 shutdown `-11` 已经归因或完全消失。全新 clean workspace 仍需重复入口/安装与 green 核验。

## 原正式异常的证据边界

原日志：`/home/usv/usv_migration/results/python-release-grid-001/python_pid_single_obstacle_03/launch.log`。第 12 行启动 fake_localization PID 105109；第 74 行转发 SIGINT，第 81 行节点 signal handler 记录收到信号，第 92 行记录退出 `-11`。同例 Gazebo 及其余节点正常退出。该日志没有调用栈、崩溃时的 `ModelStates` 数组长度或逐消息内容，不能区分库清理竞态、非法输入及其他原因。

修复前源码按 `name.size()` 遍历，却直接索引 `pose[i]` 与 `twist[i]`，没有长度检查。该缺陷存在于 `cpp-baseline` 原标签；相对该标签，正式 `e381832` 的该文件仅将重复时间戳过滤的 `<=` 改为 `==` 以允许回退后恢复，没有改动原 `init→spin→shutdown` 退出路径，也没有加入自建线程或自定义信号处理。

## 已实际执行的单次 red

使用独立 domain 188、随机节点命名空间，仅启动本任务的 GDB/节点，不启动 Gazebo。只发送上述 **一条** 畸形输入，没有信号注入。GDB 管理进程 PID 109958，inferior PID 109972；GDB 自身返回 0，**被调试程序在回调中收到 SIGSEGV**，不能把 GDB 的 0 解释为节点正常退出。

```bash
source /opt/ros/humble/setup.bash
source /home/usv/usv_migration/python/install/setup.bash
PYTHONDONTWRITEBYTECODE=1 python3 \
  '/mnt/d/ROS2轻量仿真系统/usv_ros_motion_planning-ros2-humble/scripts/validation/localization_smoke.py' \
  --workspace /home/usv/usv_migration/python \
  --output /home/usv/usv_migration/results/localization-malformed-red-001 \
  --domain-id 188 --mode malformed-probe --gdb
```

该命令已执行，复现时必须使用新的输出路径。完整原栈保存在 `/home/usv/usv_migration/results/localization-malformed-red-001/node.log`，SHA256 为 `1cf9675972cd5fc9df04612dd2cc831c4a76672d0cc135ce4e01f6d92fa9623b`。仓库保留 [red receipt](red-receipt.json) 与 [回调栈节选](red-backtrace-excerpt.txt)；receipt 含实际二进制和脚本 SHA、命令、身份与时间。该二进制无源行调试信息，栈可定位到回调函数，未取得具体机器指令对应的源行。

## 最小修复与 green 门槛

源码仅在回调索引前比较 `name/pose/twist` 三数组长度。不一致时记录警告并直接返回；保留原作者信息、时间戳与坐标转换语义、话题和生命周期。当前 `/home/usv/usv_migration/python` 的源码/install 均未重建或修改。

新增 `scripts/validation/localization_smoke.py --mode acceptance` 已完成 AST/CLI 检查和上述普通安装 ROS green。**最终全新 clean workspace 的 green 尚未执行**；待其构建完成后，在该安装环境下运行：

```bash
source /opt/ros/humble/setup.bash
source /home/usv/usv_migration/python-clean-final-001/install/setup.bash
PYTHONDONTWRITEBYTECODE=1 python3 \
  '/mnt/d/ROS2轻量仿真系统/usv_ros_motion_planning-ros2-humble/scripts/validation/localization_smoke.py' \
  --workspace /home/usv/usv_migration/python-clean-final-001 \
  --output /home/usv/usv_migration/results/localization-green-001 \
  --domain-id 188 --mode acceptance
```

验收依次检查缺失 pose/缺失 twist 被丢弃且不输出 odom；合法 yaw=π/2、世界速度 `(3,4)` 转为体轴 `(4,-3)`；重复 clock 不重复输出、回退后继续产生新 odom；合法消息流期间仅给自有节点 PID 发 SIGINT 并要求退出码 0。所有终止操作都验证创建的 PID/starttime，不操作正式矩阵进程。单次正常退出通过仍不证明先前低频退出异常已归因或完全消失；若再次发生，应在独立复现中捕获相同退出路径的栈。
