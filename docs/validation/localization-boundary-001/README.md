# fake_localization 并行数组边界：red 复现与待执行 green

原 `e381832` 安装收到一条 `name=['otter']`、`pose=[]`、`twist=[]` 的 `ModelStates` 后发生 SIGSEGV，GDB 栈顶位于 `FakeLocalizationNode::modelStatesCallback`。本次证实输入长度缺陷，**不等于** 已确定正式 PID 圆柱第 3 次 SIGINT 后退出 `-11` 的原因。

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

新增 `scripts/validation/localization_smoke.py --mode acceptance` 已完成 AST/CLI 检查，但**尚未进行修复后二进制的 ROS green 测试**。待最终干净构建后，在其安装环境下运行：

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
