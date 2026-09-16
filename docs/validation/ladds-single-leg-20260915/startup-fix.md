# 2026-09-15 启动问题排查与验证

## 现场原因

1. Conda base使用Python 3.13，而ROS Humble的rclpy扩展面向系统Python 3.10。使用`/usr/bin/env python3`的子节点启动失败。
2. 助手先前的后台ROS桥仍连接旧Windows中继。同端口的第二个中继因Windows `SO_REUSEADDR`可以重复绑定而启动；单客户端中继未及时处理第二个连接，新的ROS桥等待hello超时。
3. 两套同域仿真同时运行，导航状态出现`MODULE_RESTART`。已关闭助手先前启动的后台仿真与桥，保留用户终端中的实例。

## 修复

- 三个LaDDS WSL启动脚本优先使用系统Python，仅作用于自身进程树。
- Windows中继独占监听端口，在启动DDS前完成端口绑定；SDK启动失败释放监听资源。
- 第二个TCP客户端收到明确busy错误，原客户端仍可收发；旧客户端断开后可连接新客户端。
- ROS桥退出触发本次launch整体退出，避免任务适配器单独残留。
- 默认ROS域从211调整为42，启动说明明确两端ROS配置相同、LaDDS域继续为1。此次故障的直接原因是解释器与连接占用，域调整不是hello超时修复手段。

## 验证结果

- 修复前三个新增回归用例全部失败：复现hello超时、Windows重复绑定以及SDK启动失败未清理。
- 修复后Windows桥包测试及启动测试收集：124通过、4跳过（含依赖真实ROS的测试）。
- 修复后WSL桥包及真实ROS启动测试：128通过、1个Windows专属测试跳过。
- 真实ROS测试预置Conda base的PATH，分别启动通信桥、单段适配与桥，均完成回环TCP握手；拒绝连接时验证适配节点随launch退出。使用独立ROS域84、临时端口和假DDS边界，没有下达航行任务。
- 单独从WSL连接用户现有Windows中继`172.27.128.1:17880`，收到`REAL_RELAY_HELLO_OK: 1.0.3`后立即断开。说明清理旧连接后实际跨Windows/WSL链路可完成握手。
- 用户旧仿真只读状态仍为`MODULE_RESTART`，需要在终端重启。现有Windows中继要重启才会加载上述防重复连接修复。

运行ROS启动回归检查：

```bash
source /opt/ros/humble/setup.bash
source /tmp/usv_ladds_single_leg_20260915/install/setup.bash
/usr/bin/python3 -m pytest scripts/validation/test_ladds_startup_ros.py \
  src/core/communication_module/usv_ladds_bridge/test -q
```

启动回归验证的是解释器、握手和失败清理，未重复执行整段航行；原完整航行结果见本目录README。
