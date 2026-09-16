# 2026-09-15 单段航渡验证

本页记录修正前“重复报文去重”的历史实现。用户后续确认每次0211下发都是新执行，当前适配器已按该约定修改；下述循环发送和68包仅执行一次的结果不作为新行为的验证。当前操作见[单段航渡指南](../../../src/core/communication_module/usv_ladds_bridge/README.single-leg.md)。

## 结论

Windows原生LaDDS → 本机TCP → WSL ROS 2任务适配 → 实际Gazebo导航 → LaDDS 0052反馈，完整闭环通过。

本轮由独立DDS测试程序发送并接收，原生工具专用界面对0052的显示效果未验证；原生Linux LaDDS SDK与授权问题未在本轮解决。

## 场景和输入

- 原点WGS84：latitude=30.5453，longitude=114.392931，ENU朝向，高度0。
- 用户目标：latitude=30.546127，longitude=114.3938；本地坐标(83.3836767487,91.6829391350)m。
- speed=4节=2.0577777778m/s；travelId=53；taskType=16；count=1；point.type=1。
- 默认船位约(23,-10)，真实里程计略有波动。地图、障碍、PID/LOS/A*配置保留；独立profile的velocity_max为2.1m/s。
- 原生DDS测试域21；中继172.27.128.1:17881；ROS域211；ROS_LOCALHOST_ONLY=1；Gazebo端口11347。
- 输入文件：[单点0211](../../examples/ladds-0211-single-leg.json)。正式用户工具使用DDS域1、端口17880，按照[操作说明](../../../src/core/communication_module/usv_ladds_bridge/README.single-leg.md)启动。

## 完整观测结果

| 检查 | 结果 |
|---|---|
| 连续收到同任务 | 68包 |
| 导航request_id数量 | 1 |
| 轨迹最高期望速度 | 2.0577777778m/s |
| 实际平面速度峰值 | 2.2220798505m/s |
| 接受到完成墙钟时间 | 约67.65s |
| 完成时地图位置 | (83.6660482598,92.6031253773)m |
| 完成时距离误差 | 0.9625364361m |
| 完成时平面速度 | 0.1601974644m/s |
| 独立DDS反馈 | 0052：travelId=53、taskFinish=1 |
| DDS测试程序总等待 | 72.718s，含发现预热 |

判定标准：68次重复输入仅一个任务，轨迹含正确4节期望速度，实际到达误差≤1.5m、速度≤0.25m/s，且独立DDS端收到匹配反馈。`GOAL_REACHED`来自真实控制报告，未发布假到达状态或瞬移艇位。

4节是期望航速，当前PID存在超调；本轮不宣称实际速度严格限在4节或2.1m/s，也未验证实船、所有障碍场景、所有航速和持续运行可靠性。

## 证据

- [完整ROS观测及轨迹](gazebo-observer.json)
- [独立DDS发送和接收](dds-peer.json)
- [实际航迹与航速图](trajectory-speed.png)
- [导航ROS测试](navigation-tests.txt)
- [旧调用兼容与导航回归：41项通过](navigation-compatibility-tests.txt)
- [最终桥包：122项通过，1项Windows专用测试在Linux跳过](bridge-final-tests.txt)
- [最终仿真日志](simulation-final.log)、[桥接日志](bridge-final.log)

初轮DDS已完成，但观测进程ROS_LOCALHOST_ONLY=1、其他节点=0，观测漏收。保留[该失败记录](observer-network-mismatch.json)和[首轮DDS成功记录](dds-peer-first.json)；它们未替代最终完整验证。

正常Gazebo闭环之后补充了服务BUSY/超时恢复，使用生成的ROS消息及受控服务替身验证3次同ID重试、超时保留不确定状态、进程会话变化中止和迟到响应保护。该异常测试没有模拟真实网络丢包。旧六参数规划调用兼容也已回归通过。

交付时已停止域21的自动测试进程，重新启动域1中继（0211接收/0052发送、TCP17880）和ROS域211后台仿真；目标任务未提交，最终只读检查为IDLE。待命日志：[仿真](ready-simulation.log)、[桥接](ready-bridge.log)。运行状态是本次交付时快照，后续关闭应用/WSL或手动结束进程后需按说明重新启动。

## 复现自动验证

仿真按操作说明启动但使用headless=true。Windows中继改为`-DomainId 21 -Port 17881 -ReceiveTopic 0211 -SendTopic 0052`，ROS桥通过`LADDS_RELAY_PORT=17881`连接。

先在同ROS环境运行只读观测程序，再启动DDS端：

```bash
python3 scripts/validation/ladds_single_leg_observer.py \
  --output docs/validation/ladds-single-leg-20260915/gazebo-observer.json
```

```powershell
python scripts/validation/ladds_single_leg_peer.py `
  --library 'D:/自主航行设计文档/0. LaDDS说明/dds联调工具/LaDDS.dll' `
  --task-file docs/examples/ladds-0211-single-leg.json --domain-id 21 `
  --output docs/validation/ladds-single-leg-20260915/dds-peer.json
```

绘图：

```bash
python3 scripts/validation/plot_ladds_single_leg.py \
  --input docs/validation/ladds-single-leg-20260915/gazebo-observer.json \
  --output docs/validation/ladds-single-leg-20260915/trajectory-speed.png
```
