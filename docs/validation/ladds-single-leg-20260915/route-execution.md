# 0211 同航线重新执行修正

## 确认的约定

2026-09-15，用户确认：发送端每次下发只发一次，重复下发表示重新执行航线；航行中收到新0211时，结束当前执行，从当前位置执行新航线。协议文档将`travelId`定义为观光航线编号，0052回传此编号。重复下发触发重新执行的约定来自用户确认，不是从通用报头字段推断。

## 实现

- 每条合法0211产生独立UUID作为内部`request_id`，不以航线编号或内容摘要去重。
- 航行中收到新指令时，先确认旧提交是否已接受，再调用`cancel_navigation`；只有收到成功的零输出确认才提交新目标。取消期间再来新指令，保留最新尚未提交的候选。
- 服务超时重试保留同一内部ID，最多重试3次；未知提交或未知取消结果不能被新指令越过。
- 0052只由当前执行的匹配到达状态触发一次，被替换执行及其迟到回调不产生完成反馈。
- 新决策会话先返回服务接受、后发布状态时保留接受关联；已退休会话的迟到状态和响应被忽略。
- 保留单点航渡子集：`taskType=16`、`count=1`、点类型1。`taskType=0`修改航路尚未支持。坐标换算和节到m/s换算保持原实现。
- 本次修改适配器和验证工具，没有修改NavigateTo接口、导航规划器、控制器或轨迹预测时长。

## 验证结果

| 检查 | 结果 |
|---|---|
| WSL完整桥接回归、ROS交互、启动测试及导航服务回归 | 159通过，1个Windows专属测试跳过 |
| Windows桥接与测试发送器回归 | 131通过，5项ROS依赖测试/测试模块跳过 |
| colcon构建usv_ladds_bridge | 成功 |
| 安装内容核对 | `leg_state.py`和`single_leg_node.py`均链接到测试过的仓库源码 |

WSL结果：[JUnit记录](route-execution-tests.xml)。可在项目根目录复跑：

```bash
source /opt/ros/humble/setup.bash
source /tmp/usv_ladds_single_leg_20260915/install/setup.bash
export ROS_DOMAIN_ID=86 ROS_LOCALHOST_ONLY=1
/usr/bin/python3 -m pytest -q \
  src/core/communication_module/usv_ladds_bridge/test \
  scripts/validation/test_ladds_single_leg_peer.py \
  scripts/validation/test_ladds_route_execution_ros.py \
  scripts/validation/test_ladds_startup_ros.py \
  src/core/decision_module/usv_decision/test/test_decision_ros.py
```

真实rclpy交互测试使用域85和随机唯一话题/服务名，测试取消响应前后的提交顺序、同JSON新UUID、同航线新速度及0052关联。其导航服务和到达状态由测试夹具控制，验证ROS交互与状态处理，不代表Gazebo或实船实际到达。本次未向用户的DDS域1发送测试航线，未重启用户的仿真或中继；新行为还需在用户工具和Gazebo运行中复验。

历史的68包只执行一次、重复0211补发0052等数据保留在原报告中，仅说明旧版本行为。独立DDS测试发送器现已改为发送一次后等待反馈；完整航路观测脚本也按一次输入、一次执行验收。

## 本机加载修复

安装目录已更新为`/tmp/usv_ladds_single_leg_20260915/install`。在已有ROS桥接终端按Ctrl+C，然后执行：

```bash
cd '/mnt/d/ROS2轻量仿真系统/usv_ros_motion_planning-ros2-humble'
export ROS_DOMAIN_ID=42 ROS_LOCALHOST_ONLY=1
bash scripts/start_ladds_single_leg.bash
```

Windows中继和Gazebo可保持运行。工具使用域1、主题0211、单次发送，保留实际`travelId`。再次发送就是重新执行，循环发送会持续触发取消和替换。若目标已经位于到达容差内，新执行可能很快完成；重新执行不会让船自动返回原出发点。
