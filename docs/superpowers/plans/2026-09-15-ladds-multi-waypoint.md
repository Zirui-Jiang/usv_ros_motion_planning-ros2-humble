# LaDDS 多必经航点连续执行 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development or superpowers:executing-plans. Steps use checkbox syntax for tracking.

**Goal:** 一次0211下发按数组顺序连续经过全部必经点，最后一点停车后回0052。

**Architecture:** 适配器整包解析并调用NavigateRoute；决策端持有实测航点进度和一个task_id；独立多点轨迹入口处理几何锚点与段速度。参考增加pass_through，以关闭中间点的终点停车判定；旧单点入口保持兼容。

**Tech Stack:** Python 3.10、ROS 2 Humble、NumPy、现有A*/LOS/PID、Windows LaDDS/TCP。

**Spec:** `docs/superpowers/specs/2026-09-15-ladds-multi-waypoint-design.md`

## Global Constraints

- 原顺序、全部必经；通过半径1.5m，可配置。最终位置1.5m、速度0.25m/s沿当前配置。
- 每条收到的0211都是新执行，替换需等待旧取消确认；同次服务重试保留ID。
- speed为驶向该点一段的节数，内部乘1852/3600；转弯与变速允许提前减速。
- 无法规划不能跳点；预测通过不能推进实测进度；所有点完成才0052一次。
- 不改用户现有预测时长、PID参数、地图原点、DDS域与端口。不另启控制入口。
- 所有源码修改仅在当前工作区；此工作区包含大量单点前置未提交工作，按文件分工保存，不整体reset或创建丢失前置改动的checkout。

## 固定协作接口

```text
RouteWaypoint.msg:
  geometry_msgs/Pose2D pose
  float64 desired_speed_mps
NavigateRoute.srv request:
  string request_id
  std_msgs/Header header
  RouteWaypoint[] waypoints
  float64 waypoint_radius
response: bool accepted, string reason, string planner_session, uint64 task_id
NavigationReference.msg: 新增 bool pass_through，默认false
```

纯规划输入`PlanningRequest`末尾增加：

```python
waypoints: tuple = ()  # 剩余必经点，每个(x,y,theta,speed_mps)
waypoint_radius: float = 1.5
```

空waypoints沿用原规划；非空调用多点规划入口。多点轨迹的`goal`在剩余点数大于1时为None，剩余一点时为最终坐标。

## Task 1: 协议整包和接口（worker A）

Files: `usv_ladds_bridge/route_task.py`、`single_leg_node.py`、桥接测试；`usv_navigation_interfaces/msg/RouteWaypoint.msg`、`srv/NavigateRoute.srv`、`msg/NavigationReference.msg`、`CMakeLists.txt`。

- [ ] 先写解析测试：三点位置/速度顺序、count不匹配、后续点非法、长度1/255边界、单点兼容。
- [ ] 运行测试确认旧实现不能解析多点。
- [ ] 新增不可变RouteTask与有序Waypoints。parse_route(text, origin)完整校验后返回；不改变旧parse_single_leg公共测试契约。
- [ ] 新接口按上表定义。桥接创建NavigateRoute客户端，构造全部点与1.5m半径；每包一个request_id。
- [ ] 保留当前取消、generation、session和UNKNOWN保护；收到导航进度只报告进度，不提前完成。
- [ ] 更新ROS夹具为NavigateRoute并检查同包一次提交、各段速度、取消后替换、只完成整条0052。

验收示例：

```python
route = parse_route(json.dumps({'travelId':58, 'taskType':16, 'count':3,
    'points':[{'type':1,'latitude':30.5454+i*.0001,
               'longitude':114.3933,'speed':s} for i,s in enumerate([2,4,2])]}), origin)
assert len(route.points) == 3
assert [p.speed_knots for p in route.points] == [2,4,2]
```

## Task 2: 多点几何与连续轨迹（worker B）

Files: `trajectory_planner/route_trajectory.py`及测试、`usv_decision/runtime.py`及多点worker测试。保留旧LosTrajectoryPlanner.plan行为。

- [ ] 写失败测试：直线中间点速度非零、90度弯提前减速、不同段速度连续、仅最后一点终端停车、锚点不可达返回失败。
- [ ] PlanningRequest按固定接口扩展；非空waypoints进入独立路线规划函数。
- [ ] 逐段使用A*并验证精确航点连接，锚点不被跨段裁剪；规划总耗时受现有预算限制。
- [ ] 新rollout把预测索引与实测输入索引分离，保留航点圆盘约束；曲率/航向变化限速，按模型加减速范围平滑速度。
- [ ] 只最终目标停车；相邻重复点不会除零或死循环。完整船体扫掠检测保留。
- [ ] 纯测试和原LOS/worker回归通过，提供实际运动学闭环验证以避免仅检查预测轨迹。

验收调用边界：

```python
request = PlanningRequest(version=1, goal=(10.,0.,0.), state=state,
    grid=grid, route=np.empty((0,3)), parameters=cfg,
    waypoints=((10.,0.,0.,1.),(20.,10.,0.,.5)), waypoint_radius=1.5)
result = plan_navigation(request)
assert result.status == AlgorithmStatus.SUCCESS
assert result.trajectory.goal is None  # 实测还处于中间点阶段
```

## Task 3: 实测进度、决策生命周期与参考（parent）

Files: `usv_decision/route_progress.py`、`decision.py`及测试；`usv_navigation/protocol.py`及测试。

- [ ] 写纯进度失败测试：按序进入1.5m范围、正常样本线段穿越、拒绝时间倒退/大跳变、回到起点/重复点不会跳序。
- [ ] RouteProgress保存(x,y,theta,speed)元组、index、radius；update(state)仅使用新鲜合法实测；只有最后一个索引时允许控制器最终完成。
- [ ] 新NavigateRoute服务整包校验速度/坐标后一次接受，复用内部幂等窗口。单点NavigateTo继续工作。
- [ ] 真实进度推进只更新version与当前goal/speed，不调用_reset_task，不改变task_id；旧规划结果丢弃但保留仍有效的连续参考。
- [ ] 发布pass_through和航点进度；中间完成报告不能结束航线，最后完成保留整次request关联。
- [ ] trajectory_from_values增加可选pass_through=False；true时允许无terminal goal。旧调用缺省行为保持。

验收示例：

```python
progress = RouteProgress(((10.,0.,0.,1.),(0.,0.,0.,1.)), radius=1.5)
progress.update(VesselState(x=0.,y=0.,stamp=1.))
assert progress.index == 0  # 不能因为此时接近最后一点就完成
progress.update(VesselState(x=9.,y=0.,stamp=10.))
assert progress.index == 1
```

## Task 4: 集成、构建、Gazebo与交付（parent）

- [ ] 独立ROS域先构建接口，再验证决策/控制/桥接对新消息一致，旧单点接口兼容。
- [ ] 运行相关纯测试、真实ROS服务/控制参考回归、0211三点整包交互测试。
- [ ] 只在独立仿真域/端口启动有界测试并确保清理：三点直线、折线、2/4/2节；记录实测逐点最小距离、通过速度、终点停止、0052次数。
- [ ] 异常场景采用可控ROS/纯测试，验证无路不跳点、闭合路线、替换与旧状态。
- [ ] 更新用户启动脚本/文档和三点示例。新接口安装选择独立多点profile，避免运行中的旧进程混用消息定义。
- [ ] 独立代码审查后只修复明确问题，再运行受影响测试；最终报告真实通过与未验证范围。

测试命令采用Windows Python纯测试与WSL `/usr/bin/python3 -m pytest`；ROS测试使用独立域，不向用户域1发报文。Gazebo使用独立端口，退出清理自有进程。
