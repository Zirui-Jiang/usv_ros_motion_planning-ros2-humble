# 决策发送的导航快照：绑定三方进程会话与任务版本，空轨迹点可暂停仍活动的任务。
# Snapshot, not a collision permission. Empty points pause an active task.
# header 表示本次发送时间/坐标系；轨迹年龄单独由 trajectory_stamp 计算。
std_msgs/Header header
# 规划、控制、感知三方会话必须与当前发现的实例匹配。
string planner_session
string control_session
string perception_session
# task_id 标识目标生命周期，revision 区分规划版本，sequence 标识快照内容更新。
uint64 task_id
uint64 revision
uint64 sequence
bool active
# 中间必经点为 true，控制器不得应用终点停车判定；缺省 false 保持单目标语义。
bool pass_through
# 空值与 NAVIGATE 均表示普通导航；BRAKE 不要求位置目标，HOLD 持续保持 goal。
string motion_mode
# 仅最终航段允许转入持续保持；艏向启用标志与零度艏向必须分开。
bool guard_on_arrival
bool guard_heading_enabled
# 当前 header.frame_id 下的 ROS yaw，单位弧度；disabled 时忽略。
float64 guard_heading
# 真实导航目标 (x米, y米, theta弧度)，完成判定不使用量化后的栅格终点。
geometry_msgs/Pose2D goal
string state
string reason
# Validated effective project configuration. Only the existing HOT keys may change.
# 完整有效参数快照，控制端只允许既有 HOT 参数与启动配置发生差异。
string parameters_json
# Original planning source time, never refreshed by retransmission.
# 轨迹生成所依据的源时间秒，重传不得刷新。
float64 trajectory_stamp
string trajectory_frame
usv_navigation_interfaces/TrajectoryPoint[] points
# Part of the global route near the vessel, in trajectory_frame; never a collision permission.
# 艇附近的一段全局航线折线，trajectory_frame 下的 x/y（米），两数组等长；航控据此求艇到航线的横向偏差。
# 为空表示本参考不提供航线，航控的横向环不出力。
float64[] route_x
float64[] route_y
