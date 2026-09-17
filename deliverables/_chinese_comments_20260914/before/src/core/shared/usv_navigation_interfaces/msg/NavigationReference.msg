# Snapshot, not a collision permission. Empty points pause an active task.
std_msgs/Header header
string planner_session
string control_session
string perception_session
uint64 task_id
uint64 revision
uint64 sequence
bool active
geometry_msgs/Pose2D goal
string state
string reason
# Validated effective project configuration. Only the existing HOT keys may change.
string parameters_json
# Original planning source time, never refreshed by retransmission.
float64 trajectory_stamp
string trajectory_frame
usv_navigation_interfaces/TrajectoryPoint[] points
