# 控制执行报告：携带实际桨命令、当前任务身份和撤销/完成信号，用于决策状态与取消确认。
std_msgs/Header header
# 控制和规划会话标识进程实例，拒绝重启前的报告。
string control_session
string planner_session
# 任务号和所执行参考序号共同关联一次取消或到达事件。
uint64 task_id
uint64 reference_sequence
string state
string reason
# 请求决策端撤销当前参考；不同于任务已经完成。
bool revoke_reference
bool task_finished
bool clock_reset
# left/right/lateral 为实际发布的归一化命令，合法范围 [-1,1]。
float64 left
float64 right
float64 lateral
# JSON 字符串携带计时分位数、事件计数与进程资源统计。
string metrics_json
string counters_json
string process_json
