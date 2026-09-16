# 模块心跳：以进程会话与单调序号识别在线状态和重启，接收端使用本地墙钟计算新鲜度。
# 角色名用于路由到 decision、control 或 perception 对端记录。
string module
string session
# 同一 session 的序号必须递增；重放或乱序不刷新存活时间。
uint64 sequence
# 进程是否准备就绪，reason 可说明未就绪原因。
bool ready
string reason
