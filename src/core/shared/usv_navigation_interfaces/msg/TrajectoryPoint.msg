# 一个时间参数轨迹点，字段顺序与纯 Python N×8 轨迹数组保持一致。
# Time is relative to trajectory_stamp, metres/radians/seconds throughout.
# 相对 trajectory_stamp 的时间，单位秒。
float64 time
# 累计路径弧长，单位米，允许停车段重复。
float64 s
# x/y 为轨迹坐标系位置，单位米；theta 为 ROS 平面航向弧度。
float64 x
float64 y
float64 theta
# 曲率单位 1/米，velocity 为米/秒，acceleration 为米/秒²。
float64 kappa
float64 velocity
float64 acceleration
