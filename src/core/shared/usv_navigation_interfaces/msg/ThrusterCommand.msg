# 航控发往推进器执行端的桨角/转速指令：由唯一推进输出口把归一化左右桨命令经推力分配器换算得到。
# header.stamp 为航控发布时刻（仿真时为仿真时间），frame_id 为船体坐标系。
std_msgs/Header header
# 0 为固定桨（桨角恒为 0）；1 为全回转横移，当前版本不发布。
uint8 mode
# 桨角单位弧度，0 指向船艏；转速单位 rpm，带符号，正值沿桨角方向产生推力。
float64 left_angle
float64 left_rpm
float64 right_angle
float64 right_rpm
