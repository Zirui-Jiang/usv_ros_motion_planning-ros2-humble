# 绿色轨迹长度核对

- 实际RViz启动配置的绿色显示：`/otter/local_trajectory`，颜色25/255/0，Buffer Length=1。红色显示为`/otter/global_path`。
- 决策进程17596的启动日志确认：LOS `trajectory_time=20.0s`、`trajectory_time_interval=0.5s`。
- 当前收到的任务60：count=1，目标纬度30.546127、经度114.3938，speed=4。适配器状态确认ENU目标(83.38367675,91.68293913)m，期望速度2.05777778m/s，并记录COMPLETED。
- 在速度维持4节且未提前减速/终止的情况下，20秒预测对应约41.16米累计路程。原1m/s、同样20秒时约20米。该计算说明当前配置应有的长度量级，不是从截图测得的长度。
- 轨迹规划器不直接解析0211；任务适配器先把GPS坐标和节转换成地图坐标、m/s，导航工作进程再生成LOS轨迹。
- 实时抓取期间仿真和决策进程已经退出，只收到桥的rx和任务60历史完成反馈，未收到global_path、local_trajectory、reference或odom，不能核实截图当帧的实际长度和方向。[抓取原始记录](green-path-snapshot.json)中路径计数0表示没有收到消息，不代表当帧轨迹长度为0。
- 新增只读脚本`ladds_path_probe.py`可在仿真运行时同时采集报文、路径、参考时域/速度和船位。没有修改生产规划参数或算法。

```bash
source /opt/ros/humble/setup.bash
source /tmp/usv_ladds_single_leg_20260915/install/setup.bash
export ROS_DOMAIN_ID=42 ROS_LOCALHOST_ONLY=1
/usr/bin/python3 scripts/validation/ladds_path_probe.py \
  --output /tmp/ladds-path-snapshot.json
```
