# perception 的 Python 建图算法库

本包保持原包名 `perception` 和纯 Python 算法接口，构建类型为 `ament_python`。
ROS 建图适配器、`perception_mapping_node` 和 `usv_tf_expander.py` 入口现由同级
[`usv_perception`](../usv_perception/README.md) 包管理；本包仅安装建图内核和扫描队列。

## 内核与 ROS 接口

`perception.mapping.GridMapper` 不依赖 ROS。`GridConfig` 定义正方形地图的
`width=1024`、`resolution=0.5`、`offset_x=0.0`、`offset_y=0.0`；
`data` 是形状为 `(width, width)` 的 NumPy `int8` 数组。`reset()` 将全部
单元恢复为 `-1`；`integrate_scan(ranges, angle_min, angle_increment,
range_min, range_max, pose_x, pose_y, pose_yaw)` 按束顺序积分，返回有效束、
无效束、命中束和栅格更新次数。

栅格采用整数证据分值：射线经过单元减 `25`，下限 `0`；命中单元加
`55`，上限 `100`。因此，未知单元第一次命中为 `54`。这不是经过概率标定
的 log-odds。数据更新保留逐束顺序，重叠单元不能用改变累积次序的批量赋值替换。

`usv_perception` 的 ROS 节点订阅相对话题 `scan`（LaserScan，best effort / volatile / depth 10），
兼容可靠和尽力发布端；发布相对话题 `map`（OccupancyGrid，可靠 / volatile /
depth 10）。地图帧为 `<namespace>/map`，根命名空间下为 `map`。除上述地图
参数外，可在启动时配置 `map_publish_frequency=2.0` 和
`tf_wait_timeout=0.2`。这些参数均为启动后只读，支持原 `mapping.yaml` 中的参数名。

节点按扫描自身 `header.frame_id + header.stamp` 查询地图帧下的 TF，不使用旧
TF 代替缺失的扫描时刻 TF。队列只保留最新 5 帧，按接收顺序处理；等待超时使用
单调墙钟，仿真暂停也会清除超时扫描。零时间戳、空帧名和相同帧内重复或过时
的扫描被丢弃。零时间戳不能用于这里的精确时间查询，因为 ROS TF 将 Time(0)
解释为最新变换。扫描积分和地图发布分别执行；重复发布地图不会再次积分扫描。

检测到 ROS 时钟回退时，节点清空队列、去重记录和 TF 历史，并将地图恢复为
未知。退出入口会清空待处理队列并销毁节点。日志每 5 秒报告累计积分次数、
超时/溢出/重复/非法输入丢弃数，以及处理耗时和排队时间。

## 与旧实现的明确差异

- `NaN`、负无穷、负距离和低于 `range_min` 的数据跳过；正无穷及有限且
  不小于 `range_max` 的读数清除至 `min(range_max, 30 m)`。
- 使用地图原点与 `floor` 统一世界坐标到栅格的转换；二维射线裁剪避免旧的一维
  索引截断造成跨行写入或边角伪障碍。离开地图的射线清除边界，不在边界标记命中。
- 每个扫描时刻最多积分一次，缺少准确 TF 的数据不积分；这些缺陷用例不要求与
  旧错误输出逐单元一致。

地图输出与 costmap 的激光障碍层是分别验收的链路。本包不修改或启用 costmap
的 `static_layer`，不增加 SLAM、目标跟踪或其他感知功能。

## 验证

在本包目录运行 `PYTHONPATH=. python3 -m pytest test -q` 可验证纯核和队列。
ROS 集成测试现位于 `../usv_perception/test`；没有 rclpy 时会明确跳过。
先加载 ROS Humble 和已构建工作区，再运行
`bash ../usv_perception/test/run_ros_tests.bash`，脚本使用独立 `ROS_DOMAIN_ID=173`，测试实际
消息传输、TF 精确时间、等待超时、时钟回退、地图发布和参数只读。

性能脚本为 `test/benchmark_mapping.py`，例如
`PYTHONPATH=. python3 test/benchmark_mapping.py --range-max 25`。
默认配置为 1024² 单元、0.5 m、2000 束、15 Hz、40 帧；射线采用接近最大量程
的长射线并混合无回波，`--range-max` 默认为 30 m 压力配置。该脚本测量内核，
不包含 TF 排队和 ROS 消息序列化。

2026-09-11 本机 WSL Ubuntu 22.04 / Python 3.10.12 的 25 m 用例实测：
P50 22.43 ms、P95 23.64 ms、最大 24.73 ms，低于 15 Hz 的 66.7 ms 周期预算。
Windows Python 3.12.10 的 30 m 压力用例 P95 为 24.19 ms。当前未引入 Numba；
部署硬件与整条导航链路仍应单独实测。
