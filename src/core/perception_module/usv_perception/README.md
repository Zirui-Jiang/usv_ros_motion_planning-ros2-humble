# usv_perception 感知节点包

本包管理 LiDAR 建图 ROS 适配器、Nav2 costmap、感知进程身份监测及 TF 辅助入口。
整数证据建图与扫描队列算法仍由同级 `perception` 包提供；这里使用
`ament_cmake` 与 `ament_cmake_python` 同时安装 C++ 和 Python 节点。

独立启动命令：

```bash
ros2 launch usv_perception perception.launch.py
```

该入口只启动下列三个节点，默认命名空间为 `otter`。

| executable | ROS 节点名 | 职责 |
|---|---|---|
| `perception_mapping_node` | `mapping` | 按扫描精确时间查询 TF，维护和发布整数证据地图 |
| `costmap_support_node` | `costmap` | 管理 Nav2 costmap 生命周期和 LiDAR 障碍层 |
| `perception_monitor` | `usv_perception` | 监测 odom、costmap_raw、map 发布者身份并广播模块心跳 |

`fake_localization_node` 由 `sim_env` 的仿真支持入口管理。独立使用本包时，上游需要
提供里程计、LiDAR 扫描和 TF。`usv_tf_expander.py` 辅助入口仍可通过
`ros2 run usv_perception usv_tf_expander.py` 使用，普通导航入口不会重复启动它。

配置默认从本包 share 目录读取，可使用绝对路径覆盖：

```bash
ros2 launch usv_perception perception.launch.py \
  robot_namespace:=boat_two use_sim_time:=false robot_x:=23.0 robot_y:=-10.0 \
  mapping_config:=/absolute/path/mapping.yaml \
  costmap_config:=/absolute/path/global_costmap_params_otter.yaml
```

`mapping_config` 默认为 `config/costmap/mapping.yaml`；`costmap_config` 默认为
`config/robots/otter/global_costmap_params_otter.yaml`。两个文件保持原项目 YAML 格式，
不接受 ROS 参数转储格式。`robot_x/y` 默认 `23.0/-10.0`，用于覆盖建图原点偏移；
`use_sim_time` 默认 `true`。扫描重映射到命名空间下的 `sensors/lidars/lidar/scan`，
costmap 使用同一 LiDAR 坐标帧。只加载 YAML 明确列出的插件，并继续拒绝启用
`static_layer`，保持当前的实时 LiDAR costmap 行为。

感知监测只接收 `watched_topics` 和 `use_sim_time` 参数，其输入消息新鲜度仍由
导航消费者判断。感知入口不加载规划和控制的四份 YAML，也不启动任务节点。

加载 ROS Humble 和已构建工作区后可运行 `bash test/run_ros_tests.bash`，使用
独立 `ROS_DOMAIN_ID=173`。测试包括真实 ROS launch 动作、TF 精确时间、队列超时、
时钟回退、地图发布、只读参数和感知进程重启身份。算法与队列的纯数值测试、基准
脚本继续位于 `../perception/test`。
