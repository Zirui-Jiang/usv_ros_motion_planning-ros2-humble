# USV 任务模块

`usv_mission` 位于 `core`，负责任务下发、参数校验、任务状态机、执行切换和控制权协调。
目前包含待机值守、绕圆值守和阵位保持；后续任务可在该模块内扩展。
保持控制器、绕圆轨迹与数值算法仍分别属于现有 `controller`、`trajectory_planner`、`common` 包。
本次迁移不重设计控制器公共接口，也不改变船体、推进器和 Gazebo 物理参数。

## 构建与启动

在 ROS 2 Humble 环境构建并加载工作区：

```bash
colcon build --packages-up-to usv_mission
source install/setup.bash
```

完整仿真入口由任务模块组合现有资源与支持节点：

```bash
ros2 launch usv_mission mission_sim.launch.py world:=ocean
ros2 launch usv_mission mission_sim.launch.py world:=ocean headless:=true
```

该入口直接启动 Gazebo、现有船体描述、定位、LiDAR/costmap 及 `mission_node`。
它不会启动旧 `sim_env_node`，以保证同艇由任务模块统一发布左右推进器指令。
**不要同时运行向同一艇发布推力的旧导航入口或其他控制节点。**

已有外部仿真或艇端 odom、TF 和 costmap 时，仅启动任务节点：

```bash
ros2 launch usv_mission mission.launch.py robot_namespace:=otter \
  use_sim_time:=true world_path:="$(ros2 pkg prefix --share sim_env)/worlds/ocean.world"
```

这个入口不启动 Gazebo、感知或定位。也可用 `geo_reference:=/absolute/path/origin.yaml`
代替 `world_path`，其内容如下；两种参考来源不能同时指定：

```yaml
latitude: -33.7242
longitude: 150.68
altitude: 0.0
```

参考必须对应外部状态与地图所在的 WGS84/ENU、零地图艏向坐标系。
未给参考时地理任务保持禁用，不会把默认 `(0, 0)` 当作有效原点；非法显式参考会阻止启动。
组合仿真入口从实际选定的 world 读取参考。

## 任务调用

```bash
ros2 run usv_mission mission_cli start "$(ros2 pkg prefix --share usv_mission)/config/mission/examples/standby.yaml"
ros2 run usv_mission mission_cli start "$(ros2 pkg prefix --share usv_mission)/config/mission/examples/orbit.yaml"
ros2 run usv_mission mission_cli start "$(ros2 pkg prefix --share usv_mission)/config/mission/examples/station_keep.yaml"
ros2 run usv_mission mission_cli status
ros2 run usv_mission mission_cli cancel
```

服务类型仍为 `usv_mission_interfaces/srv/SetMission`；默认服务 `/otter/set_mission`、
取消 `/otter/cancel_navigation`、状态 `/otter/navigation_status` 保持兼容。
新 ROS 节点名为 `/otter/mission_manager`。
非默认艇使用 `mission_cli --namespace boat_two start task.yaml`。
新请求被拒绝时保留原任务；收到超时先查询状态，避免不确定结果下重复下发。

任务输入与算法/安全调参分开。`config/mission/examples` 的参数数值保持迁移前版本；
示例中心对应 ocean 世界约 `(23, -10)` m，更换参考后需重新换算。
`mission_config` 来自本包；四个算法 YAML 默认只读复用 `usv_navigation/config` 的共享资源，亦可通过
`dynamics_config`、`controller_config`、`trajectory_config`、`planner_config` 显式指定路径。
组合入口保留 `world`、`robot_namespace`、`robot_x/y/z/yaw`、`gui`、`rviz`、`headless`、
`use_sim_time`和算法选择参数，传感器链路固定使用 LiDAR。

## 职责边界

任务模块按当前已验证算法协调导航、保持和避障，并维持任务替换、取消、状态过期与
控制指令切换的一致性。`sim_env` 提供仿真资源与定位节点，`usv_perception` 提供建图与代价地图节点；普通导航可独立使用，主导航入口不启动任务节点。
任务节点不继承 `Simulator`、不调用其私有启动函数；运行时通过 ROS 输入输出与环境连接。
不同艇型的统一航控接口及新的控制器设计属于后续独立工作，不在本次迁移范围。
