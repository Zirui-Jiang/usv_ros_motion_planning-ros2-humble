# 定点任务配置与命令行

使用 `ros2 launch usv_mission mission_sim.launch.py` 启动任务仿真后，在相同 ROS 环境中执行：

```bash
ros2 run usv_mission mission_cli start "$(ros2 pkg prefix --share usv_mission)/config/mission/examples/standby.yaml"
ros2 run usv_mission mission_cli start "$(ros2 pkg prefix --share usv_mission)/config/mission/examples/orbit.yaml"
ros2 run usv_mission mission_cli start "$(ros2 pkg prefix --share usv_mission)/config/mission/examples/station_keep.yaml"
ros2 run usv_mission mission_cli status
ros2 run usv_mission mission_cli cancel
```

每次 `start` 原子替换当前任务；被拒绝的请求保留原任务。非默认艇命名空间使用
`mission_cli --namespace boat --timeout 10 start mission.yaml`，全局参数放在子命令前。
服务 `/otter/set_mission` 的类型为 `usv_mission_interfaces/srv/SetMission`；
状态订阅 `/otter/navigation_status`；取消调用 `/otter/cancel_navigation`。
超时表示结果尚未确认，应先查询状态；任务拒绝与超时均返回非零退出码。

任务 YAML 必须提供 `mode`、`latitude`、`longitude`。坐标为 WGS84 经纬度，
航速单位 m/s，距离单位 m，`heading_deg` 以真北为 0°、顺时针为正。
可选默认值为航速 0.5、位置精度 0.5、值守半径 10、绕圆半径 10、顺时针、
圈数 0、总时长 0、艏向 0°。圈数及总时长为 0 表示不限，同时非零时先到者结束绕圆。
圈数只统计有效巡航，总时长从接受任务起按仿真时间累计；暂停仿真不累计时间。
绕圆示例配置三圈，完成后返中心待机。

示例经纬度来自 `ocean.world` 和 `example_course.world` 的相同地理原点，
对应本地中心约 `(23, -10)` m。更换 world 后必须重新转换；这些示例只定义任务输入，
不保证该位置、整圆或切入区域没有障碍。使用实际地图检查所选水域。

`mission_params.yaml` 为独立控制/安全配置，通过
`ros2 launch usv_mission mission_sim.launch.py mission_config:=/absolute/path/mission_params.yaml` 加载。
它以 `mission` 为唯一顶层键，与任务输入 YAML 分开；不接受 `mission.geo` 覆盖。
组合仿真入口从实际选定 world 的 WGS84 / ENU / 零 heading 配置读取原点。
仅任务节点入口 `mission.launch.py` 使用显式 `geo_reference` YAML 或 `world_path`，
不提供参考时地理任务禁用；详细格式见任务包 README。
若该参考缺失或不支持，地理任务不可接受，普通本地 `/otter/goal` 导航仍可使用。

源代码环境下也可使用 `common.geodesy.GeoReference.from_world(path).to_geographic(x, y)`
生成经纬度；`to_local(latitude, longitude)` 返回世界 ENU 坐标。
