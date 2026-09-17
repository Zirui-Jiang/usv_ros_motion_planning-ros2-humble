# 毫米波视场边框 TF 修复

2026-09-12，针对 RViz `Mmwave field of view` 不稳定显示。修复已增量安装到 `/home/usv/usv_migration/mmwave-sim-001/install`。重新启动对应毫米波 launch 后生效。

原因：边框 MarkerArray 使用扫描源戳且 `frame_locked` 默认为 false。扫描与船体 TF 独立到达；RViz Humble 的 MarkerArray 直接进入处理队列，在源时刻 TF 尚未到达时，LineMarkerBase 会隐藏线条。代码依据为官方 [MarkerBase](https://raw.githubusercontent.com/ros2/rviz/humble/rviz_default_plugins/src/rviz_default_plugins/displays/marker/markers/marker_base.cpp)、[LineMarkerBase](https://raw.githubusercontent.com/ros2/rviz/humble/rviz_default_plugins/src/rviz_default_plugins/displays/marker/markers/line_marker_base.cpp)。

修复仅将几何视场边框设为 `frame_locked=true`。边框每次渲染按最新安装坐标系变换；检测列表和点云的源时间、雷达障碍层的源时刻投影均保留。Marker 的 1 s 有限寿命保留，传感器持续无新数据时仍可过期。

验证结果：

- 新回归测试用真实 tf2 Buffer 构造 TF 为 10.0 s、扫描为 10.05 s 的场景。修复前测试因 ExtrapolationException 失败；修复后可取得最新安装位置，并随下一次 TF 更新移动。
- `pytest -q src/core/mmwave_perception/test src/sim_env/test/test_radar_navigation_ros.py`：32 passed，4.10 s，包含原有采样、空帧、断流、时钟重置与导航门控测试。
- 独立 ROS 域 194 和独立 Gazebo master 下，25 s 观察收到 279 帧视场 Marker，均已 frame locked；TF 建立后按新方式查询成功 279/279。同一批消息按旧源戳立即查询仅成功 4/279，275 次为 ExtrapolationException。原始结果：[runtime/probe.json](runtime/probe.json)。
- 探针的最长主机接收间隔为 2.295 s；这是主机时间统计，不能直接代替 Marker 使用的仿真时间寿命。该测试验证 TF 显示前提，不声称像素级持续渲染已验收，也未将延长寿命用于掩盖真正的传感器断流。
- 增量构建完成，证据为 build.log。仿真进程只用于验证并已清理，结束时未留后台仿真。

只读探针 `scripts/validation/mmwave_fov_probe.py` 可在同一 ROS 域观察当前仿真，不发送目标、不修改节点、不关闭进程。`run_smoke.py` 是本次隔离验证入口，结果目录存在时拒绝覆盖。

本修复在原 `mmwave-sim-001` 验收之后发生；原验收的源码校验表仍代表修复前版本，本目录的 source-sha256.json 记录此次修改文件。
