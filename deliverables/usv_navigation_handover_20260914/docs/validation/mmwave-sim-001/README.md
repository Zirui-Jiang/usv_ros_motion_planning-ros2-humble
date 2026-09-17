# 虚拟毫米波雷达验证记录

执行日期：2026-09-12。环境：Windows 宿主机的 Ubuntu-22.04 WSL、ROS 2 Humble、Gazebo Classic 11.10.2、系统 Python 3.10。

当前未提交工作树构建于 `/home/usv/usv_migration/mmwave-sim-001`，包含现有任务功能及新增毫米波功能；未覆盖此前安装区。构建和测试依据当前源码，不是声称已发布某个固定提交。源码校验值见 `source-sha256.json`。

| 检查 | 实际结果 | 证据 |
| --- | --- | --- |
| 完整构建 | 17 包完成；后续雷达层及导航修正增量构建通过 | build.log、costmap-rebuild.log、sim-rebuild.log |
| Python / ROS 测试 | 686 passed，35.80 s | pytest.log |
| C++ / Nav2 测试 | 20 个 gtest，2/2 套件通过 | costmap-tests.log |
| 首次 Gazebo 短程验证 | 全部断言通过 | smoke-001/summary.json |
| 最终安装版本 Gazebo 验证 | 全部断言通过 | smoke-002/summary.json |

完整构建有既有波浪插件编译警告及 wave_gazebo 的 xacro 依赖探测警告，未导致构建失败；受控海面场景已实际成功加载。最终验收使用独立 ROS 域 193、随机 Gazebo master 端口及专属进程组，结束后只清理自身进程。原 LiDAR 仿真的 ros2、gzserver、导航、RViz 进程仍在运行。

最终 smoke 的前方箱体位于 (12, 0) m，任务目标 (5, 0) m。观测到 53 帧检测列表与点云逐帧对应，消息契约无违规，地图存在致命障碍与膨胀代价，旧 LiDAR 没有发布者。暂停毫米波处理进程并排空在途数据后，仍收到 49 帧原始扫描、2 帧代价地图，点云增加数为 0；地图源时间前进 2 s，三路推进指令保持为 0。恢复处理进程后，状态重新进入 NAVIGATING。

`stop_to_check_wall_s` 是测试等待后检查的时长，不是测得的精确停艇延迟。零推进指令也不等于有惯性的艇体瞬时停止。该验证覆盖传感器、地图、短程规划和断流恢复，不覆盖长距离任务完成、复杂动态避碰、雷达目标跟踪、Doppler、RCS 或海杂波效果。

Windows 纯 Python 全量运行中，既有 WSL 专用验收入口的 mock 测试受平台检查影响失败；最终以目标运行环境 WSL 中全部 686 项通过为准，未为掩盖平台差异修改原有测试。

复现操作见 [功能说明](../../mmwave-simulation.md)。两次运行使用相同受控世界，世界文件和 launch 日志分别保存在对应 smoke 目录。
