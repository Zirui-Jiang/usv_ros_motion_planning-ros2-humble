# USV ROS 2 Python 导航重构

本分支在原 ROS 功能包内实现 Python 导航、控制、轨迹与 LiDAR 栅格建图。
Gazebo、水动力、推进器、传感器及 RViz 插件继续复用原 C++ 工程。
原作者代码保存在 `main` / `cpp-baseline`；当前开发分支为 `refactor/python-navigation`。

当前默认算法为 **A* → LOS → PID**，也可选择 Dijkstra、GBFS 和 LQR。
算法类通过 Python 导入和对象调用连接；ROS 节点负责输入、参数、TF、调度及发布。
LiDAR 累积地图与 Nav2 实时代价地图分别工作，当前没有启用 static layer。

| 文档 | 内容 |
| --- | --- |
| [工程使用指南](docs/python-navigation-guide.md) | WSL 构建、原包分工、launch、YAML 兼容与在线调参、回退 |
| [实施与验收状态](docs/python-migration-status.md) | 区分静态检查、单元测试、ROS 集成与闭环验收 |
| [迁移差异](docs/python-migration-differences.md) | 原行为、已修复缺陷、未迁移能力和比较边界 |
| [自动验收入口](scripts/validation/README.md) | 固定场景、重复试验、原始数据与性能指标 |
| [环境变更](docs/validation/environment-changes.md) | ROS/Gazebo 版本、依赖安装与备份记录 |

在完成独立工作区构建后，用 WSL 新终端加载 ROS 和该安装区：

```bash
source /opt/ros/humble/setup.bash
source /home/usv/usv_migration/python/install/setup.bash
ros2 launch sim_env main.launch.py headless:=true
# LQR：
ros2 launch sim_env main.launch.py headless:=true controller_name:=lqr
```

这两条 launch 是互斥示例，每次运行一个。去掉 `headless:=true` 可启动 Gazebo GUI
和 RViz；本机 WSLg 的初始化证据与画面验证限制见 [GUI 记录](docs/validation/gui/README.md)。
其他机器请将安装路径替换为自己的工作区路径。

源码回归入口：

```bash
bash scripts/validation/run_source_tests.bash
```

本次没有实现 AIS/Radar 融合、COLREGs、DWA 或 MPC。使用这些能力需要后续独立实现与验证。
各包原作者信息及原许可声明保留；原本标为 `TODO` 的声明仍沿用原记录，未在语言迁移中
擅自指定新的许可证。
