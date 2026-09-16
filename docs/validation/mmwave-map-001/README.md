# 毫米波地图显示修正与验证

日期：2026-09-12。环境：本机 WSL Ubuntu 22.04、ROS 2 Humble、Gazebo Classic、实际 RViz 窗口。

## 定位结果

用户报告启动日志中的 `otter/map` 坐标系不存在。原运行记录中，最后一次等待发生在 `1789205878.162653958`，costmap 于 `1789205878.663101674` 输出 `start`，RViz 于 `1789205879.210925547` 开始创建 800 × 800 地图。该次 TF 在启动后恢复，并非持续缺失。

毫米波模式停用原 LiDAR 的扫描建图节点。旧 RViz 配置仍保留名为 `map`、订阅 `/otter/map` 的显示项；启用它会出现 `No map received`。有效的雷达障碍图发布在 `/otter/costmap`，其坐标系仍为 `otter/map`。修正前的实际窗口同时显示旧项警告及正常绘制的雷达障碍图，见 [before.png](before.png)。

## 修正范围

- `src/sim_env/rviz/sim_env_mmwave.rviz` 移除无发布者的 `/otter/map` 显示项，将有效项命名为 `Mmwave obstacle map`，默认展开。
- `/otter/costmap` 保持 Reliable / Transient Local 订阅，Fixed Frame 保持 `otter/map`。
- 默认 Orbit 视距由 35 调整为 90，焦点由 `(23, -10, 0)` 调整为 `(38, -10, 0)`，便于看到船体和前方障碍。
- 将该 RViz 资源同步到 `/home/usv/usv_migration/mmwave-sim-001/install/sim_env/share/sim_env/rviz/sim_env_mmwave.rviz`；源文件与安装文件经 `cmp` 一致。仅显示资源更新，无须重新编译。

本次未修改定位 TF、雷达采样、障碍层、控制器或系统图形库。

## 实际运行证据

两次运行均使用默认 ocean 场景，船体初始位姿 `(23, -10, 0)`，独立 ROS domain 195、独立 Gazebo 端口；实际启动 RViz、关闭 Gazebo GUI，不发送导航目标。测试只回收自身进程组，运行结束退出码均为 0。

| 项目 | 修正前 | 修正后 |
|---|---:|---:|
| 测试墙钟时长 | 75 s | 60 s |
| OccupancyGrid 消息 | 64 | 52 |
| 含障碍的地图消息 | 64 | 52 |
| 非空点云消息 | 974 | 767 |
| `/otter/map` 发布者 | 0 | 0 |
| `/otter/costmap` 发布者 | 1 | 1 |
| 结束时 map → base_link TF 可用 | 是 | 是 |
| 结束时 map → mmwave_link TF 可用 | 是 | 是 |

修正后地图尺寸 800 × 800，分辨率 0.5 m；最后一帧有 56 个障碍值 100 栅格及 808 个值 1–99 栅格。实际检查 [after.png](after.png)，`Mmwave obstacle map` 显示 `Status: Ok`，雷达点迹和蓝紫色障碍代价图可见。截图验证与 ROS 数据检查分别进行，不能仅凭节点收到了地图判定像素绘制成功。

两次启动阶段均出现过短暂 TF 等待。报告中的 `latest.base_tf_error` 和 `latest.sensor_tf_error` 保存最近一次失败的文本，不会在后续成功时清空；当前状态请看 `tf_available_at_end`。报告未声称启动阶段所有 TF 查询均成功，也未验证实时性能或长期运行稳定性。

本机 RViz 仍输出一次 `active samplers with a different type refer to the same texture image unit` 初始化错误，但本次实际画面能绘制地图。此项日志未被修复，不作为本次不可视化的根因。

- [修正前配置](before.rviz)
- [修正前数据报告](before-run/summary.json) 与 [启动日志](before-run/launch.log)
- [修正后数据报告](after-run/summary.json) 与 [启动日志](after-run/launch.log)

## 复现

在项目根目录的 WSL 终端中加载毫米波安装区，使用尚不存在的输出目录：

```bash
source /opt/ros/humble/setup.bash
source /home/usv/usv_migration/mmwave-sim-001/install/setup.bash
python3 scripts/validation/mmwave_map_smoke.py \
  --output /tmp/mmwave-map-new --duration 60 --domain-id 195
```

脚本执行前检查测试域是否已有 `/clock` 或 `/otter/odom` 发布者；运行中输出 `GUI_READY` 后检查 RViz 实际画面。Windows 端可使用 `scripts/validation/capture_rviz_window.ps1 -OutputPath <新文件绝对路径>` 保存匹配的 RViz 窗口截图。

当前代价图只反映雷达视场内观测和有限时间内保留的障碍，并非持久累积的环境建图。
