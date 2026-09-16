# 移除毫米波雷达后的 LiDAR 导航验证

2026-09-14（Asia/Shanghai）完成：在 `codex/distributed-navigation` 的未提交状态上移除毫米波雷达功能。本记录对应本轮源码及独立安装区，修改未提交。基准 HEAD 为 `81ab8ed1a228a23f8c1f0a1c071ef4acdfbe34dd`，不能仅凭该提交号还原验证源码。

删除前已将 687 个源码、脚本和根配置文件复制到 `/home/usv/navigation-lidar-only-20260913/source-before`，并核验复制后的 SHA-256。[快照清单](source-before-manifest.json) 和 [本轮源码差异](source-delta.json) 用于区分已有未提交工作与本轮修改：清单中 621 个文件字节一致、26 个修改、40 个删除。新增验证材料及历史文档提示不在该源码清单内。

## 清理范围

| 范围 | 本轮结果 |
|---|---|
| 专用功能包 | 删除 `mmwave_perception`、`usv_radar_costmap`、`usv_radar_interfaces`，当前为 [17 个功能包](packages.txt) |
| 普通导航与已有任务入口 | 删除毫米波订阅、健康状态、雷达地图前置检查和 `WAITING_SENSOR` 分支 |
| 配置与启动 | 删除 `obstacle_sensor`、`mmwave_config` 切换入口、两个 `sensor_configuration.py`、毫米波 YAML、xacro 和 RViz 配置；固定使用原 LiDAR 配置 |
| 参数 | 删除 `navigation.require_radar` 与 `navigation.radar_timeout`；普通导航解析器拒绝再接收这两个参数 |
| 测试与工具 | 删除毫米波专用测试和四个验证脚本；清理 12 个对应旧 `.pyc` 缓存；保留用于验证旧包不可发现及旧参数被拒绝的负例 |
| 历史材料 | 历史验证归档保留；旧毫米波使用文档增加历史提示，当前 README 指向 LiDAR 安装区 |

现有感知定位、决策规划、航行控制的进程划分保持。感知定位使用仿真定位、仿真 LiDAR、mapping 和 costmap；航控继续读取地图。路径规划、轨迹规划、控制器及碰撞检查算法保持，原任务替换、取消和重启边界保持。本轮没有接入真实传感器。

## 实际验证结果

以下构建和运行命令的终端退出码均为 0。

| 检查 | 实际结果 | 证据 |
|---|---|---|
| 独立目录完整构建 | 17 包完成，6 min 13 s | [构建日志](build.log) |
| 删除参数的回归测试 | 修改前 2 项按预期失败；修改后参数文件 21 项通过 | [修改前](removal-red.log)、[修改后](removal-green.log) |
| 全源码与 ROS 测试 | 869 passed，127.70 s；包含真实安装区测试 | [日志](source-tests/tests.log)、[JUnit](source-tests/tests.xml)、[实际命令](source-tests/command.json) |
| 原算法与配置保留性 | 47/47 通过 | [源码/AST 对比](preservation.json) |
| 干净安装区审计 | 49/49 通过；三个已删除包在 ament 中均不可发现 | [安装区审计](install-audit.json) |
| 实际分进程联调 | 12/12 通过；唯一推进器发布者、轨迹传输、取消、断流和重启 | [联调结果](process-smoke/summary.json) |
| 实际 YAML 加载与热更新 | 4/4 通过；两个节点各核对 8 个参数；航控实际读回热更新值 4.75 | [参数结果](yaml/summary.json) |
| 导航及故障回归 | 9/9 通过，`all_passed=true`、`run_finished=true` | [总结果](gazebo/summary.json)、[场景与阈值](gazebo/manifest.json) |

导航回归包含 PID、LQR 各运行直航、转向、单障碍场景一次，共 6 次实际 headless Gazebo 闭环；另有 Gazebo 中的目标替换和取消，以及使用可控状态/地图发布者的 odom 断流检查。最后一项不使用 Gazebo。断流停推实测 0.435 s，阈值 0.7 s，三路输出在观察窗口内持续为零。地图断流及三模块重启由独立分进程联调覆盖，地图断流停推实测 1.516 s，阈值 1.9 s。任一模块重启后旧任务失效，重新下发目标后恢复。

47 项保留性检查覆盖 common、controller、path_planner、trajectory_planner、perception 的 40 个 Python 源文件字节比较，以及 6 个规划/时效定义的 AST 比较；配置文件只允许删除上述两个默认项，其余完整 AST 相同。快照差异中，YAML 文件仅删除了两份毫米波配置，保留的原有 YAML 均未改动。人工复查确认取消请求及零推力 ACK 的并发处理没有变化。

全源码回归沿用既有导出测试夹具的 `PYTEST_ADDOPTS='-o testpaths=src/common/test'` 环境。外层仍显式收集 `src scripts`，没有排除外层测试；设置仅作用于旧导出测试的子 pytest 默认发现路径。该既有夹具问题见[原始记录](../distributed-navigation-001/preexisting-export-test/findings.md)。

构建中三个原有 Gazebo 包输出 CMake/编译警告，但全部成功完成。此次结果是同机 ROS 2 Humble / WSL Ubuntu 22.04 的仿真回归；未据此验证实船硬件、跨机器网络、GUI 画面或统计性能改进。

## 本机启动

在新 WSL 终端加载本轮独立安装区：

```bash
source /opt/ros/humble/setup.bash
source /home/usv/navigation-lidar-only-20260913/install/setup.bash
ros2 launch sim_env main.launch.py headless:=true
```

分模块入口及话题见[三模块普通导航](../../distributed-navigation/README.md)。旧毫米波 `install` 目录作为历史产物保留；当前验证没有 source 旧工作区，安装审计也会拒绝旧雷达包仍可被发现的环境。

本轮构建命令，从源码仓库根目录执行：

```bash
source /opt/ros/humble/setup.bash
colcon --log-base /home/usv/navigation-lidar-only-20260913/log build \
  --base-paths src \
  --build-base /home/usv/navigation-lidar-only-20260913/build \
  --install-base /home/usv/navigation-lidar-only-20260913/install \
  --executor sequential --event-handlers console_cohesion+ \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON
```

复验时为输出选择新的绝对目录，保持此次原始证据不被覆盖。测试入口为 `scripts/validation/run_distributed_source_tests.bash`；安装审计、保留性比较、分进程联调、YAML 和场景回归分别由同目录下的 `verify_distributed_install.py`、`verify_navigation_preservation.py`、`distributed_smoke.py`、`distributed_yaml_probe.py`、`ros_acceptance.py` 执行。实际运行参数已保存在对应 JSON 证据中；保留性比较使用本记录开头的删除前快照。
