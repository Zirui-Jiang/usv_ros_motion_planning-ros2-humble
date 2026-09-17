# 按业务模块整理功能包的验证记录

日期：2026-09-14。分支：`codex/distributed-navigation`。本轮基于未提交工作树整理，HEAD `81ab8ed1a228a23f8c1f0a1c071ef4acdfbe34dd` 不能单独标识本轮源码。

需求见[模块整理范围](../../distributed-navigation/module-package-organization.md)，使用说明和目录见[三模块普通导航](../../distributed-navigation/README.md)。本轮保持现有导航任务，不接入指定时间到达、指定速度航渡、定点值守或目标跟随任务。

## 基线与变化

修改前将 647 个源码/脚本/根测试配置文件保存到 `/home/usv/module-layout-20260914/source-before`，逐文件 SHA256 校验后再移动目录。比较依据是该快照，包含原先未提交的改动，不以 Git HEAD 代替。

三个新业务包分别为 `usv_perception`、`usv_decision`、`usv_control`，节点、launch 和专属配置归所属包；原算法包与节点包并列。公共支持移入 `shared/usv_navigation`；仿真定位、Gazebo 资源和主组合入口保留在 `sim_env`。四份共享项目 YAML 只有一份，感知拥有 mapping/costmap 两份 YAML。已有 mission 实现只适配资源路径，普通导航 main 不启动 mission。

## 验证结果

| 检查 | 结果 | 证据 |
|---|---|---|
| 全新构建 | 20 包完成，退出码 0，6 分 45 秒 | [构建日志](build/logger_all.log)、[构建事件](build/events.log) |
| 源码与 ROS 全集 | 最终 926 通过，0 跳过，退出码 0，120.05 秒 | [最终日志](source-tests-final/tests.log)、[JUnit](source-tests-final/tests.xml) |
| 安装区审计 | 76/76 通过，退出码 0 | [install-audit.json](install-audit.json) |
| 源码保留审计 | 98/98 通过，退出码 0 | [preservation.json](preservation.json) |
| 独立进程与重启检查 | 12/12 通过，退出码 0 | [process-smoke/summary.json](process-smoke/summary.json) |
| 独立 launch 的实际参数探针 | 4/4 通过，退出码 0 | [yaml/summary.json](yaml/summary.json) |
| PID/LQR 场景与故障回归 | 9/9 通过，退出码 0 | [gazebo/summary.json](gazebo/summary.json) |

干净构建从 `/opt/ros/humble` 开始，没有加载旧工程安装区。新安装路径为 `/home/usv/module-layout-20260914/install`。原仿真插件有构建 stderr 提示，涉及 `usv_gazebo_plugins`、`wave_gazebo`、`wave_gazebo_plugins`，均未导致失败；原始日志保留。

保留审计包含 46 份算法源码的字节比较、6 份 YAML 字节比较、完整配置 AST，以及节点、worker、共享心跳和运行函数的定义/常量比较。它证明指定实现保持，不单独作为 ROS 运行正确性的证据。

分进程检查使用合成 odom 与空 costmap，验证唯一推力发布者、完整轨迹传递、取消确认、迟到参考、输入断流，以及各模块重启后的目标失效和重新下发。YAML 探针通过新包独立 launch 启动决策和航控，读取实际参数服务，检查自定义文件、热更新与导出接口。

Gazebo 回归包含 PID/LQR 各自的直航、转弯、单障碍六个场景，以及目标替换和取消两个用例；第九项 odom 断流使用合成输入独立进程，不计作 Gazebo 场景。

首次源码全集与 Gazebo 使用不同 ROS domain 并行运行，旧参照测试 `test_process_planning_parameters_stale_inputs_cancel_and_clock_reset` 在第一次等待轨迹时超时，其余 925 项通过，见[首次日志](source-tests/tests.log)。保持代码、10 秒等待阈值和断言不变，单项复跑 1 通过（10.05 秒），见 [legacy-isolated.log](legacy-isolated.log)；随后在 Gazebo 结束后使用相同脚本串行复跑，926 项全部通过。失败时规划请求仍在产生，未报告 worker 异常；worker 使用 `spawn`，其定义、旧参照和该测试与迁移前相同。只读排查未找到确定的全局测试污染链。尚未定位这次瞬态失败的确切原因，不能仅凭复跑通过将其归因为负载，保留为测试稳定性待观察项。

最终源码身份和各组结果汇总见 [validation-summary.json](validation-summary.json)。[迁移前清单](source-before-manifest.json)与[整理后清单](source-after-manifest.json)记录文件哈希；工作树保持未提交。

## 复现命令

从仓库根目录，在新 WSL 终端执行。输出目录必须不存在；复跑时替换验证输出目录和构建根目录。

```bash
source /opt/ros/humble/setup.bash
colcon --log-base /home/usv/module-layout-20260914/log build \
  --base-paths src \
  --build-base /home/usv/module-layout-20260914/build \
  --install-base /home/usv/module-layout-20260914/install \
  --executor sequential --event-handlers console_cohesion+ \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON
source /home/usv/module-layout-20260914/install/setup.bash
export USV_DISTRIBUTED_INSTALL=/home/usv/module-layout-20260914/install
bash scripts/validation/run_distributed_source_tests.bash \
  /home/usv/module-layout-20260914/install \
  /mnt/d/ROS2轻量仿真系统/usv_ros_motion_planning-ros2-humble/docs/validation/module-layout-001/source-tests-final
python3 scripts/validation/verify_distributed_install.py \
  --install /home/usv/module-layout-20260914/install \
  --output /mnt/d/ROS2轻量仿真系统/usv_ros_motion_planning-ros2-humble/docs/validation/module-layout-001/install-audit.json
python3 scripts/validation/verify_navigation_preservation.py \
  --baseline /home/usv/module-layout-20260914/source-before \
  --source /mnt/d/ROS2轻量仿真系统/usv_ros_motion_planning-ros2-humble \
  --output docs/validation/module-layout-001/preservation.json
python3 scripts/validation/distributed_smoke.py \
  --workspace /home/usv/module-layout-20260914 --domain-id 217 \
  --output docs/validation/module-layout-001/process-smoke
python3 scripts/validation/distributed_yaml_probe.py --domain-id 218 \
  --output /mnt/d/ROS2轻量仿真系统/usv_ros_motion_planning-ros2-humble/docs/validation/module-layout-001/yaml
python3 scripts/validation/ros_acceptance.py \
  --workspace /home/usv/module-layout-20260914 --distributed \
  --controllers pid lqr --repeats 1 --faults \
  --scenarios-file scripts/validation/scenarios_grid_interior.json \
  --domain-id 216 --costmap-topic /otter/costmap_raw \
  --output docs/validation/module-layout-001/gazebo
```

源码回归保留原导出工作区夹具的 `PYTEST_ADDOPTS='-o testpaths=src/common/test'`。外层显式运行全部 `src scripts`，该设置只影响导出夹具里的子 pytest 默认收集，不减少本轮外层测试。

## 审查和适用边界

独立审查使用同一迁移前快照。包依赖无环，三个业务包的依赖闭包不含仿真包或其他业务包。审查发现进程清理脚本仍匹配旧感知安装路径；已补齐新包路径并完成定点复核。匹配样例测试修复前 6 失败/6 通过，修复后 12 通过；`bash -n` 通过，见 [cleanup-patterns-green.log](cleanup-patterns-green.log)。验证过程中没有执行该清理脚本，运行脚本只管理各自创建的进程。最后审查没有未解决的可行动问题。

初始开发阶段的分组测试使用新目录源码，部分 ROS 测试只借用了旧安装中的消息类型。这些结果不与本轮全集相加，不替代新安装区证据。历史 `lidar-only-001`、`distributed-navigation-001` 报告保留原样。

本轮验证范围是同机、ROS 2 Humble、LiDAR 仿真、普通导航。没有验证真实传感器、实船、跨机网络或甲方 DDS 适配；GUI 显示不属于本轮无界面回归。每种 PID/LQR 场景运行一次，用于组织调整的回归，不能用作统计性能或实船安全结论。
