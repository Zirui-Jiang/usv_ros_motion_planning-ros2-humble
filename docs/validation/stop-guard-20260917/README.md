# 0331 主动刹停 / 0211 终点值守验证记录

日期：2026-09-17。验证对象是本地工作区源码，未提交 Git，未部署到实船。
原有规划器、仿真和交付材料等未提交工作保留；本记录不为那些其他改动单独背书。

## 已实现的业务边界

- 0331 使用独立协议入口和 `StopNavigation` 服务，接受空 JSON 内容，不要求位置、艏向或时长；主动反推，连续低速确认后归零，不保持停止位置，也不恢复旧任务。
- 0211 `taskType=80` 执行航路，最后一个航点到达后进入持续微控；中心始终是任务终点，不是接管瞬间的船位。`taskType=16` 保留普通航路完成流程。
- `guardingHeading` 已由用户确认为真北顺时针度数；null/缺失不约束固定艏向，0 明确表示北向。内部接口转换为地图坐标系的 ROS yaw 弧度。
- 不适配区域或 `guardingRadius`。类型 80 携带相关非空内容会告警，不会把区域当作安全约束或把半径当作保持精度。
- 类型 80 首次确认终点到达并进入值守时发送一次 0052，但仍保留活动的微控任务；重复状态和旧回包不重复发送。0331 刹停不发送 0052。新的航路等待停止确认及先前在途停止/取消调用消退；迟到旧任务不得复活。

协议示例、两侧启动配置和限制见
[通信说明](../../../src/core/communication_module/usv_ladds_bridge/README.stop-guard.md)。

## 本轮实际执行结果

以下为终点值守增加一次 0052 反馈之后的重新构建与回归结果。

| 环境/检查 | 结果 |
| --- | --- |
| WSL Ubuntu 22.04 / ROS 2 Humble，独立 colcon 安装区 | 10 个相关包全部构建成功 |
| Linux 主回归：控制算法、控制节点、决策节点、共享导航、通信适配器 | 509 passed，1 skipped，75.01 s |
| 独立进程的普通航路与值守反馈 ROS 集成回归 | 4 passed，4.79 s |
| Windows Python 对同一组包的可执行回归 | 390 passed，13 skipped，2.68 s |
| `git diff --check` | 通过 |

Linux 唯一跳过项是 `test_close_releases_dependency_search_directory_after_native_handle`，
依赖 Windows 专属 `os.add_dll_directory`。Windows 不能运行的 ROS 等检查不计作通过。
两套平台的通过数存在重叠，不能相加作为独立用例总数。

构建关闭了 Python 字节码编译，因此 colcon 提示 8 个包有 stderr；内容为
`byte-compiling is disabled, skipping`，无构建失败。

本轮 Linux 临时安装区为 `/tmp/usv-stop-guard.q4pb9h`，原始 JUnit 记录为
`all-tests.xml` 和 `route-integration.xml`。临时目录不属于交付安装区，系统清理后可用以下命令复现。

## 复现方法

WSL 中，在仓库根目录：

```bash
bash scripts/validation/run_stop_guard_checks.bash all
```

该脚本将相关源码复制到新的 ASCII 临时目录，构建接口和依赖，绕开 Humble rosidl
对中文源码路径的限制。主测试使用域 86，单独的航路适配器集成测试使用域 85 和唯一命名。
全部仅使用本机发现及人工输入，不覆盖原有导航安装目录，不启动 Gazebo，不连接 LaDDS SDK 实例。

Windows 中，在仓库根目录：

```powershell
python -B -m pytest -q -p no:cacheprovider src/core/control_module/controller/test src/core/control_module/usv_control/test src/core/decision_module/usv_decision/test src/core/shared/usv_navigation/test src/core/communication_module/usv_ladds_bridge/test
```

## 关键覆盖

- 正向航行反推、反向航行正推制动；偏航制动、横向仍有速度时不误报停止；没有速度积分引发持续倒航。
- 低速确认依赖连续源时间观测：重复时间戳不累计、时间倒退或超过 0.5 秒观测空隙重新计时。
- 同一简化动力学模型中，主动制动相比零推力滑停使用更短时间/距离；不外推为实船最优刹停性能。
- 0331 无定位参数；前一任务或停止回包迟到、停止重试、决策/控制/感知重启不会让旧任务直接恢复。
- 中间航点不得提前进入值守；最终点进入 HOLD 后保持同一目标和任务，旧参考不能重新移动保持中心。
- 每次值守执行只生成一次到达反馈，反馈仍为 `{"travelId": ..., "taskFinish": 1}`；不清除活动任务或发 COMPLETED 通知。首次 GUARDING 状态未收到时，可从同任务锁存的 HOLD 模式确认此前到达；替换中的旧任务不反馈，相同航路重新执行仍需取消旧值守并获得自己的一次反馈。
- 可选艏向微控、有艏向微控、0 与 null 区分；实际 DecisionNode + ControlNode 通过 ROS 服务/话题进入 HOLD，人工位移产生纠偏输出，随后停止到零并不再回点。
- 参考续租不覆盖实际 WAITING_MAP/WAITING_REFERENCE 等安全状态；地图/状态失效、障碍和命令过期时不能伪称已停稳。
- SDK 替身及 loopback TCP 验证双主题分流，不依赖报文形状猜测 0211/0331。

## 未验证及上线前必做

没有执行真实 LaDDS 双主题联调、Gazebo 完整船体闭环、实艇反推试验、风浪流扰动试验，
也没有测量真实刹停距离、制动时间、终点误差或抗流能力。
“一次反馈”指适配器对每次执行只发布一次，不代表新增了上位机确认、断线补发或跨进程重启的可靠送达协议。

初始限幅 0.4、停稳阈值 0.05 m/s 与 0.02 rad/s、位置容差 0.5 m、艏向容差 5°
均为现有模型/仿真起点，不是实艇参数或保证。双纵向桨没有侧推，不能承诺任意横流下
位置和艏向绝对静止；软件通信和安全检查也不能替代硬件急停。

上线前必须统一重编译新 ROS 接口和依赖，确认 Windows/ROS 两侧均订阅 0331，
先在隔离闭环仿真核对方向、限幅、制动尾迹和连续停稳确认，再按企业安全流程验证实船。
