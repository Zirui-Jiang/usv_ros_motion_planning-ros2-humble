# 第一版接入执行记录

计划：docs/superpowers/plans/2026-09-14-0914-controller-integration.md（以顶部用户范围更新为准）。

- Ruling: 在用户指定的当前工作目录直接实施，保留 codex/distributed-navigation 分支及现有未提交工作；不以 HEAD 重建工作树，避免丢失当前未跟踪的正式模块。ROS 构建使用单独安装区。
- Ruling（已更正）: 用户明确“第一版”指完整五步计划；撤销助手此前原样接入和延期验证的理解。根目录源文件保持；正式核心实施确认的问题修正，执行参数匹配、真实 ROS / Gazebo 与对比。

| 任务 | 交付接口与依赖 | 状态 |
| --- | --- | --- |
| 核心与适配 | 四文件移植及局部修正；实际样本时间、生命周期、分配抗饱和 | 已实现、测试和审查 |
| 配置与启动 | adrc_controller.* 与适配层参数名称一致；独立 YAML | 已构建及安装审计 |
| 运行验证 | 使用上述工厂及共享配置，检查真实 ROS 消息与生产节点 | 数值、1050 项回归、3 次故障及 27 次矩阵全部通过 |
| 文档与审查 | 记录原版限制、具体命令和实际证据 | 代码/复现文档审查完成，证据和图表已归档 |

Windows 捆绑 Python 没有 pytest，首次测试命令在收集前退出（No module named pytest）；转用已有 WSL Python/ROS 环境。

更正前已完成的可复用部分：工厂/launch 注册、独立 YAML、标量映射及物理分配。系统 Windows Python 3.12 可运行 pytest；旧 PID/LQR 基线 34 passed，缩减实现的专项 80 passed。该结果仅是中间状态，不能代表完整计划通过。

| 共享边界 | 核验与裁定 |
| --- | --- |
| 核心 ↔ 适配 | 核心支持 measurement_stamp；缓存 non_integral_output、refresh_output()；heading integral_reset 防止分配冻结撤销清积分 |
| 适配 ↔ 配置 | max_sample_gap=0.5，默认原始限幅；专用 YAML 为 Otter 候选，数值/闭环验证后定稿 |
| 节点 ↔ 适配 | on_control_hold 默认空操作，ADRC 复位；重复状态保持命令而不更新历史 |
| 配置 ↔ 验收 | decision/control 使用同一全量参数；故障循环按所选算法；安装审计检查实际模块来源 |
| 构建 ↔ 运行 | 新源快照包含未跟踪正式文件；独立安装、ROS domain 与 Gazebo 端口，保留用户现有仿真 |

## 已完成的验证阶段

- 核心 TDD：23 failed / 7 passed → 33 passed；核心＋适配＋配置 148 passed。
- 独立审查修复：异步空参考、无效参考、配置不匹配即时停推时补 hold 通知，覆盖两次 timer 之间暂停又恢复的情况。
- 完整构建：初次 wave_gazebo 的 ERB 与 xacro 并行生成竞争失败；改本轮构建命令为 MAKEFLAGS=-j1，未改插件源码。第二次 20 包全部成功。
- 新安装区 `/home/usv/adrc-0914-20260914-mRBVAh`：83 项安装审计通过，包含实际核心来源及专用配置哈希。
- 完整 ROS 源码回归：修复新增 launch 夹具缺失 adrc 子配置后，1045 passed，记录 source-tests-002。旧失败 source-tests-001 保留。
- 原增益＋物理限幅候选：数值测试通过；Gazebo 直线通过（34.5 s，最大横向 0.353 m），转弯在 4.013 m 横向偏差触发边界停止；尚未运行的同轮单障碍不能记通过。
- 艏向 k_p=8 / k_d=4 候选：数值测试通过；Gazebo 转弯通过（40.4 s，最大横向 0.923 m），单障碍通过（42.0 s，最小保守间隙 1.089 m）。速度增益、艇模、LOS 参数和验收门槛保持。
- 专用 YAML 固定为上述艏向增益，等待最终 27 次对比与 ADRC 故障矩阵；DEFAULTS 仍保留来源参数。
- 断流验收补齐实际算法回读，并在连续停推证据之后恢复 odom，要求 NAVIGATING 状态和新收到的非零命令。

## 最终验证已完成

- 隔离工作区：`/home/usv/adrc-0914-20260914-mRBVAh`，20 包已构建；用户原有仿真仍位于 `/home/usv/module-layout-20260914/install`，不要停止或改写。
- 安装审计 `validation-results/install-audit-final.json`：83/83 pass。
- 故障 `validation-results/adrc-faults-final`：3/3 pass，goal_change、cancel、odom_dropout；取消零输出 0.003969 s，断流零输出 0.419674 s，输入恢复后 0.313062 s 恢复 NAVIGATING 与新非零命令。
- 新汇总测试在 ROS 旧 pytest 上的包名冲突已改直接 importlib 文件加载；完整回归 `source-tests-final-002` 已完成：1050 passed，36.06 s，无跳过。先前收集失败的 `source-tests-final` 保留。
- 最终矩阵 `validation-results/matrix-final` 已完成：27/27 passed、27/27 arrived、27/27 实际控制器身份一致。控制器顺序 adrc/pid/lqr，三场景各 3 次，ROS_DOMAIN_ID=199。源工作区/安装区运行期间未改，运行退出码 0。
- 最终参数已写入仓库与安装区专用 YAML：速度 kp8 ki0.8，总力±40、积分±30；艏向 kp8 kd4 ki0.03，力矩±10、积分±2，acceleration_gain0，window10/history50，maxgap0.5。
- 最终汇总见 comparison-final，PNG/SVG 见 figures-final，27 次原始日志见 evidence/matrix-final 及 matrix-evidence.tar.gz；阶段失败与故障日志见 development-evidence.tar.gz。
- 186 个当前核心/共享/launch 源文件和验收脚本与已测 WSL 快照逐一哈希一致；根目录 0914controller 的 4 文件与原始记录一致。ADRC 配置 SHA 与所有 27 次参数清单一致。
- ADRC 的最小保守障碍间隙 1.019 m，最大上报窗口 P99 0.393 ms、最大控制耗时 0.622 ms；当前参数到达明显慢于 PID/LQR，详细差异和不同到点容差已写入报告。
- 接入说明补齐已构建工作区准确路径、干净 ROS 环境、MAKEFLAGS=-j1、三算法与故障复现命令。保留分支和用户现有运行实例，未提交或推送。
