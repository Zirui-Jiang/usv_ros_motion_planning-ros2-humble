# 0914 PI＋ADRC Controller Implementation Plan

> **2026-09-14 用户澄清后的执行范围：** 按下文完整五步计划实施，包括多圈解缠、真实采样时间、生命周期、积分与分配处理、验收工具以及 Otter 参数匹配和闭环对比。此前将“第一版”理解为原样接入是助手误解，已撤销该缩减范围。原始交付目录保留；原参数仅作为来源对照，运行使用单独验证的 Otter 配置。

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 将用户提供的 `0914controller` 接入当前普通导航，提供可安装、可配置、可回退并经过闭环验证的 `controller_name:=adrc`。

**Architecture:** 在现有 `controller` 包内保存 PI、艏向控制和回归滤波核心，外包一个 `ADRCController(model, config)`，继续实现 `compute(state, trajectory, dt) → ControlCommand`。使用现有 LOS 参考、艇体纵向速度 `state.u` 和物理力/矩分配；通过独立 ADRC YAML 启用，沿用现有任务、地图和状态有效性检查。

**Tech Stack:** Python 3、现有 NumPy/pytest、ROS 2 Humble、Gazebo Classic、WSL/Ubuntu；算法核心不新增第三方依赖。

**Spec:** 本轮用户指定的 `0914controller` 源码及用户已确认的完整五步计划；既有接口依据 [新控制器接入说明](../../controller-development-guide.md) 和 [设计开发说明](../../controller-design-brief.md)。实施状态和实际证据见 [验证记录](../../validation/adrc-0914-v1/README.md)。

## Global Constraints

- 以本轮指定的 `0914controller` 为算法来源；`adrc`、`USV_control` 不进入新实现的导入链。
- 当前源工作树有大量未提交及未跟踪文件。读取和保留它们；不得使用 `git reset`、清理目录或仅按 HEAD 克隆来替代当前工作树。
- 执行阶段仅修改下文列出的接入范围，保留根目录原始交付算法，并记录移植差异。
- 正式算法位于 `src/core/control_module/controller/controller`，不采用运行时 `sys.path` 指向数字开头的根目录。
- 输入位置为 m、艏向为 rad、速度为 m/s；速度闭环采用有符号纵向 `state.u`。当前 LOS 前进参考语义不变，`state.v/r` 仍保留用于统一检查和其他模块。
- 接入约定：PI 输出纵向力 N，艏向控制输出偏航力矩 N·m；输出使用现有 `allocate_force` 变换为归一化左右指令，`lateral=0`。
- 首版保留源控制器的“滤波艏向误差＋误差差分＋积分＋可选测得角加速度反馈”结构；不添加神经网络、ESO、COLREGs 或新的参考生成器。
- `acceleration_gain=0.0` 作为首版固定设置；本轮不以调大该项作为跟踪失败的修复手段。
- PID/LQR 的默认选择、算法参数和数值行为保持；ADRC 使用启动参数显式选择，首版参数仅在启动时加载。
- 原 `controller_params_otter.yaml` 不新增 ADRC 键。新增专用 YAML，避免破坏 `usv_mission` 独立复制的旧参数白名单；本轮不增加 mission 的 ADRC 模式。
- 所有异常返回明确失败和零指令；到点使用既有位置、合速度判定，`SETTLING` 是停推等待，不宣称主动制动。
- ROS 测试在 Linux/WSL 中进行；缺少 ROS 而跳过的测试不能记作通过。数值诊断、安装验证、生产节点测试、Gazebo 闭环分别报告。

## 1. 已核实的源码事实与设计选择

来源共四个 Python 文件：`__init__.py`、`adrc_controller.py`、`pi_controller.py`、`heading_linear_regression_filter.py`。两条控制核心均采用 `compute(desire_psi, measured_psi, desire_speed, measured_speed, dt=None)`；PI 返回单个总推力，ADRC 返回单个偏航力矩。艏向控制内部估计角速度和角加速度，默认控制律的微分项实际使用误差差分，不直接使用 `state.r`。

已完成只读诊断，结果如下；这些不是闭环测试结果：

| 检查 | 结果 | 对计划的影响 |
| --- | --- | --- |
| 四个文件语法与包导入 | 成功 | 可移入标准 Python 子包 |
| 回归 `[0,1,4]` | 最新拟合值 3.6667、斜率 2.0 | 本版回归求和完整，不能沿用旧版本“循环内 return”的结论 |
| 连续旋转解缠 | 累计到 9.5 rad 时错误返回 3.2168 rad | 修正为累加相邻测量的最短角差，并覆盖正反多圈 |
| `compute(dt=0.2)` 配合 0.1 s 配置 | 真正 1 rad/s 的输入估计成 2 rad/s | 滤波改用样本时间；控制 dt 与测量时间分开管理 |
| 结构参数更新 | 配置已改为 0.2 s/4 点，内部滤波仍为 0.1 s/10 点 | 明确更新/复位规则并测试；ROS 首版不支持热改 |
| 默认速度力上限 | 8 N | 当前无扰动直线线性阻尼模型下，稳态上限约 8/20=0.4 m/s |
| 默认速度积分上限 | 8 N 的积分贡献 | 即使只提高总力上限，仍不能在零速度误差时提供维持 1 m/s 所需的约 20 N |

选择包内适配方案：它复用现有安全门、任务身份和参数协议，并允许与 PID/LQR 对照。直接另起推进器发布节点会重复控制权；把新算法替换掉旧 PID 会丢失可用回退基线，因此不选这两条路径。

数据映射固定为：

```python
reference = trajectory.reference_at_time(reference_time)
desired_heading = reference.theta
measured_heading = state.psi
desired_speed = reference.velocity
measured_speed = state.u
# 两个核心各返回物理量，再共同分配。
surge = speed_loop.compute(desired_heading, measured_heading,
                           desired_speed, measured_speed, sample_dt)
yaw = heading_loop.compute(desired_heading, measured_heading,
                           desired_speed, measured_speed, sample_dt,
                           measurement_stamp=state.stamp)
command = allocate_force(surge, yaw, model, command_limit=command_limit)
```

其中 `reference_time` 沿用现有 `_reference()` 的状态年龄＋前视逻辑，包括最终目标进近处理；以上片段仅描述核心映射，不替代完整输入检查。

## 2. 文件与责任边界

| 创建/修改 | 路径 | 职责 |
| --- | --- | --- |
| 创建 | `src/core/control_module/controller/controller/adrc_core/__init__.py` | 导出独立控制核心 |
| 创建 | `src/core/control_module/controller/controller/adrc_core/pi_controller.py` | 移植 0914 PI，保留增益与控制律，补齐输入/输出一致性验证 |
| 创建 | `src/core/control_module/controller/controller/adrc_core/adrc_controller.py` | 移植艏向核心，保持误差差分结构，接收样本时间并正确复位 |
| 创建 | `src/core/control_module/controller/controller/adrc_core/heading_linear_regression_filter.py` | 正反多圈解缠、按样本时间回归 |
| 创建 | `src/core/control_module/controller/controller/adrc_controller.py` | 轨迹/状态适配、采样管理、物理推力分配及状态返回 |
| 修改 | `src/core/control_module/controller/controller/controller.py` | 增加默认空实现的暂停通知方法 |
| 修改 | `src/core/control_module/controller/controller/controller_factory.py`、`src/core/control_module/controller/controller/__init__.py` | 注册并导出外层 `ADRCController` |
| 修改 | `src/core/control_module/usv_control/usv_control/control.py` | 在当前控制未执行或进入等待/失败时调用暂停通知；不改 timer 的 dt 定义 |
| 修改 | `src/core/shared/usv_navigation/usv_navigation/configuration.py` | ADRC 全量参数声明、类型/范围/到点容差校验 |
| 修改 | `src/core/shared/usv_navigation/usv_navigation/launch_support.py` | controller choices 加 `adrc` |
| 创建 | `src/core/shared/usv_navigation/config/robots/otter/adrc_controller_params_otter.yaml` | 旧 PID/LQR 参数快照＋专用 ADRC 参数 |
| 修改 | `scripts/control/control_demo.py` | ADRC 单步示例 |
| 修改 | `scripts/validation/ros_acceptance.py` | 新算法选择、故障测试按算法与同一 YAML 配置执行 |
| 修改 | `scripts/validation/verify_distributed_install.py` | 显式核查安装后的新类/核心模块和参数 |
| 创建 | `docs/0914-controller-integration.md` | 源码对应、变更说明、参数、运行和回退方法 |
| 修改 | `README.md` | 新算法入口及验证边界 |

`controller/setup.py` 的 `find_packages()` 可收集新子包，`usv_navigation/setup.py` 的 YAML 递归收集可安装专用配置。`scripts/main.sh` 已透传 launch 参数，通常无需修改。不重写普通规划与 mission 实现。

## Task 1: 移植与修正数值核心

**Files:** 创建上述 `adrc_core` 四个文件；创建 `src/core/control_module/controller/test/test_adrc_core.py`。

**Interfaces:** 保留 PI 的现有四标量＋可选 dt 接口；艏向核心兼容原调用，并增加仅关键字参数 `measurement_stamp=None`。滤波提供 `update(measurement, *, stamp=None, dt=None)`、`output()`、`reset()`；直接旧式调用使用配置周期累计时间，给定 stamp 时以 stamp 为准。

- [x] 记录四份源文件 SHA-256，并先保存“等采样、小角度、未饱和”原始控制输出，作为移植对照。
- [x] 先写缺失模块和已知问题的失败测试：回归数值、正反多圈、±π 跨界、不等间隔恒定角速度、初始化、非有限输入、积分限幅和结构参数更新。
- [x] 执行 `python -B -m pytest -q -p no:cacheprovider src/core/control_module/controller/test/test_adrc_core.py`，保留实际失败原因。
- [x] 复制核心后修正解缠：保存上一原始角和连续角，采用以下等价规则；明确假设相邻有效观测真实角增量小于 π。

```python
delta = math.atan2(math.sin(raw_angle - previous_raw_angle),
                   math.cos(raw_angle - previous_raw_angle))
continuous_angle = previous_continuous_angle + delta
```

- [x] 回归使用窗口内真实相对时间，避免把最后一次 dt 套给整个窗口；斜率单位直接为 rad/s。角加速度由带时间戳的角速度窗口回归得到。

```python
mean_t = sum(times) / len(times)
mean_y = sum(values) / len(values)
den = sum((t - mean_t) ** 2 for t in times)
slope = sum((t - mean_t) * (y - mean_y)
            for t, y in zip(times, values)) / den
latest_value = mean_y + slope * (times[-1] - mean_t)
```

- [x] 单样本导数为零；拒绝非有限输入；重复时间戳不入窗；倒退和长间隔由适配层复位。所有参数先完整验证再提交：周期正、整数窗口至少 2、历史长度不少于窗口、增益非负、积分上下限有序、力/矩上下限有序且包含零、误差复位角在 `(0,180]`。
- [x] `update_config` 改变滤波周期/窗口/历史长度时重新创建滤波器并清空导数历史；普通增益变化不暗改结构。ROS 仍只在启动时给配置。
- [x] 饱和与大误差清积分时，确保返回输出和最终保存的积分状态采用同一语义；明确记录相对原始实现的变化，并以第一拍输出和随后多拍输出的测试验证，不能用静默改增益掩盖差异。
- [x] 运行专项测试至通过。关键时间回归测试示例：

```python
from controller.adrc_core.heading_linear_regression_filter import HeadingLinearRegressionFilter

def test_irregular_samples_have_correct_rate():
    f = HeadingLinearRegressionFilter(window_length=5, history_length=10)
    for t in (0.0, 0.08, 0.21, 0.31, 0.49):
        f.update(0.3 + 0.2 * t, stamp=t)
    assert abs(f.output()[1] - 0.2) < 1e-10
```

交付：可独立调用的两个核心和有时间语义的滤波器；原始文件不改。

## Task 2: 外层适配、采样与暂停恢复

**Files:** 创建外层 `adrc_controller.py` 和 `test/test_adrc_controller.py`；修改 `controller.py`、`controller_factory.py`、包 `__init__.py`、`usv_control/control.py`；扩展生产节点 `usv_control/test/test_control_ros.py`。

**Interfaces:** 外层 `ADRCController(USVModel, config=None)` 继承 `Controller`，提供 `compute(VesselState, Trajectory, dt)`、`reset()`、`on_control_hold()`；内层艏向类导入时使用别名 `HeadingController`，避免与适配类混淆。

- [x] 先写接口测试：工厂选择、速度采用 `u` 而不是 `hypot(u,v)`、左右方向、零侧推、输出范围、空/负速/不同坐标系轨迹、非有限状态、无效 dt、终点和局部预测终点区分。
- [x] 外层复用 `_reference()`；调用两个核心并复用 `allocate_force(..., command_limit=...)`。不重复归一化，不对物理力矩做 `wrap_angle`。
- [x] 每个实例记录上次有效 `state.stamp`、上次输出。新观测才更新两个核心；重复状态在输入/参考有效时保持上次命令，不重复积分或入窗。新任务由既有节点 reset 清除缓存。
- [x] 连续控制时使用两次有效状态的时差作为核心 sample_dt；首拍使用配置名义周期并初始化导数。倒退状态本拍返回零并复位；样本间隔超过 `max_sample_gap=0.5 s` 时清空历史并按首拍重新开始，不能对整段停顿积分。
- [x] 增加暂停通知，解决“节点一直更新 timer dt，却在等待中跳过 compute”的实际行为。默认方法为空，ADRC 覆盖为 reset；生产控制节点在本周期没有执行成功的正常跟踪时通知，覆盖 `WAITING_*`、`SETTLING`、暂停和控制错误。

```python
# Controller 基类
def on_control_hold(self):
    pass

# ADRC 适配类
def on_control_hold(self):
    self.reset()
```

- [x] 同一任务正常滚动重规划不 reset；测试“收到新轨迹仍连续滤波”与“等待后恢复首拍无旧导数尖峰”。既有时钟回退/取消/模块重启的任务撤销逻辑保留。
- [x] 推力分配若发生共同缩放，则从实际左右输出还原已应用的力/矩，对积分采用条件冻结：`(requested_surge-applied_surge)*(speed_integral_delta)>0` 冻结速度积分；`(requested_yaw-applied_yaw)*(-heading_integral_delta)>0` 冻结艏向积分。大误差清积分和任务复位优先，冻结不得恢复已经清除的历史。冻结后用本拍缓存的非积分项重算输出，不重新推进滤波器。测试前进/后退及正反转向四种符号。
- [x] 算术或分配失败返回 `INVALID_INPUT`、原因和零指令，同时复位，避免已写入的历史污染后续计算。
- [x] 运行核心/适配专项及旧 PID/LQR 测试；在生产节点测试中验证暂停通知对 PID/LQR 是空操作，旧行为无变化。

物理分配独立预期：当前 `D=0.395 m`、`max_force=50 N`，输入 `surge=20 N`、`yaw=3.95 N·m`、`command_limit=1.0`，应得到 `left=0.1`、`right=0.3`。

## Task 3: 配置、启动和兼容性

**Files:** 修改共享 `configuration.py`、`launch_support.py`、`scripts/control/control_demo.py`；新增专用 YAML；扩展 `src/sim_env/test/test_configuration.py`、`test_launch_modules.py`、控制/决策模块的 launch 测试；回归 `src/core/mission_module/usv_mission/test/test_task_configuration.py`。

**Interfaces:** 全部新配置位于 `usv_simulation.controller.adrc_controller.*`；decision/control 通过同一共享 DEFAULTS 声明、发送和校验完整快照。

- [x] 写失败测试：`controller_name=adrc` 可解析、专用 YAML 被所有普通导航入口一致读取、CLI 名称覆盖 YAML、未知参数拒绝、非法上下限拒绝、启动后参数修改提示重启。
- [x] 工厂注册 `"adrc": ADRCController`；共享 launch choices 增加 `adrc`；DEFAULTS 和类型/范围校验同时增加新参数。保证 `adrc_controller.goal_distance_tolerance` 存在，满足规划层的按名称读取。
- [x] 基础共有参数：`goal_distance_tolerance=1.5`、`goal_speed_tolerance=0.25`、`lookahead_time=2.6`、`sample_time=0.1`、`max_sample_gap=0.5`、`command_limit=1.0`。这些是首轮比较起点，前视时间尚未针对 ADRC 标定。
- [x] 核心参数使用平坦且不歧义的键：`k_p_speed`、`k_i_speed`、`force_min/max`、`speed_integral_min/max`、`k_p_heading`、`k_d_heading`、`k_i_heading`、`moment_min/max`、`heading_integral_min/max`、`error_reset_deg`、`acceleration_gain`、`filter_window_length`、`filter_history_length`。外层分别映射为原核心的 `integral_min/max`。
- [x] 共享 DEFAULTS 保留原始核心增益和限幅以供来源对照；专用 YAML 使用下列**待闭环调定的 Otter 联调起点**。复制旧 PID/LQR 配置到专用 YAML，三种算法对照时使用同一份参数文件。

```yaml
controller:
  controller_name: adrc
  control_frequncy: 10.0
  adrc_controller:
    goal_distance_tolerance: 1.5
    goal_speed_tolerance: 0.25
    lookahead_time: 2.6
    sample_time: 0.1
    max_sample_gap: 0.5
    command_limit: 1.0
    k_p_speed: 8.0
    k_i_speed: 0.8
    force_min: -40.0
    force_max: 40.0
    speed_integral_min: -30.0
    speed_integral_max: 30.0
    k_p_heading: 1.5
    k_d_heading: 0.5
    k_i_heading: 0.03
    moment_min: -10.0
    moment_max: 10.0
    heading_integral_min: -2.0
    heading_integral_max: 2.0
    error_reset_deg: 20.0
    acceleration_gain: 0.0
    filter_window_length: 10
    filter_history_length: 50
```

这段只展示 ADRC 部分；最终文件中的 PID/LQR 两段必须逐项来自执行开始时的旧 YAML，不能以本文重新抄写的默认值覆盖用户设置。新的力与积分限幅用于容纳当前艇模的稳态阻尼，不能称为已验证参数。原始限幅只用于数值来源对照；正式启动示例必须显式加载专用配置。

- [x] 参数 `sample_time` 表示无时间戳调用和首拍的名义周期；普通 ROS 部署要求与配置 `control_frequency` 的倒数一致，但后续样本按真实时间处理。核心窗口长度必须为整数，首版不接受隐式四舍五入改变窗口。
- [x] 保留原 `controller_params_otter.yaml`，不修改 mission 的白名单或默认入口；用旧 YAML 跑 mission 配置回归。专用 ADRC YAML 只用于普通导航。
- [x] 在 `control_demo.py --controller adrc` 添加明确配置和单步 JSON 输出；输出说明物理单位及所用参数来源。
- [x] 运行配置、launch、demo 与旧配置回归；确认新模块和专用 YAML 被已有 setup 自动安装。

## Task 4: 生产节点与验收工具覆盖

**Files:** 修改 `scripts/validation/ros_acceptance.py`、`verify_distributed_install.py`；扩展 `test_harness_options.py`、`test_distributed_validation.py`、`test_distributed_install.py` 和生产 `usv_control/test/test_control_ros.py`。

- [x] 先写 CLI/dry-run 测试，使 `--controllers adrc`、三算法矩阵和 ADRC 故障矩阵可见；运行时从真实 `usv_control` 参数读回算法名称和全部 `adrc_controller.*`。
- [x] 扩展验收 selector。保留现有 ListParameters 全量采集逻辑，不重写历史结果协议。
- [x] 将任务替换、取消和 odom 断流用例的 PID 硬编码改成选中的控制器。断流用例直接启动进程，没有经过 launch；因此必须把同一 `--controller-config` 解析后的全量参数同时传给 decision/control，不能只改名字。
- [x] 生产 ROS 测试覆盖：新算法创建、有效指令、取消、过期/旧会话参考拒绝、配置快照不一致、状态丢失、重复/倒退状态时间、暂停/恢复、时钟回退、模块重启；结果包含实际算法名称。
- [x] 安装审计显式导入外层和三个核心模块，调用 `create_controller('adrc', ...)`，记录模块 `__file__`、安装前缀和专用 YAML SHA-256；与本轮快照中的算法/YAML 文件逐个比对。
- [x] 旧 `sim_env/test/test_navigation_ros.py` 仅作 legacy 参照；新算法的 ROS 证据必须来自 `usv_control`。不把面向 Python/C++ 迁移的 `compare_results.py` 直接当作三算法性能比较器。
- [x] 指标复用原始 JSONL 和既有轨迹/间隙统计；新增 ADRC/PID/LQR 同场景结果表。航速/艏向误差、饱和率如需新增计算，明确公式、参考对齐时刻、单位和采样范围。

## Task 5: 构建、参数验证、闭环与交付

**Files:** 创建 `docs/0914-controller-integration.md` 与 `docs/validation/adrc-0914-001/README.md`，更新 README；新验证脚本仅在现有工具无法输出必须指标时添加，首个为 `scripts/validation/adrc_numeric_probe.py`（纯数值通道测试，无 ROS）。

- [x] 执行开始时记录当前源文件/配置哈希和 PID/LQR 基线。局部先跑核心、控制器、配置、launch 和生产控制节点测试；修复后再跑完整源码回归一次。
- [x] 数值通道验证使用 `common.step_dynamics`：直线固定 1 m/s 参考，时间步 0.1 s、运行 180 s，末 20 s 平均纵向速度误差 ≤0.1 m/s，全部指令有限且合法；艏向阶跃正负 30°、初始纵向速度 0.3 m/s，末 20 s 平均绝对角误差 ≤5°。这是本轮新增的模型测试门槛，不是 Gazebo 或实艇结论。
- [x] 原始增益＋新的物理限幅先测试；若不能满足门槛，按速度环、艏向环顺序进行可记录的参数调整。保留失败结果和候选参数；不修改艇模阻尼、LOS 轨迹或放宽验收阈值来制造通过。最终 YAML 必须使用通过实际验证的参数。
- [x] 从当前授权工作树复制到一个全新的 WSL 工作区，包括未跟踪的正式算法与配置；不使用只包含已提交 revision 的 `build_clean.bash`。建立源路径、文件清单和哈希记录。
- [x] 新 shell 只加载 Humble 和本轮安装区，构建 `--base-paths src`。安装检查在不加入源码 PYTHONPATH 的环境执行，避免把源码导入误记为安装成功。
- [x] ADRC 先跑直线一次，再跑直线/转弯/单障碍各三次；PID/LQR 使用同一艇模、场景、规划参数和专用 YAML 中保留的原参数，各三次。单独跑 ADRC 故障矩阵。
- [x] 沿用已有场景门槛：实际控制器位置容差内、合速度 ≤0.25 m/s、保持 1 s 墙钟；180 s 仿真/300 s 墙钟超时；最大横向误差 ≤4 m；单圆柱保守间隙 ≥0.2 m；取消 0.5 s、状态断流 0.7 s 内零推力。任何缺测/超时/未知结果都如实记录。
- [x] 当前验收脚本不会以航速跟踪误差直接判失败，因此必须保留上述独立速度测试；不能用“到点通过”替代“能跟踪 1 m/s”。10 Hz 控制下单独检查控制耗时 p99 <100 ms，记录最大值及超周期次数，不宣称节点能抢占超时算法。
- [x] 文档列出原始文件对应关系、必要修正、默认参数与已验证配置、启动/回退命令、结果/失败样本，以及实艇未验证边界。验收通过后才标记“接入完成”；性能是否优于 PID/LQR 按结果另行判断。

### 执行命令约定

以下为 Linux/WSL 仓库目录中的复现命令。实际本轮工作区、执行命令及结果见验证记录；复制时保持当前工作树，不带旧 build/install 或 deliverables：

```bash
set -euo pipefail
export ADRC_SOURCE="$(pwd)"
export ADRC_WS="/tmp/usv-adrc-$(date +%Y%m%d-%H%M%S)"
mkdir "$ADRC_WS"
tar -C "$ADRC_SOURCE" -cf - src scripts docs README.md conftest.py pytest.ini 0914controller \
  | tar -C "$ADRC_WS" -xf -
cd "$ADRC_WS"
source /opt/ros/humble/setup.bash
MAKEFLAGS=-j1 colcon build --base-paths src --executor sequential \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON
source "$ADRC_WS/install/setup.bash"
export ADRC_CONFIG="$(ros2 pkg prefix usv_navigation)/share/usv_navigation/config/robots/otter/adrc_controller_params_otter.yaml"
```

准备快照时检查管道两端退出码，保存源和快照哈希；若临时目录生命周期不足以留存结果，交付前把最终报告、日志、参数和构建清单归档到项目的验证目录。

```bash
bash scripts/validation/run_source_tests.bash \
  src/core/control_module/controller/test \
  src/core/control_module/usv_control/test \
  src/sim_env/test/test_configuration.py \
  src/sim_env/test/test_launch_modules.py \
  scripts/validation/test_harness_options.py \
  scripts/validation/test_distributed_validation.py

export USV_DISTRIBUTED_INSTALL="$ADRC_WS/install"
bash scripts/validation/run_distributed_source_tests.bash \
  "$USV_DISTRIBUTED_INSTALL" "$ADRC_WS/validation-results/adrc-source-tests-001"

python3 scripts/validation/verify_distributed_install.py \
  --install "$ADRC_WS/install" \
  --output "$ADRC_WS/validation-results/adrc-install-audit-001.json"

python3 scripts/validation/ros_acceptance.py \
  --workspace "$ADRC_WS" --distributed --controllers adrc \
  --controller-config "$ADRC_CONFIG" --scenario straight --repeats 1 \
  --output "$ADRC_WS/validation-results/adrc-smoke-001"

python3 scripts/validation/ros_acceptance.py \
  --workspace "$ADRC_WS" --distributed --mode python \
  --controllers pid lqr adrc --controller-config "$ADRC_CONFIG" \
  --scenario straight turn single_obstacle --repeats 3 \
  --costmap-topic /otter/costmap_raw \
  --output "$ADRC_WS/validation-results/adrc-matrix-001"

python3 scripts/validation/ros_acceptance.py \
  --workspace "$ADRC_WS" --distributed --controllers adrc \
  --controller-config "$ADRC_CONFIG" --only-faults \
  --output "$ADRC_WS/validation-results/adrc-faults-001"
```

输出目录必须是新的；重试使用新后缀并保留失败目录。上述新增 selector、故障参数传递和安装审计需要先按 Task 4 实现。初始数值脚本的接口固定为 `python3 scripts/validation/adrc_numeric_probe.py --controller-config "$ADRC_CONFIG" --output "$ADRC_WS/validation-results/adrc-numeric-001"`，输出 CSV 状态/参考/命令与 JSON 判定。

手动启动和回退（每次运行一个）：

```bash
ros2 launch sim_env main.launch.py world:=ocean headless:=true \
  controller_name:=adrc controller_config:="$ADRC_CONFIG"
# 停止上一进程后回退原配置和 PID：
ros2 launch sim_env main.launch.py world:=ocean headless:=true controller_name:=pid
```

## 3. 本轮交付与执行边界

用户已确认执行本计划。核心、适配、配置、验收工具已经实施，实际构建、测试、参数调整及闭环证据统一记录在验证目录中。文中的初始参数表保留为调参起点，正式启动以专用 YAML 定稿参数为准。保留当前分支及已有用户工作，不自动暂存或提交。
