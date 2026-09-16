# 新控制器设计开发说明

面向：负责重新设计无人艇控制算法的开发者。基于当前 Python / ROS 2 项目，日期：2026-09-14。

本任务是设计一个能跟踪现有期望轨迹的控制器，并输出左右推进器指令。首轮使用现有 Otter 仿真完成验证；实船参数由甲方提供或通过试验辨识。实现后的代码接入步骤见 [新控制器接入说明](controller-development-guide.md)。

## 1. 要完成的控制任务

根据本船实际状态和规划层给出的期望轨迹，计算每一周期的推进器指令，使船能完成直线、转弯、变速跟踪和到点停船。

规划层继续承担路径规划与轨迹生成，控制层负责跟踪、推力分配和控制异常处理。当前船为双推进器差动结构，没有可独立控制的横向推进器。算法由开发者选择，并说明所需模型、控制变量和适用范围。

首版沿用接口：

```python
controller = NewController(model, config)
command = controller.compute(state, trajectory, dt)
controller.reset()  # 新任务或任务结束时清除算法历史
```

## 2. 已有输入与输出

| 输入 | 内容与单位 |
| --- | --- |
| 实际位置、艏向 | `state.x/y`：世界坐标，m；`state.psi`：艏向，rad |
| 实际速度、转向率 | `state.u/v`：艇体系纵向/横向对地速度，m/s；`state.r`：转向角速度，rad/s |
| 时间与坐标系 | `state.stamp`、`state.frame_id`；与轨迹使用同一时钟和世界坐标系 |
| 期望轨迹 | 每行 `[t, s, x, y, theta, kappa, velocity, acceleration]`，分别为相对时间、弧长、位置、艏向、曲率、速度、加速度 |
| 控制间隔 | `dt`：两次控制调用的实际时间差，s；当前默认频率 10 Hz |
| 模型与配置 | `model` 保存艇型参数，`config` 保存新算法参数 |

坐标约定：艏向正东为零、逆时针为正；艇体系向前为正纵向、向左为正横向。实际横向速度 `v` 应保留，不能因无侧推而设为零。

轨迹来自现有 LOS 生成器，默认每 0.5 s 一个点，向未来预测 20 s。`t` 相对 `trajectory.stamp`；`goal` 是整个任务最终目标。设计人可选择使用一个参考点或整段轨迹。现有控制器用“状态时刻减轨迹起点时刻，再加预瞄时间”读取参考，并对离散点插值。

需确认新算法的速度口径：当前 PID 跟踪速度大小 `hypot(u,v)`，LQR 将参考速度用作纵向 `u`。有侧滑时两者不同。当前 LOS 是无侧滑运动学参考，未严格施加加速度、jerk 和推进器动态约束；设计中需要考虑参考不可实现时的限幅、降速或失败反馈。

输出为 `ControlCommand(left, right, lateral, status, message)`：

- `left/right` 是归一化推进器指令，范围 `[-1,1]`，正值正推、负值反推。
- 当前双推进器结构要求 `lateral=0`。
- 成功返回 `SUCCESS`；无效输入或计算失败返回明确状态和原因，输出零指令。求解超时使用 `TIMEOUT`。
- `GOAL_REACHED` 根据实际位置和速度判断；预测轨迹到达终点不能当作实际到达。零指令表示停推，不保证立即停止，主动制动需求应写入控制方案。

数据结构见 [models.py](../src/core/shared/common/common/models.py) 和 [trajectory.py](../src/core/decision_module/trajectory_planner/trajectory_planner/trajectory.py)；已有参考与到点逻辑见 [controller.py](../src/core/control_module/controller/controller/controller.py)。

## 3. 设计前需要的参数

以下是当前 [dynamics_otter.yaml](../src/core/shared/usv_navigation/config/robots/otter/dynamics_otter.yaml) 中的仿真模型参数，可用于首轮开发，不能直接认定为甲方实船参数。

| 参数 | 当前值 | 用途 |
| --- | --- | --- |
| `mass`、`i_z` | 29.84 kg、10.03 kg·m² | 平动质量、艏摇惯量 |
| `x_dot_u`、`y_dot_v`、`n_dot_r` | 5 kg、5 kg、1 kg·m² | 纵向/横向附加质量、艏摇附加惯量 |
| `x_u`、`y_v`、`n_r` | -20、-20、-20 | 带符号的线性阻尼；前两项单位 N·s/m，后一项 N·m·s/rad |
| `D` | 0.395 m | 单侧推进器到中心线的横向力臂；左右间距为 `2D` |
| `max_force` | 50 N | 单台推进器推力尺度 |
| `L`、`W` | 1.35 m、1.00 m | 模型尺寸参数 |
| `cruising_speed` | 1.0 m/s | 参考巡航速度 |
| 速度、加速度、jerk 范围 | 0～1 m/s、-1～1 m/s²、-2～2 m/s³ | 配置范围，实际执行能力需验证 |

平面模型计算可参考 [common/dynamics.py](../src/core/shared/common/common/dynamics.py)。它是低速平面近似；LQR 的历史线性化模型另写在其算法文件中，两者不能未经验证就视为完全相同。

若面向实船，需要甲方/控制组补齐下列资料，并记录来源、单位与试验条件：

| 需要的资料 | 具体内容 |
| --- | --- |
| 船体与载荷 | 质量、质心、惯量或可估算资料，典型载荷变化，控制参考点 |
| 推进器 | 安装位置与方向、正反转能力、左右各自的命令—推力关系、死区与最大连续输出 |
| 执行与测量动态 | 推进器响应延迟、输出变化率限制，导航状态频率、精度与延迟 |
| 辨识与性能数据 | 已有加减速、停车、转圈等试验；缺少水动力参数时明确需要补做的辨识 |

算法 YAML 与 Gazebo 船体参数分开维护；修改前者不会自动改变仿真对象。涉及船体/执行器变化时，还需核对 [Otter URDF 目录](../src/sim_env/urdf/otter)。

## 4. 推力分配在哪里、如何使用

当前代码有三种处理方式：

| 代码位置 | 当前处理 |
| --- | --- |
| [PID 的 compute()](../src/core/control_module/controller/controller/pid_controller.py) | 先计算纵向和转向控制量，再在函数内部合成为左右指令；这些历史控制量属于归一化标定域 |
| [LQR 的 compute()](../src/core/control_module/controller/controller/lqr_controller.py) | 通过 `thrust = K @ error` 直接得到左右归一化指令，推进器力臂和推力尺度已进入输入矩阵 |
| [dynamics.py 的 allocate_force()](../src/core/shared/common/common/dynamics.py) | 接收物理纵向力和艏摇力矩，分配为左右归一化指令；现有阵位保持和制动工具会调用 |

**若新控制器输出纵向力 X（N）和艏摇力矩 N（N·m），可使用公共 `allocate_force()`。** 在当前对称线性推进器模型下，未限幅时：

\[
F_L=\frac{X-N/D}{2},\qquad F_R=\frac{X+N/D}{2},
\]
\[
a_L=F_L/F_{max},\qquad a_R=F_R/F_{max}.
\]

对应调用示意：

```python
from common.dynamics import allocate_force

command = allocate_force(surge=20.0, yaw=0.0, model=model)
# 当前 D=0.395、max_force=50 时：left=0.2，right=0.2。
```

`allocate_force()` 默认 `command_limit=0.4`，超限时两侧按同一比例缩放。普通 PID/LQR 的 `_command()` 则分别裁剪到 `[-1,1]`，两者饱和策略不同。设计人需确定采用的输出限幅和抗积分饱和方案；新算法若已经输出左右归一化指令，则直接构造输出，不再进行一次力分配。

当前仿真推进器的线性映射和正反向最大推力在 [otter_gazebo_thruster_config.xacro](../src/sim_env/urdf/otter/thrusters/otter_gazebo_thruster_config.xacro)。实船应采用标定得到的映射。

## 5. 开发者需要完成的设计

按下面顺序给出简要方案和实现即可：

1. **定义控制变量与误差。** 说明跟踪位置、艏向、速度中的哪些量，以及速度使用 `u` 还是速度大小；角度误差处理 ±π 跨界。
2. **确定模型与控制律。** 列出实际使用的状态方程、控制公式、离散周期和参数含义；无完整模型时说明所需辨识或近似。
3. **处理参考与约束。** 明确单点/整段参考、预瞄、饱和、输出平滑、到点减速/制动与参考切换行为。
4. **完成推力分配。** 写清内部控制量的单位、左右符号和归一化映射，保证没有侧推输出。
5. **实现计算与复位接口。** 算法对象跨周期复用；`reset()` 清除历史。有迭代求解时提供停止条件和超时处理。
6. **接入并验证。** 注册算法和配置，完成数值测试及 ROS/Gazebo 联调，保留对比记录。

当前 `_reference()` 在实际位置进入目标容差但速度较高时返回零指令等待减速。若新设计需要主动制动，应同步调整这部分处理，而不是直接继承后默认已有制动能力。状态断流、取消、会话和 ROS 输出调度由现有 [usv_control/control.py](../src/core/control_module/usv_control/usv_control/control.py) 承担，新算法应兼容这些行为。

## 6. 验证与交付要求

| 验证内容 | 至少记录或检查 |
| --- | --- |
| 直线、转弯、变速跟踪 | 横向误差 RMS/最大值、最短角差艏向误差、按约定速度口径计算的速度误差 |
| 到点与切换 | 实际到点距离和速度、停船过程、目标替换后无旧历史导致的异常输出 |
| 输出与计算 | 左右指令范围、饱和比例、输出变化；计算耗时和超周期次数 |
| 异常情况 | 空轨迹、非有限输入、求解失败、取消任务、状态断流时的状态与输出 |
| 新旧对比 | 同一工况、参考与评价口径下对比现有 PID/LQR，报告结果与未达到的目标 |

当前 10 Hz 对应每周期 100 ms，总控制回调需要在周期内完成，算法应为状态检查和发布预留时间。跟踪精度和停船性能的最终阈值由双方在试验前约定；现有 PID 到点距离 1.5 m、LQR 1.0 m，速度阈值均 0.25 m/s，仅供基线参考。对比时应采用统一的验收条件，不用各算法自己的预瞄点作为唯一精度基准。

交付四项：设计说明与参数含义、控制器源码及配置、可复现运行方法、测试结果与未验证项。如用于实船，还需向规划组提供有依据的速度、转向、加减速和停车能力。

代码接入和命令见 [新控制器接入说明](controller-development-guide.md)；既有例程见 [control_demo.py](../scripts/control/control_demo.py)，算法测试见 [controller/test](../src/core/control_module/controller/test)。本文件是开发任务说明，不表示新控制器或实船性能已经验证。
