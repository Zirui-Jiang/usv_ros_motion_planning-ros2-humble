# Ocean 场景随机性与波相位可复现性核查

本目录记录一次**只读源码、安装文件和既有日志核查**。未启动 Gazebo，未修改被测源码、安装、环境、场景或验收阈值。核查发现当前 ocean 场景有非零 PMS 波浪，而两版 PID turn 01 的 goal 观测源时间分别为 3.1 s 和 2.8 s；现有重置流程没有将起航锁定到同一波浪时刻。这是潜在混杂条件，**不能证明海浪造成了该配对 `1.10912971776 > 1.10` 的跟踪指标失败**。原正式结果保持不变。

[sources.json](sources.json) 保存文件绝对路径、字节数、SHA256、代码原文和行号、公式输入输出，以及三条相邻事件的原始 JSON。其 SHA256 为 `6c82170b8a0de41978c93d2b6051164b26c3b4753ed4e6b0f39428c483084f5a`。该文件是本报告的精确来源索引，未复制大型原始日志。

## 版本和环境文件一致性

| 对象 | 固定版本 / 核查位置 |
| --- | --- |
| C++ 数值版本 | `632734ba31478bdb833e15949287b53ae9334cf3`，`/home/usv/usv_migration/baseline` |
| Python 数值版本 | `3afb25f84232e560c62599a721dc0bfbe1a21657`，`/home/usv/usv_migration/python` |
| 基线格内试验 harness | `4ef01a645ada5fb816c8e1a412fa6cd779723539` 的 `scripts/validation/ros_acceptance.py` |
| 本次 Python harness | `3afb25f84232e560c62599a721dc0bfbe1a21657` 的同名脚本 |

两版实际文件逐字节比较：`ocean.world`、Otter 主 xacro、水动力/风 xacro、LiDAR xacro、`Wavefield.cc`、`WavefieldModelPlugin.cc`、水动力插件和风插件共 9 个文件相同；主 launch 不同，未宣称 launch 全文相同。两版 harness 的 `setup_world` 完整函数块相同，起始行与块哈希见 JSON。

两版源文件及各自 `install/sim_env/share/sim_env/worlds/ocean.world` 均为：

`c3b3daf40fd4e8815da1a6566da56514da0b8556c18b54bf1e96927be0585d1d`

关键源码 SHA256：

| 文件 | SHA256 |
| --- | --- |
| `otter_gazebo_dynamics_plugin.xacro` | `4f7cba242e3a75bf7ddf9364462e895526a1aae5366598e5e9d363e33fac6bda` |
| `Wavefield.cc` | `a611a9282a8c0a518e5d512c7828f90423950f7b3f25dda6a60423203a0e4174` |
| `usv_gazebo_dynamics_plugin.cc` | `428e01ee7e3fb8ed77076a70f98a09c221b021ce8ba04eece108cbe33309c953` |

文件和模型一致不等于轨迹逐时刻可复现。异步节点就绪、传感器更新、消息投递和控制应用时刻仍需单独约束；本核查没有测量它们各自造成的影响。

## PMS 幅值与时间相位

`src/sim_env/worlds/ocean.world:57–67` 设置 `model=PMS`、`period=5 s`、`number=3`、`scale=2.5`、`gain=0.1`、`tau=2 s`、`amplitude=0`、`steepness=0`。其中 `amplitude=0` **不使当前 PMS 分量归零**：`Wavefield.cc:191–197,220–245` 按以下公式计算幅值，公式中没有使用该 amplitude 参数：

\[
S(\omega)=\frac{0.0081\,9.81^2}{\omega^5}
\exp\left[-1.25\left(\frac{\omega_p}{\omega}\right)^4\right],
\qquad a_i=\gamma\sqrt{2S(\omega_i)\Delta\omega_i},
\]

其中 \(\omega_p=2\pi/5\)、增益 \(\gamma=0.1\)、\(\omega_i=\omega_p(2.5)^{i-1}\)，间隔按源码三项公式取值。代入结果如下；**这是源码公式计算，不是 Gazebo 实测浪高、艇体响应或实际外力时间序列**。

| 分量 | 角频率 / rad·s⁻¹ | 分量幅值 / m |
| --- | ---: | ---: |
| 0 | 0.502654824574 | 1.51021858×10⁻¹¹ |
| 1 | 1.256637061436 | 0.043367799353 |
| 2 | 3.141592653590 | 0.009643921858 |

Otter 水动力插件在 `usv_gazebo_dynamics_plugin.cc:309–333` 取世界 `SimTime`，调用 `ComputeDepthSimply` 并将波面高度用于浮力计算。实际路径 `Wavefield.cc:514–536` 为：

\[
h(x,y,t)=\left[\sum_i a_i\cos(k_i\mathbf d_i\cdot(x,y)-\omega_i t)\right]
\left(1-e^{-t/\tau}\right),
\]

此调用使用默认 `time_init=0`。未在该路径发现随机波相位；虽然参数对象有默认 0 的 phase，`ComputeDepthSimply` 不读取 phase 向量。因此，仅固定随机 seed 或修改 phase 配置，不足以让当前起航波面相同。`WavefieldModelPlugin::Reset():132–140` 仅把更新计时标记设为当前仿真时间，没有建立相对于 goal 的新相位原点。

## 两次 goal 的原始时间证据

两条目标均为 `[23.125, -22.125, -π/2]`，原始事件如下。

| 运行 | goal 事件行 | goal 源时间 / s | goal 单调墙钟 / s | 前后 odom 源戳 |
| --- | ---: | ---: | ---: | --- |
| C++ PID turn 01 | 107 | 3.1 | 71053.810687360 | 第 104 行：3.1；第 113 行：3.2 |
| Python PID turn 01 | 173 | 2.8000000000000003 | 79584.976664309 | 第 167 行：2.8；第 175 行：2.9 |

原始文件完整路径及 SHA256：

- `/home/usv/usv_migration/results/baseline-grid-interior-001/cpp_pid_turn_01/events.jsonl`：`76a9b9645af80d264d0979449940051d2691bfd0dae7e16caa75780c3b8a5bd6`。
- `/home/usv/usv_migration/results/python-grid-endpoint-final-matrix-001/python_pid_turn_01/events.jsonl`：`eb2b97bfa8355c8b95e104733b413e98000e018300216bc84c8268e82fbacbde`。

这些是 harness 发布 goal 时记录的源时间，有相邻 odom 源戳支持；不是逐物理步测量到的实际控制应用时刻。可确认记录中的起航时刻相差约 **0.3 s**，不能由此反算实际艇体扰动力，也不能认定这就是 RMSE 差异原因。

## 随机 seed、风和传感器

- `usv_gazebo_wind_plugin.cc:139–147` 在未提供非零 seed 时使用 `std::random_device` 初始化 `mt19937`；world 和 Otter 风 xacro 的 `<random_seed />` 都未固定有效 seed。
- 当前两处风配置均为 `wind_mean_velocity=0`、`var_wind_gain_constants=0`。插件 `:150–159,209–216` 的随机速度初值和滤波增益为 0。在正常正有限时间步下，这不构成非零随机风激励；**不能把未固定风 seed 等同于已证实的随机风扰动原因**。
- 实际安装的 `/opt/ros/humble/share/gazebo_ros/launch/gzserver.launch.py:155` 将 `seed` 默认设为空，上层项目 launch 没有传入 seed。全局 seed 未固定是协议记录缺口，但本核查未证明其对该航迹产生了何种影响。
- 两版主 launch 均显式关闭 IMU/GPS。启用的 ray LiDAR 为 2000 束、15 Hz，其 xacro 没有 noise 块；未发现该有效传感器路径中由项目配置引入的随机噪声。仍不能据此声称 Gazebo/ROS 的整体执行逐位确定。

## reset_world 的保证范围和后续边界

两版 `setup_world` 相同。当前 Python harness `:538` 以可选方式调用 `reset_world`，随后通过 `set_entity_state` 设置起始位置、朝向及消息默认零速度；`:550–558` 等待新 odom 的二维位置在 0.5 m 范围内以及较新的 costmap。代码没有固定仿真时间的 goal 屏障，没有验证完整六自由度姿态/速度已达到同一稳态，也没有相同波面或外力轨迹的验收条件。每个 trial 启动独立 Gazebo 能隔离前次实例状态，但不锁定相对于波场绝对时间的节点就绪和起航时刻。

本次证据足以指出“相同环境模型不保证相同扰动初始条件”，不足以建立“海浪导致这次失败”的因果关系。若另行开展诊断，可对称约束两版起航仿真时间、状态和波场观测，或另开相同静水配置的敏感性试验；应独立命名和保留原结果。**本轮未执行这些试验，现有分数、1.10 阈值和 M5 门槛均保持不变。**
