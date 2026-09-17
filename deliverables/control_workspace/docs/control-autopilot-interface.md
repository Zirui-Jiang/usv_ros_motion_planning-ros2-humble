# 独立控制开发与航控接口定义

> 快照链接说明：三个数值包的源码链接已适配本工作区；sim_env 证据路径仅在原仓库中可访问，本快照不包含该仿真包。

版本：2026-09-12，接口讨论稿。依据当前工作树的 PID、LQR、阵位保持控制器与 Otter 仿真实现。

**已确定的分工：与当前方案相同，由上位机控制器计算左右推进器指令，航控负责执行、反馈和控制权管理。** 航控厂家、通信协议、实艇推进系统及传感器能力尚未提供；本文的实艇字段、频率和超时是待双方确认的接口要求，不表示已经实现航控驱动或完成实艇验证。

## 1. 控制功能提取结果与边界

现有 `controller` 已经是一个 `ament_python` 功能包，算法入口为 `compute(state, trajectory, dt)`。本次将其与必要数值依赖提取为可独立使用的开发工作区：

```text
deliverables/control_workspace/
  src/controller/           PID、LQR、阵位保持及原测试
  src/common/               VesselState、ControlCommand、USVModel、数学/动力学工具
  src/trajectory_planner/   Trajectory 数据结构、插值、原 LOS 实现及测试
  examples/control_demo.py  用同一算法接口执行一次控制计算
  requirements.txt         纯数值开发依赖
  provenance.json          导出源文件、SHA256 与源码版本记录
  docs/control-autopilot-interface.md
```

以导出工具实际生成的文件清单为准。完整保留三个现有包，避免复制几个类后形成另一套行为不同的控制实现。保留 `trajectory_planner` 是因为它拥有当前轨迹数据类型；独立测试不需要启动 ROS、Gazebo、A* 或传感器节点。部分 `common` ROS 可视化适配函数仍需 ROS 环境，不能据此将包中所有模块都称为无 ROS 依赖。

提取的是**控制器开发快照**。原工程仍是维护源，快照不会自动同步。本次没有将原 `sim_env` 中的任务状态机、控制定时器、避障安全判定搬到独立 ROS 进程，也没有实现厂商通信协议。重新设计算法可直接在独立工作区进行；最终接入需将算法变更回迁并完成集成验证。

推荐的部署分工如下，图中的航控适配器属于后续实艇接入设计：

```mermaid
flowchart LR
    P[任务与路径规划] -->|带时间的轨迹与运行许可| C[控制算法 controller]
    C -->|左右归一化推进器指令| A[航控适配器]
    A -->|厂商执行协议| F[航控与推进系统]
    F -->|导航状态、执行反馈、控制权与故障| A
    A -->|统一状态与有效性| C
    A -->|艇体状态| P
```

| 模块 | 负责内容 | 对外约定 |
| --- | --- | --- |
| 规划与任务模块 | 路径、时序轨迹、任务切换、碰撞约束、任务许可 | 轨迹、目标、取消/停止、参考版本 |
| 控制算法包 | 轨迹跟踪、艏向/速度控制、左右推力分配、控制器历史 | `VesselState + Trajectory + dt → ControlCommand` |
| 航控适配器 | 解包、时间同步、坐标转换、传感器安装补偿、质量检查、指令封装、执行反馈 | 对算法提供统一坐标与单位；隔离厂家字段 |
| 航控 | 推进器执行、输出限幅/斜率约束、命令超时保护、急停/人工接管、状态反馈 | 具有最终控制权；返回接收/应用/拒绝结果 |

底层模式采用“外部推进器控制”。若航控还保留电机转速内环，必须声明归一化输入最终被映射成转速还是驱动占空比。不得同时让航控自己的艏向/航速闭环和上位机的同层闭环争夺左右推进器输出。

## 2. 当前代码已实现的接口

| 项目 | 当前定义与实现 |
| --- | --- |
| 状态类型 | `common.models.VesselState(x, y, psi, u, v, r, stamp, frame_id)` |
| ROS 状态入口 | 相对话题 `odom`，默认命名空间下为 `/otter/odom`，类型 `nav_msgs/msg/Odometry` |
| 位置和姿态 | 位姿按消息时间经 TF 转到 `otter/map`；内部只保留平面位置和 yaw |
| 速度 | `twist.linear.x/y` 必须已是艇体坐标速度，`twist.angular.z` 为平面转向角速度；`child_frame_id` 必须等于 `otter/base_link` |
| 轨迹 | 每点 8 列：`[t, s, x, y, theta, kappa, velocity, acceleration]`，另有 `stamp, frame_id, goal` |
| 控制器输出 | `ControlCommand(left, right, lateral, status, message)`；`left/right ∈ [-1,1]`，当前双推进器的 `lateral=0` |
| ROS 输出 | `/otter/thrusters/left_thrust_cmd`、`right_thrust_cmd`，各为 `std_msgs/msg/Float32`；现节点也发布恒零侧推通道，但当前 xacro 只配置左右两个推进器 |
| 控制调度 | 默认 10 Hz 的稳态时钟定时器；传入算法的 `dt` 是相邻控制回调的 ROS 时间差，不是固定写死的 0.1 s，也不是状态报文间隔 |
| 状态和轨迹有效期 | 默认状态 0.5 s、轨迹 1.0 s；检查未来时间与坐标一致性 |
| 仿真推力映射 | `mappingType=0`、`maxCmd=1`、单推进器正反向力为 `+50/-50 N`；插件命令超时为 1.0 s |
| 当前航控链路 | 使用 Gazebo 真值定位及推进插件；没有真实航控通信、RPM 反馈、控制权握手或实艇急停状态输入 |

代码依据：

- [状态与控制量](../src/common/common/models.py)
- [PID 控制及归一化分配](../src/controller/controller/pid_controller.py)
- [LQR 六状态及离散模型](../src/controller/controller/lqr_controller.py)
- [阵位保持控制器](../src/controller/controller/station_keeping_controller.py)
- [轨迹定义](../src/trajectory_planner/trajectory_planner/trajectory.py)
- ROS 状态接入、调度与命令发布（仅原仓库：`src/sim_env/sim_env/simulation.py`）
- 默认参数（仅原仓库：`src/sim_env/sim_env/configuration.py`）
- 仿真真值定位转换（仅原仓库：`src/sim_env/src/fake_localization_node.cpp`）
- 推进器映射（仅原仓库：`src/sim_env/urdf/otter/thrusters/otter_gazebo_thruster_config.xacro`）
- 推进器超时（仅原仓库：`src/sim_env/urdf/otter/thrusters/otter_gazebo_thruster_plugin.xacro`）

`local_trajectory` 目前发布为 `nav_msgs/Path`，用于显示，只带位姿，缺少控制算法所需的速度、加速度、相对时间等信息。后续拆成独立 ROS 控制节点时，应另定义完整轨迹消息，不能直接把这条显示话题当成等价的控制输入。

## 3. 航控必须返回什么

“算法必需”表示当前 PID/LQR/阵位保持兼容接口需要；“接入必需”表示接入真实推进系统需要，但当前仿真尚未消费。两者应一并纳入双方接口确认。

### 3.1 导航状态：形成完整的六状态向量

\[
\boldsymbol{x}=[x_E,\ y_N,\ \psi,\ u,\ v,\ r]^\mathsf{T}.
\]

| 返回参数 | 建议字段 | 精确定义 | 单位/范围 | 要求与用途 |
| --- | --- | --- | --- | --- |
| 状态对应时刻 | `state_time` | 状态估计实际对应的时刻；不是上位机收包时刻 | 明确时钟域的整数时间戳 | 算法必需；插值、参考对齐、数据年龄 |
| 平面位置 | `east_m, north_m` | 指定艇体参考点在固定局部 ENU 原点下的位置 | m | 算法必需；可由航控经纬度在适配器中换算 |
| 艏向 | `yaw_enu_rad` | 艇艏方向相对正东的角度，逆时针为正 | rad，统一到 `[-π,π)` | 算法必需；真北参考转换而来，不以 COG 替代 |
| 纵向速度 | `surge_ground_mps` | 艇体参考点对地速度沿艇艏方向的有符号分量，向前为正 | m/s | 算法必需；前进/倒退不能用无符号航速代替 |
| 横向速度 | `sway_ground_mps` | 对地速度沿艇体左舷方向的有符号分量，向左为正 | m/s | 算法必需；反映侧滑，不能用“艇无侧推”作为填零理由 |
| 转向角速度 | `yaw_rate_body_radps` | 艇体 FLU 坐标绕 z 轴的角速度；平面假设下等于 yaw 导数 | rad/s，逆时针/左转为正 | 完整状态必需；LQR、阵位保持及转向阻尼使用 |
| 坐标参考 | `world_frame, body_frame, origin_id` | 局部原点、轴向、艇体参考点、原点版本 | 字符串或协议枚举 | 接入必需；原点变化应显式通告并使旧参考失效 |
| 状态有效性 | `position_valid, heading_valid, velocity_valid, rate_valid` | 各字段是否可用于控制；包含失锁、未初始化、过期等区别 | 布尔/枚举及原因 | 接入必需；缺测不能用 0 伪装成静止 |
| 报文序号和启动标识 | `sequence, boot_id` | 每个数据流递增序号；设备重启后改变 boot 标识 | 协商整数位宽 | 接入必需；识别丢包、重复、乱序与重启 |

航控可以提供下面任一形式，经适配器转换后必须得到同一 `VesselState`：

1. **优先：融合导航状态。** 已完成 GNSS/INS 融合，返回位置、真艏向、艇体速度、角速度及质量信息。
2. **可接受：经纬度 + 真艏向 + 东北向速度 + 角速度。** 上位机进行地理和坐标转换。
3. **退化输入：经纬度 + SOG/COG + 真艏向 + 角速度。** 可在 SOG/COG 有效时构造速度，但低速/静止时质量会受限，需明确速度有效性；不能视为天然满足定点保持要求。

原始 IMU、原始 GNSS不是“已经提供六状态”的同义词。如果航控只提供原始数据，还需要明确由哪一方实现状态估计器；这一工作不属于更换 PID/LQR 本身。

### 3.2 执行器反馈：确认命令实际到达了哪里

| 返回参数 | 建议字段 | 定义与要求 | 级别 |
| --- | --- | --- | --- |
| 命令接收/应用结果 | `last_received_seq, last_applied_seq, result` | 区分收到、接受、已经应用、拒绝；拒绝给出原因。应用是航控输出已更新，不代表推力已达到 | 接入必需 |
| 左右最终输出 | `left_applied, right_applied` | 航控在控制权裁决、限幅、限速、互锁之后实际施加的指令；明确处于哪个处理环节 | 接入必需 |
| 输出类型与能力 | `output_mode, reverse_supported` | 当前使用归一化驱动/RPM/PWM等哪种模式；是否允许倒车与左右独立反转 | 接入必需；与上位机模式一致 |
| 左右推进状态 | `left_enabled, right_enabled, fault_bits` | 每侧使能、离线、过流、过热、堵转、驱动器故障等；厂商位映射有版本 | 接入必需 |
| 左右实际转速 | `left_rpm, right_rpm` | 实际传感器/驱动器测得转速，与目标转速分字段；约定符号和正反转对应关系 | 强烈建议；执行诊断及辨识的重要反馈 |
| 电源电压和电流 | `bus_voltage_v, left_current_a, right_current_a` | 电池/母线测点与每侧驱动测点必须说明 | 推荐；识别电压变化、饱和、异常负载 |
| 温度/可用输出限额 | `temperature_deg_c, available_limit` | 热降额后的当前允许输出范围，而非仅铭牌上限 | 推荐 |
| 推力 | `left_force_n, right_force_n` | 如有测力传感器可报实测力；模型反算必须命名为 `estimated_force_n` 并给出模型版本 | 可选；没有传感器时不得伪称“实际推力” |

没有 RPM 遥测时，仍可能用艇体状态闭合轨迹控制环，但无法据此验证电机是否准确执行，系统辨识、异常诊断与饱和补偿能力受限。必须把“无法提供”写入协议，不要拿命令回显冒充转速测量。

### 3.3 控制权与系统健康

| 返回参数 | 建议字段 | 定义 |
| --- | --- | --- |
| 当前控制者 | `control_owner` | `MANUAL / EXTERNAL / AUTOPILOT / FAILSAFE / NONE` 等明确枚举，不能只写一个不清楚含义的 AUTO |
| 执行模式 | `control_mode` | 外部左右推进器、内部航速艏向、电机转速等；与 owner 分开定义 |
| 解锁与外部控制许可 | `armed, external_control_allowed` | 当前允许推进器输出与允许外部指令是否为真 |
| 急停与人工接管 | `estop_active, manual_override` | 当前实际状态及锁存/解除规则；来自航控独立保护链路 |
| 失效保护 | `failsafe_active, failsafe_reason` | 通信超时、导航失效、低电量、驱动故障等触发原因与当前动作 |
| 链路健康 | `heartbeat, last_command_age_ms` | 独立心跳及航控侧最后有效指令年龄；心跳正常不等于状态数据新鲜 |
| 导航估计健康 | `estimator_status, gnss_fix_type, heading_source` | GNSS/INS 初始化、失锁/降级；真艏向来自双天线、磁罗经融合还是其他方式 |
| 软件和协议版本 | `firmware_version, protocol_version, capabilities` | 连接时获取，变化时重新校验字段和模式能力 |

定位精度建议返回位置、速度、艏向方差/协方差及其有效标记；若只返回精度指标，应说明是标准差、置信半径还是其他定义。GNSS 卫星数、DOP、RTK 解状态可以辅助判断，但不能直接代替状态协方差。

ROS `NavSatStatus` 的 `FIX/SBAS_FIX/GBAS_FIX` 不能直接当作 `RTK_FLOAT/RTK_FIXED` 使用；RTK 解类型需另行定义厂商扩展字段。参见 [ROS 2 Humble NavSatStatus](https://github.com/ros2/common_interfaces/blob/humble/sensor_msgs/msg/NavSatStatus.msg)。

## 4. 坐标、单位与速度定义

### 4.1 统一采用 ENU + FLU

- 局部世界系 ENU：x 向东、y 向北、z 向上；指定 WGS84 原点 `(latitude0, longitude0, altitude0)` 与 `origin_id`。
- 艇体系 FLU：x 指向艇艏、y 指向左舷、z 向上；指定 `base_link` 参考点，推荐与控制动力学参考点一致。
- 线性量用 m、m/s，角度用 rad，角速度用 rad/s，力用 N，力矩用 N·m。
- 通信协议若使用真北顺时针角度、节、度/秒，统一在航控适配器内换算，控制算法内部不混用。

以上轴向与单位遵循 [ROS REP 103](https://github.com/ros-infrastructure/rep/blob/master/rep-0103.rst)。`map` 的长期全局一致性与 `odom` 的局部连续性职责不同，参见 [ROS REP 105](https://github.com/ros-infrastructure/rep/blob/master/rep-0105.rst)。当前仿真使用对齐的 map/odom；实艇融合定位可能发生 map 修正，不能假定两者永久重合。

如航控给经纬度，定义纬度北正、经度东正，单位 degree；高度若遵循 `NavSatFix` 是 WGS84 椭球高，不能与海拔高直接混用。坐标原点应固定并显式同步，不能每收到一帧数据就重新选择。当前工程 `GeoReference` 可执行 WGS84→ECEF→局部 ENU 转换。参见 [ROS 2 Humble NavSatFix](https://github.com/ros2/common_interfaces/blob/humble/sensor_msgs/msg/NavSatFix.msg)。

### 4.2 艏向不等于航迹向

设航海艏向为 \(H\)，真北为 0°、顺时针为正，则

\[
\psi=\operatorname{wrap}_{[-\pi,\pi)}\left(\frac{\pi}{2}-H\frac{\pi}{180}\right).
\]

| 真艏向 H | 船艏指向 | 内部 yaw ψ |
| --- | --- | --- |
| 0° | 北 | +π/2 |
| 90° | 东 | 0 |
| 180° | 南 | −π/2 |
| 270° | 西 | −π |

`heading` 是艇艏指向，`COG` 是对地速度方向。受流、侧滑或倒车影响时二者不同；静止时 COG 也不能作为可靠艏向反馈。若航控提供磁艏向，应另报参考类型，并在适配器内根据约定的磁偏角来源转换到真北。

### 4.3 航速不等于纵向速度

若航控给出 ENU 水平对地速度 \((v_E,v_N)\)，转换为

\[
\begin{bmatrix}u\\v\end{bmatrix}
=\begin{bmatrix}\cos\psi&\sin\psi\\-\sin\psi&\cos\psi\end{bmatrix}
\begin{bmatrix}v_E\\v_N\end{bmatrix},\qquad
U_g=\sqrt{u^2+v^2}.
\]

如果只有 SOG 与真北顺时针的 COG \(\chi\)，在二者有效时可先计算

\[
v_E=\mathrm{SOG}\sin\chi,\qquad v_N=\mathrm{SOG}\cos\chi.
\]

公式中的角度均以 rad 代入。`1 knot = 1852/3600 m/s`。例如艇艏向北且正向前进 1 m/s 时，`v_E=0,v_N=1`，转换后应为 `u=1,v=0`。

当前 PID 的误差是 `reference.velocity - hypot(u,v)`，使用对地速度大小；LQR 的目标状态为 `[x_ref,y_ref,psi_ref,velocity_ref,0,0]`，其中 `velocity_ref` 被作为艇体纵向目标速度。二者在存在横向速度时含义不同，重新设计控制器时应明确采用 \(u_d-u\) 还是 \(U_d-U_g\)，并重新整定，不能静默替换。

当前 `u,v` 源自仿真对地运动；若后续考虑水流，动力学阻尼通常需要相对水速度 \(\boldsymbol\nu_r\)，应另报/估计 `water_relative_velocity` 或流速，不得把对水航速和对地航速写进同一字段。

### 4.4 转向角速度与姿态

如果航控给的是平面航海艏向变化率 \(\dot H\)（顺时针正，°/s），平面转换为

\[
r=-\dot H\,\pi/180.
\]

如果给的是 FRD 陀螺仪向量，应先按完整安装旋转矩阵转换到艇体 FLU。仅在传感器轴与艇体轴对齐时，`FRD→FLU` 才是 `(p,q,r)→(p,−q,−r)`。不能把世界 NED 坐标转换与艇体 FRD 轴向转换混为一件事。

在明显横摇/纵摇时，陀螺 `r` 与欧拉角 `dot(psi)` 不相等。对 ZYX 欧拉角，

\[
\dot\psi=(q\sin\phi+r\cos\phi)/\cos\theta.
\]

因此建议同时保留四元数、横摇/纵摇与完整 `p,q,r`，说明姿态顺序、四元数分量顺序和变换方向。当前平面控制模型对大姿态运动的适用性需要另做验证。ROS `Imu` 的角速度单位为 rad/s，加速度单位为 m/s²，缺失字段和未知协方差有特定约定，参见 [ROS 2 Humble Imu](https://github.com/ros2/common_interfaces/blob/humble/sensor_msgs/msg/Imu.msg)。

### 4.5 传感器安装位置不能忽略

GNSS 天线、IMU 和动力学参考点通常不重合。协议配置需包含各传感器到 `base_link` 的安装位置、姿态和版本；位置、速度必须补偿到同一艇体参考点。若传感器相对参考点的艇体系位移为 \(\boldsymbol\ell\)，则

\[
\boldsymbol p_s=\boldsymbol p_b+R\boldsymbol\ell,\qquad
\boldsymbol v_s=\boldsymbol v_b+R(\boldsymbol\omega\times\boldsymbol\ell).
\]

转弯时天线速度不等于艇体参考点速度。安装外参应由双方确认，不从仿真尺寸推断实艇安装位置。

## 5. 上位机发给航控的指令

本次沿用当前算法的归一化执行量定义：

\[
\boldsymbol a=[a_L,a_R]^\mathsf T,\qquad a_L,a_R\in[-1,1].
\]

正值请求产生向前推进力，负值请求反向推进力，0 为约定中位/零推进请求。0 不意味着船速立刻为零，也不自动等同断电或机械制动。没有倒车能力的实艇不能直接兼容当前允许负指令的算法，需返回能力声明并重新约束控制分配。

| 指令字段 | 定义 |
| --- | --- |
| `protocol_version` | 约定消息版本，未知版本拒绝 |
| `session_id, command_seq` | 本次外部控制会话与递增命令序号；用于拒绝旧会话/乱序指令 |
| `command_time, valid_until` | 命令对应时刻和失效时刻，必须使用双方同步的时钟域；不能只从收到时刻重新计时使旧包复活 |
| `mode` | 固定为双方确认的外部左右推进器模式；不复用航速/艏向指令字段 |
| `left_normalized, right_normalized` | 同一报文携带、同一执行周期原子应用；有限数且在 [-1,1] 内 |
| `enable` | 请求外部输出；不代替航控解锁和控制权许可 |

心跳、控制权申请、释放和急停应有独立且明确的状态转换定义。航控应用命令后，通过前文反馈返回对应 `last_applied_seq`、应用时刻和最终输出。实艇单一报文可以用 CAN 多帧/串口帧/UDP 等传输，但需要完整组帧检查与序号；不预设厂商协议。

对称双推进器、左舷位置 \(y_L=+D\)、右舷位置 \(y_R=-D\)、推力均沿艇艏方向时，

\[
\begin{bmatrix}X\\N\end{bmatrix}
=\begin{bmatrix}1&1\\-D&D\end{bmatrix}
\begin{bmatrix}F_L\\F_R\end{bmatrix}.
\]

\(D\) 是单侧推进器相对中心线的横向力臂，不是左右间距；当前模型值 0.395 m，左右间距为 2D。实艇应以安装测量值为准。当前仿真线性模型使用 \(F_i=50a_i\) N，因此右侧推力大于左侧时，产生正的逆时针/左转力矩。

**归一化指令、推力、PWM 和 RPM 必须分开命名。** 实艇一般需要标定

\[
F_i=f_i(a_i,V,\mathrm{direction},\mathrm{inflow}),
\]

或 `a → RPM/PWM → 推力` 映射，考虑死区、正反转不对称、电压、饱和与动态延迟。不能把仿真的 `50 N` 直接用作实艇标定。当前 PID 增益按历史归一化分配整定，不能在其输出后未经分析又除一次 `max_force`；LQR 已在输入矩阵中使用 `max_force`。

## 6. 时序、质量与失效行为

下表均为**实艇接口协商的初始建议**，不是 ROS 标准要求或现有硬件性能。控制周期应结合船舶响应、通信带宽、估计器延迟与求解耗时确定。

| 数据/环节 | 当前仿真 | 实艇初始协商建议 |
| --- | --- | --- |
| 控制输出 | 10 Hz | 先支持稳定 10 Hz 联调；接口具备 20–50 Hz 能力便于重新设计 |
| 融合六状态 | 随 Gazebo 真值消息更新 | 20–50 Hz；报告实际输出频率、状态有效更新频率及估计延迟 |
| 原始角速度/IMU | 控制器未独立订阅 | 50–100 Hz，供辨识/状态估计；不要求与低频 GNSS 同包更新 |
| 推进器 RPM/最终输出 | 无实机遥测 | 10–20 Hz，并记录真实测量时刻 |
| 控制权/故障 | 无航控状态接入 | 状态周期 10 Hz，关键状态变化立即报告 |
| 状态年龄阈值 | 0.5 s | 初始考虑不超过 0.1–0.2 s；按传感器及网络实测后确定 |
| 命令看门狗 | 插件 1.0 s | 10 Hz 命令时可从 0.3 s 开始协商，按最大连续丢包与停推距离验证 |

报文频率不代表信息频率：GNSS 每秒更新 5 次但打包 50 次，应保留 GNSS 原始时刻；若 INS 输出 50 Hz 传播估计，则注明该状态的估计时刻、最近观测时间和质量。不同测量时刻的字段不能简单打成一包后赋同一新时间戳。

建议保留 `t_measurement`、`t_estimate`、`t_receive`、`t_apply` 四种时刻和时钟同步误差估计。GNSS UTC/GPS 周秒、航控启动计时与 ROS 时间不能直接相减；明确转换、闰秒策略、重启/回绕规则。真实硬件看门狗使用航控本地单调时钟，即使 ROS 时钟暂停或上位机进程终止也能执行保护。

推荐保护流程如下，属于后续适配器/航控实现要求：

```text
收到反馈：
  校验版本、来源、boot_id、sequence、坐标约定与数值范围
  转换采样时钟；拒绝重复、乱序、过期及不可解释的未来数据
  按有效性与测量时刻建立统一导航状态
  更新控制权、故障和实际执行反馈（分别计算数据年龄）

每个控制周期：
  若控制权不是 EXTERNAL、未解锁、急停或关键故障：
      清控制器历史，退出外部控制；航控按既定保护策略裁决输出
  否则若状态/参考失效、参考版本不符或时钟异常：
      清控制器历史；请求零推进/释放，并使上一非零命令尽快失效
  否则：
      command = controller.compute(state, reference, measured_dt)
      检查有限值、范围、算法状态和参考许可
      发送带 session/sequence/期限的左右原子命令
  记录命令、应用反馈、状态、参考、参数版本和故障原因

航控独立执行：
  人工/急停/失效保护优先于外部命令
  只有当前会话、合法模式、未过期且许可满足时才应用命令
  超时后执行双方确认的保护动作，并拒绝旧命令重新激活
```

“零推进”仅表示停止继续施加推力，艇体会惯性滑行或随流漂移。实艇失联后的减速、停推、返航或驻位策略需根据航区和底层能力明确，不能用软件发布一次 0 作为“已安全停车”的证据。恢复链路后须重新确认控制会话与新参考，不自动恢复旧非零命令。

## 7. ROS 2 映射建议

| 内容 | ROS 表达 | 必须保留的信息 |
| --- | --- | --- |
| 融合导航状态 | 沿用 `/otter/odom`，`nav_msgs/Odometry` | `header.stamp` 为状态时刻；pose 在 `header.frame_id`，twist 在 `child_frame_id=otter/base_link` |
| 原始 GNSS | 建议 `/otter/autopilot/gnss_fix`，`sensor_msgs/NavSatFix` | WGS84、天线参考、协方差、fix 状态；RTK 类型另报 |
| 原始/融合 IMU | 建议 `/otter/autopilot/imu`，`sensor_msgs/Imu` | 传感器 frame、安装 TF、是否融合、角速度与加速度含义 |
| 航控状态与执行反馈 | 建议专用 `AutopilotStatus`、`ActuatorFeedback` 消息 | 控制权、应用序号、时间、质量、实测 RPM、故障 |
| 实艇左右执行命令 | 建议专用 `TwinThrusterCommand` 消息 | 同一报文的左右值、会话、序号、失效时间、模式 |
| 控制参考 | 建议专用 `ControlReference` 消息 | 时间轨迹、frame、stamp、完整八列数据、最终目标、任务/参考版本、运行许可 |

这些自定义消息名称为设计建议，**本次未创建对应 ROS 消息或节点**。现有 `Float32` 推力话题仅有数值，缺少组帧、期限与 ACK；通过航控适配器封装后才可构成完整实艇接口。参见 [ROS 2 Humble Odometry](https://github.com/ros2/common_interfaces/blob/humble/nav_msgs/msg/Odometry.msg) 的 pose/twist 坐标约定。

状态流建议保留最新样本并检查数据年龄；控制/权限消息应按确认语义选择 QoS。`RELIABLE` 只能约束中间件递送，不等于推进器已经应用；两者不能互相替代。

## 8. 重新设计前需要的模型与标定资料

以下主要是配置与试验辨识结果，不需要航控每帧返回：

| 资料 | 定义/用途 |
| --- | --- |
| 质量、质心、惯量 | 含典型载荷、电池位置与变化范围；纵向/转向动力学 |
| 左右推进器位置/方向 | 相对动力学参考点的三维安装坐标；控制分配矩阵 |
| 推进器映射 | 左右各自的命令—RPM—推力，正反转分开，记录电压/测试条件 |
| 死区与饱和 | 起转阈值、中位区间、最大连续输出、倒车切换限制、输出斜率限制 |
| 执行动力学 | 指令到输出/RPM 的延迟、上升时间、必要时一阶时间常数 |
| 水动力参数 | 附加质量、阻尼及适用航速范围；实艇辨识而非直接套用 Otter YAML |
| 状态估计延迟 | GNSS/INS 采样、滤波、发布延迟与精度，艏向来源及低速可用性 |
| 安装外参 | GNSS/IMU/雷达/AIS 参考点与 base_link 的转换，版本和标定日期 |

航控若能返回完整 IMU、实际 RPM、电压/电流与应用时间，可支持阶跃、定速、转圈/之字形等辨识分析。加速度字段应注明是否为包含重力效应的比力、是否去重力、在哪个坐标系；不能直接把 IMU 读数当艇体平动加速度使用。

## 9. 联调与验收清单

| 验证 | 操作/观察 | 合格判据 |
| --- | --- | --- |
| 四方向艏向 | 北/东/南/西分别输入已知真艏向 | yaw 与第 4 节表一致，跨 ±π 误差无跳变 |
| 速度坐标 | 艏向北且向北运动；另做横移、倒退 | u/v 符号和幅值符合旋转公式；不是直接复制 ENU 速度 |
| 左右推进符号 | 可控条件下分别施加小幅左右指令 | 通道对应实物左右，右侧正推更大产生约定的左转趋势 |
| 六状态与参考对齐 | 对比采样、估计、收包和参考时间 | 未用收包时间掩盖旧状态；延迟和抖动有统计记录 |
| 无效导航 | 注入 heading/position 无效、NaN、过期 | 控制器不以 0 作为替代有效测量继续闭环 |
| 指令与执行区分 | 触发限幅/限速或驱动拒绝 | 回报最终应用指令、对应 seq 与拒绝/降额原因 |
| 单侧故障 | 模拟左/右侧失效或 RPM 无响应 | 明确故障；执行约定降级/停推策略，不当成正常双侧推进 |
| 丢包/乱序/重启 | 重放旧指令、倒序包、重复包及 boot 变化 | 旧会话指令拒绝；超时保护按实测约定执行 |
| 控制权切换 | 手动接管、急停、外部模式退出 | 航控立即按优先级裁决；上位机清历史，旧指令不恢复 |
| 上位机失效 | 停进程/断链，不依赖退出回调 | 航控独立看门狗生效；有应用反馈或实物证据 |
| 跟踪与控制性能 | 同一轨迹与环境下比较当前 PID/LQR 和新算法 | 横向误差 RMS/P95/最大值、艏向误差、速度误差、控制变化率/饱和比例、计算周期/延迟、最小障碍距离 |

误差、延迟、停车距离等数值验收门槛应结合实艇尺寸、速度与任务精度确定。本文没有把仿真通过视为实艇通过，也未填造实验结果。

## 10. 可直接发给航控对接人的需求摘要

我们保留上位机左右推进器控制器，要求航控支持外部双推进器独立控制，并返回：

1. 有采样/估计时间戳和有效性标记的融合导航状态：经纬度或局部 ENU 位置、真艏向、艇体纵横向对地速度或东北向对地速度、转向角速度。
2. 航控当前控制权、执行模式、解锁、外部输出许可、手动接管、急停、失效保护及故障原因。
3. 命令接收/应用序号、实际应用时刻、左右最终执行指令、左右使能和驱动故障；优先提供左右实测 RPM、电压、电流。
4. 明确坐标轴、角度零点与正方向、单位、参考点、时间基准、输出频率、质量标记、超时和重启行为。
5. 提供推进器控制映射、倒车能力、死区、饱和/斜率约束、安装力臂与通信协议；反馈无法提供的字段及替代方式。

控制器所需的目标轨迹/期望速度/期望艏向由我方规划模块提供，不要求航控代为生成；航控需要返回的是艇体实际状态和执行状态。
