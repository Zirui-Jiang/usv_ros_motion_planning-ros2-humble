# 0914 PI＋ADRC 接入说明

本次按完整五步计划接入普通导航。控制器名称为 `adrc`，速度通道为 PI，艏向通道使用交付代码的误差反馈和回归滤波；该名称沿用交付源码，不额外声称实现了 ESO。根目录 `0914controller` 四个原始文件保持原样，移植及修正记录见 [SOURCE.md](../src/core/control_module/controller/controller/adrc_core/SOURCE.md)。

## 数据接口

| 数据 | 生产代码来源 | 单位 / 含义 |
| --- | --- | --- |
| 期望艏向 | `_reference()` 返回的 `reference.theta` | rad；现有轨迹年龄＋前视选点 |
| 当前艏向 | `state.psi` | rad；由里程计姿态转换 |
| 期望航速 | `reference.velocity` | m/s；现有 LOS 前进参考 |
| 当前航速 | `state.u` | m/s；带符号船体纵向速度 |
| 实际样本周期 | 相邻有效 `state.stamp` 的差 | s；首拍使用 `sample_time` |
| 艏向滤波时间 | `measurement_stamp=state.stamp` | 真实观测时间戳 |

节点回调 dt 继续按 ROS 时钟的当前时刻减上次回调时刻计算，并用于公共输入检查。ADRC 核心的积分及差分使用有效状态的时间间隔；相同状态时间戳只保持上次指令，不再次积分/滤波。`state.r` 仍受统一有限性检查，但本控制核心没有使用它作为反馈输入；角速度、角加速度从带真实时间的艏向窗口估计。

PI 输出总纵向力 F（N），艏向核心输出偏航力矩 M（N·m）。使用现有 `allocate_force`，其中 `D=0.395 m`、单桨 `max_force=50 N`：

```text
left  = (F - M/D) / (2*max_force)
right = (F + M/D) / (2*max_force)
```

超出 `command_limit` 时两桨共同缩放以保持比例，侧推为零。适配层依据分配后的实际力/矩冻结继续推向饱和的积分，不额外推进滤波器；大艏向误差强制清积分优先。

## 状态和生命周期

- 正反多圈解缠改为累加相邻原始角度的最短角差，要求相邻采样真实转角小于 π。
- 回归按窗口内真实相对时间拟合；单样本导数为零，重复样本不入窗。
- 倒退观测本拍返回零并复位；间隔超过 `max_sample_gap=0.5 s` 按首拍重启。
- 等待、暂停、异常参考或控制失败立即清空 ADRC 历史。新任务、取消、时间回退也沿用节点复位流程。
- 同任务正常滚动参考不复位；到点采用原位置和合速度条件。SETTLING 为零推力等待。
- 基类 `on_control_hold()` 对 PID/LQR 为空操作，保留它们原来的暂停行为。

## 参数与运行

参数使用独立文件 `src/core/shared/usv_navigation/config/robots/otter/adrc_controller_params_otter.yaml`。旧 PID/LQR 配置保持，值守模块仍用原配置。ADRC 参数为启动时参数，改动后需重启。通过 `controller_config` 直接指定源码 YAML 时无需重新构建；若仍加载安装区的 YAML，则需要先把源码配置更新到安装区。`sample_time` 必须等于 `1/control_frequency`。

原核心默认限幅（8 N / 2 N·m）只作来源对照。首轮 Otter 联调配置为总力 ±40 N、速度积分贡献 ±30 N、力矩 ±10 N·m、艏向积分贡献 ±2 N·m；当时速度增益为 8 / 0.8，艏向比例、微分、积分增益为 8 / 4 / 0.03。这是历史验证配置，日常调参以源码 YAML 和运行节点回读值为准。2026-09-15 核对的源码速度增益为 8 / 4，亦为 [WSL 卡顿修复复测](validation/wsl-clock-fix-20260915/README.md) 所用参数。首轮参数调整过程和各验证层级见[验证记录](validation/adrc-0914-v1/README.md)。

本次已构建并验收的工作区为 `/home/usv/adrc-0914-20260914-mRBVAh`。在原仿真终端按 Ctrl+C 正常结束旧仿真，再打开 WSL Bash 终端运行以下命令。第一条启动不继承旧 ROS/Gazebo 安装路径的 shell，保留显示环境；随后明确加载本次安装区：

```bash
env -u AMENT_PREFIX_PATH -u COLCON_PREFIX_PATH -u CMAKE_PREFIX_PATH \
  -u PYTHONPATH -u LD_LIBRARY_PATH -u GAZEBO_MODEL_PATH \
  -u GAZEBO_RESOURCE_PATH -u GAZEBO_PLUGIN_PATH bash --noprofile --norc
cd /home/usv/adrc-0914-20260914-mRBVAh
source /opt/ros/humble/setup.bash
source install/setup.bash
ADRC_CONFIG="$(ros2 pkg prefix usv_navigation)/share/usv_navigation/config/robots/otter/adrc_controller_params_otter.yaml"
ros2 launch sim_env main.launch.py world:=ocean \
  controller_name:=adrc controller_config:="$ADRC_CONFIG"
```

此命令显示 Gazebo/RViz；使用原来的设置目标操作即可导航。仅后台运行时加 `headless:=true`。本轮测试在独立通信域执行，没有替换用户原有运行进程。

### 日常调参：直接读取 D 盘配置，无需编译

若希望先快速查看速度响应，可使用[本地速度环调参台](../scripts/control/speed_tuner/README.md)。它调用正式控制器与数值艇模，支持 Kp/Ki 滑块、网格筛选和完整候选 YAML 导出；候选通过数值初筛后，再用下述方式进入 Gazebo 验证。

在 Windows 修改 `D:\ROS2轻量仿真系统\usv_ros_motion_planning-ros2-humble\src\core\shared\usv_navigation\config\robots\otter\adrc_controller_params_otter.yaml` 的 `controller.adrc_controller` 段。速度响应主要调 `k_p_speed` 和 `k_i_speed`；`force_min/max` 是总纵向力限幅，`speed_integral_min/max` 是速度积分贡献限幅，单位均为 N。

保存后，在旧仿真终端按 Ctrl+C，等待退出；在干净的 WSL Bash 终端执行：

```bash
source /opt/ros/humble/setup.bash
source /home/usv/adrc-0914-20260914-mRBVAh/install/setup.bash
ADRC_CONFIG='/mnt/d/ROS2轻量仿真系统/usv_ros_motion_planning-ros2-humble/src/core/shared/usv_navigation/config/robots/otter/adrc_controller_params_otter.yaml'
ros2 launch sim_env main.launch.py world:=ocean \
  controller_name:=adrc controller_config:="$ADRC_CONFIG"
```

这条命令使用已构建的程序，启动时直接读取刚修改的 D 盘 YAML；不会读取安装区的旧配置副本。每次保存后重启整个 main.launch，使决策和控制进程加载同一份参数。重新设置导航目标后开始本轮测试。单独 `source` 不会更新运行中的节点，首版 ADRC 也不接受在线修改这些增益。

在另一个同通信域的 WSL 终端加载相同安装区，回读实际参数：

```bash
source /opt/ros/humble/setup.bash
source /home/usv/adrc-0914-20260914-mRBVAh/install/setup.bash
ros2 param get /otter/usv_control usv_simulation.controller.controller_name
ros2 param get /otter/usv_control usv_simulation.controller.adrc_controller.k_p_speed
ros2 param get /otter/usv_control usv_simulation.controller.adrc_controller.k_i_speed
```

回读应为 `adrc` 和你刚保存的两个数值。若自定义了 `robot_namespace`，相应替换 `/otter`。

### 使用现有脚本启动 ADRC

本机已构建的程序位于 `/home/usv/adrc-0914-20260914-mRBVAh/install`，日常修改的配置位于 D 盘源码。
在新的 WSL Ubuntu Bash 终端运行以下命令；旧仿真应先在其终端按 Ctrl+C 正常结束：

```bash
cd '/mnt/d/ROS2轻量仿真系统/usv_ros_motion_planning-ros2-humble'

USV_INSTALL_DIR='/home/usv/adrc-0914-20260914-mRBVAh/install' \
bash scripts/main.sh \
  world:=ocean \
  controller_name:=adrc \
  controller_config:="$PWD/src/core/shared/usv_navigation/config/robots/otter/adrc_controller_params_otter.yaml" \
  gui:=false rviz:=true
```

脚本会加载系统 ROS 和指定安装区，并将 launch 参数原样传入主入口。上述配置关闭 Gazebo GUI、保留 RViz；启动后按原有方式设置导航目标。
修改 YAML、保存、Ctrl+C 结束旧仿真、重跑同一命令即可刷新全部导航节点。无需因单独修改此源码 YAML 而执行 build.sh。

必须同时传入专用 `controller_config`：只传 `controller_name:=adrc` 不会自动选择 ADRC YAML，而是继续加载普通默认配置，并补入程序内 ADRC 默认值。
已完成的 WSL 双校时修复与控制器参数独立，不需要每次启动重复执行校时修复命令。

### 修改代码后重新构建

以后从修改后的源码重建时，在不含旧 overlay 的 WSL 终端、项目目录中执行。`MAKEFLAGS=-j1` 避免本项目已观察到的海面资源生成并发竞争：

```bash
source /opt/ros/humble/setup.bash
MAKEFLAGS=-j1 colcon build --base-paths src --executor sequential \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON
source install/setup.bash
ADRC_CONFIG="$(ros2 pkg prefix usv_navigation)/share/usv_navigation/config/robots/otter/adrc_controller_params_otter.yaml"
ros2 launch sim_env main.launch.py world:=ocean headless:=true \
  controller_name:=adrc controller_config:="$ADRC_CONFIG"
```

要显示 RViz/Gazebo，去掉 `headless:=true`。在同一通信域启动新仿真前结束旧仿真，避免多个节点共同发布推进器指令。恢复 PID 时加载原配置：

```bash
ros2 launch sim_env main.launch.py world:=ocean headless:=true controller_name:=pid
```

仅计算一拍、查看输入和左右桨输出：`python3 scripts/control/control_demo.py --controller adrc`。仓库内运行时默认读取专用 Otter YAML，也可用 `--controller-config` 指定文件；输出包含配置来源、有效参数和物理力/矩。独立导出目录若没有该 YAML，则使用核心来源默认参数并在输出中注明。

## 验证入口

```bash
python3 -B -m pytest -q src/core/control_module/controller/test
python3 scripts/validation/adrc_numeric_probe.py \
  --controller-config "$ADRC_CONFIG" --output /tmp/adrc-numeric-new
python3 scripts/validation/verify_distributed_install.py \
  --install "$PWD/install" --source "$PWD" --output /tmp/adrc-install-new.json
python3 scripts/validation/ros_acceptance.py --workspace "$PWD" --distributed \
  --controllers adrc --controller-config "$ADRC_CONFIG" \
  --scenario straight --repeats 1 --output /tmp/adrc-straight-new
```

输出目录必须是新的。验收脚本会回读真实控制节点算法和参数；故障用例也按选中的控制器运行，断流测试的决策与控制进程加载同一份完整配置。数值模型、生产 ROS 消息测试、安装审计与 Gazebo 结果分开记录，尚未验证实艇。

复现本轮三算法重复矩阵及 ADRC 故障测试，依次执行：

```bash
python3 scripts/validation/ros_acceptance.py --workspace "$PWD" --distributed \
  --controllers adrc pid lqr --controller-config "$ADRC_CONFIG" \
  --scenario straight turn single_obstacle --repeats 3 \
  --domain-id 199 --costmap-topic /otter/costmap_raw \
  --output /tmp/adrc-matrix-new
python3 scripts/validation/ros_acceptance.py --workspace "$PWD" --distributed \
  --controllers adrc --controller-config "$ADRC_CONFIG" --only-faults \
  --domain-id 199 --costmap-topic /otter/costmap_raw \
  --output /tmp/adrc-faults-new
python3 scripts/validation/summarize_adrc_acceptance.py \
  --input /tmp/adrc-matrix-new --output /tmp/adrc-comparison-new
```

矩阵可能耗时约半小时。验收工具为各仿真实例分配独立 Gazebo master 端口，并在结束后清理自己启动的进程。上述域 199 不应同时用于另一组验收。
