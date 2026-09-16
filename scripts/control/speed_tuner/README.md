# Otter 速度环调参台

在 Windows 本地浏览器中调 `k_p_speed` 和 `k_i_speed`，无需启动 ROS 或 Gazebo。后端直接调用项目正式 `ADRCController` 和 `common.step_dynamics`，保留物理推力分配、积分限幅与抗饱和。

## 启动

在资源管理器双击上一级的 `start_speed_tuner.bat`，浏览器会打开 `http://127.0.0.1:8767`。启动窗口保留运行，按 Ctrl+C 关闭。

也可以在仓库根目录的 PowerShell 运行：

```powershell
python -B scripts/control/tune_speed.py
```

当前电脑已具备 NumPy、PyYAML、Flask。其他电脑首次运行若提示缺少模块，先执行：

```powershell
python -m pip install -r scripts/control/speed_tuner/requirements.txt
```

端口被占用时使用 `--port 8768`；不自动打开浏览器用 `--no-browser`。只监听本机 `127.0.0.1`，页面不依赖 CDN 或网络字体。Linux 也可以用 `python3` 启动同一工具；首轮交付在 Windows 验证。

## 一次完整调参

1. 打开后先看源配置基准。当前专用 YAML 的速度增益为 `8 / 0.8`，并非 PID 的归一化增益。
2. 修改 Kp、Ki，或拖动滑块。停止变动约 0.38 s 后自动计算，比较 90% 响应时间、稳定时间、超调和末段误差。输入不合法或仍在计算时，导出不可用。
3. “固定当前对照”可以留下额外一组曲线。源配置始终作为灰色虚线；换参考速度、时长或场景时清除不兼容的固定对照和筛选结果。
4. “开始筛选 25 组”在指定矩形范围内做 5×5 等距网格，展示评分靠前的 8 组。“采用”仅更新界面数值，继续计算响应。
5. 分别检查加速、降速、到点停推；可用“前 20 秒”查看初期响应，P/I 分量展示积分如何补足推力。鼠标或读数时间滑块可读原始采样值。
6. 点击“导出候选 YAML”。默认保存到 `scripts/control/speed_tuner/exports/`，每次创建唯一文件名，并保存同名 JSON、CSV。原始 YAML 不会被覆盖。
7. 停止旧仿真，在已加载新 ADRC 安装区的 WSL 终端运行导出弹窗提供的命令，重新设置目标。用原有 Gazebo 直线、转弯、单障碍和故障验收验证候选。

导出的 YAML 是完整项目控制配置，PID/LQR 以及 ADRC 其他参数都保留，只改 `k_p_speed`、`k_i_speed`。JSON 记录数值结果、完整模型、采样周期、场景、源配置及实现文件哈希；CSV 保存逐样本曲线。导出时会重新计算当前候选，避免把旧图表错配到新参数。

若在外部编辑了源 YAML，点击“重新读取配置”。旧界面带来源校验，不能继续把已变化的文件当成旧基准。若修改 Python 算法或模型代码，需要关闭并重启本工具。

## 场景与指标口径

三个场景都从静止开始，期望艏向固定为零。未运行 LOS、规划器、ROS 状态机或 Gazebo。

| 场景 | 数值输入 | 主要指标窗口 |
| --- | --- | --- |
| 加速 | 0 → 固定参考速度 | 全程 |
| 降速 | 前半程跟踪参考，中点降到原参考的 30% | 从中点切换开始 |
| 到点停推 | 前半程跟踪参考，中点输出零并清积分 | 从中点切断推力开始 |

“到点停推”模拟当前生产节点的 SETTLING 推力策略，只对零推力后的艇体响应进行数值检查；不包含目标位置判定、重新入轨或主动倒桨制动。

- **T90**：从本阶段开始的实际速度，首次完成向阶段最终参考速度的 90% 变化所需时间；未达到返回空值，界面显示“本次未达到”。
- **稳定时间**：进入 `max(0.02 m/s, 5% × 阶段实际初速到最终参考的差值)` 误差带，并保持到试验结束至少 1 s。不是无限时间稳定性证明。
- **超调**：沿阶段变化方向超过最终参考的最大幅度，除以阶段速度变化幅度。降速和加速分别按对应方向计算。
- **末段误差**：本阶段最后至多 10 s 的速度绝对误差平均值。
- **IAE**：整场试验中速度绝对误差对时间的离散积分，单位 m。
- **输出触及限幅**：实际总力触及纵向限幅或桨指令触及分配限幅的控制样本比例；不等同于内部每次抗饱和分支执行比例。
- **筛选评分**：`IAE/(时长×参考速度) + 0.5×超调比例 + 0.25×T90/阶段时长 + 0.1×饱和比例`。T90 缺失时评分中取阶段时长。此评分只是透明的初筛规则，不是全局最优控制器，也不自动验证多场景鲁棒性。

曲线没有平滑或抽样丢点。状态位于标注时刻；推力/P/I/参考对应从该时刻开始的控制区间。最后一行保存末状态，并延续最后一个控制区间的命令供绘图。

模型采用实际 `dynamics_otter.yaml` 的有效惯性、阻尼和三自由度耦合，RK4 子步不超过 0.05 s；控制周期读取专用 YAML，当前为 0.1 s。默认数值场景不能代表风浪、传感器噪声、执行器延迟和实艇非线性。应先用它排除明显太慢、超调或持续饱和的参数，再做 Gazebo 验证。

## 文件结构与验证

| 文件 | 职责 |
| --- | --- |
| `../tune_speed.py` | 启动命令与本机端口 |
| `engine.py` | 参数校验、正式核心调用、艇模推进、指标、网格筛选、候选导出 |
| `web.py` | 本地 HTTP 接口与静态页面 |
| `static/index.html` | 中文界面结构 |
| `static/style.css` | 视觉、布局与响应式样式 |
| `static/app.js` | 滑块联动、请求状态、图表和导出交互 |
| `../test_speed_tuner.py` | 数值、指标、配置保护和 HTTP 测试 |
| `browser_smoke.py` | 可选 Playwright 真实浏览器验收 |

```powershell
python -B -m pytest -q scripts/control/test_speed_tuner.py
# 另一个终端先启动 --port 8766 --no-browser；浏览器测试会创建候选文件：
python -B scripts/control/speed_tuner/browser_smoke.py --url http://127.0.0.1:8766 --output docs/validation/speed-tuner-v1/browser
```

浏览器测试需要可选的 Playwright 和 Chromium，本机已具备；使用调参工具不需要 Playwright。

现成通用工具可参考 [MATLAB PID Tuner](https://www.mathworks.com/help/control/ref/pidtuner-app.html) 和 [Python Control PID 设计器](https://python-control.readthedocs.io/en/stable/generated/control.rootlocus_pid_designer.html)。本工具选择复用项目实际 Python 核心，使输出单位、积分限幅和控制分配与当前系统一致。
