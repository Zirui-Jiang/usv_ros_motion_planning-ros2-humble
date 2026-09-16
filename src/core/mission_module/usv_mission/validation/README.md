# 任务模块迁移验证

此目录是 `usv_mission` 自己的验证入口。验收启动
`ros2 launch usv_mission mission_sim.launch.py`，执行节点为
`/<boat>/mission_manager`，ROS 可执行名为 `mission_node`。
船体动力学、任务安全阈值、固定 60 秒保持窗口和 7 场景 × 3 次重复规则沿用此前验收。
迁移前的通过记录属于旧实现，不能用于宣称本次迁移已经通过。

## 构建与源码测试

在 WSL / ROS 2 Humble 环境，从源码调用：

```bash
bash src/core/mission_module/usv_mission/validation/build_missions.bash /home/usv/usv_migration/station-guard-core-001
```

输出目录必须尚不存在，且不能位于输入源码目录内。脚本保留 Git 基线并复制
`src`、`scripts` 和根测试配置，不复制旧构建、交付物或历史证据目录。
随后执行全包构建、已安装包审计以及 `pytest src scripts`。
`refresh_missions.bash` 同样要求一个全新输出目录，不覆盖冻结源码或既有证据。

## 实际闭环验收

```bash
validation=src/core/mission_module/usv_mission/validation
workspace=/home/usv/usv_migration/station-guard-core-001
bash "$validation/run_mission_matrix.bash" "$workspace" \
  --output "$workspace/results/mission-core-matrix-001" --jobs 3 --domain-id 170
```

每个子试验使用独立 ROS domain、Gazebo 端口和自有进程组，失败时停止后续调度。
验收启动后每 0.5 秒墙钟轮询一次 ROS 图，开始测量前及结束时强制检查：

- 本艇命名空间中存在且仅存在一个 `mission_manager` 节点。
- 本艇命名空间中不存在旧 `usv_simulation` 导航节点。
- 左、右、兼容保留的 lateral 指令话题，以及所有额外推进器指令话题，均只有一个来自 `mission_manager` 的发布端点。

这些是运行时观测，写入 `events.jsonl`、各试验指标与子试验清单。
任何一次图检查失败都会使该试验失败或受阻，后续恢复正常也不能抹去失败。
矩阵汇总还核对图证据、子试验编号以及源文件和安装目录前后指纹。
`--dry-run` 只生成待执行清单，从不报告实际通过。

## 两套导航回归

任务节点的兼容接口与恢复后的旧导航入口应分别运行，输出到不同目录：

```bash
bash "$validation/run_navigation_regression.bash" "$workspace" --target mission \
  --output "$workspace/results/core-navigation-001" --controllers pid --repeats 1 --faults --domain-id 210
bash "$validation/run_navigation_regression.bash" "$workspace" --target simulation \
  --output "$workspace/results/simulation-navigation-001" --controllers pid --repeats 1 --faults --domain-id 211
```

每套包含直航、转向、单障碍、目标替换、取消及里程计掉线六项。
`mission` 使用本目录的 `task_ros_acceptance.py` 并检查控制权；
`simulation` 只读调用原 `scripts` 中的普通导航工具。
算法参数键仍采用原 `usv_simulation.controller.*` 与
`usv_simulation.trajectory_planner.*`，节点参数服务地址已迁至 `mission_manager`。

## 离线检查与出图

```bash
python -m pytest -q src/core/mission_module/usv_mission/validation
python src/core/mission_module/usv_mission/validation/plot_missions.py /path/to/child-result
```

离线单测包含合成输入，不能当作实测数据。绘图只读取实际保留的日志。
任务工具使用 `task_*` 名称和显式兄弟文件加载，与根目录普通验证工具共同收集时不共享同名模块缓存。
源码指纹遍历 `src` 一次，因此任务算法及本目录的 Python、Bash、YAML、JSON 配置均被纳入；
同时覆盖被复用的 `scripts` 以及所有已安装包的源码、配置、接口和类型支持文件。
