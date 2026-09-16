# Python Hybrid A* 接入实施计划

**目标：** 按当前 Python A* 的 `PathPlannerCore.make_plan(start, goal, GridSnapshot)` 接口增加 `hybrid_astar`，交付可运行代码、回归测试和切换说明。

**设计依据：** 本任务前述无人艇全局规划方案及用户“按当前系统的 astar 接入方式，用 Python 写 Hybrid A*”的明确实施请求。

**范围：** 第一版为前进限定、固定最小回转半径的几何规划；状态保留连续位置和艏向，使用位置格/朝向分箱/前一转向类型判重。包含精确圆弧基元、Dubins 目标连接、沿途矩形船体与中心软代价检查、时间/扩展预算。没有速度状态、水动力响应、动态目标或 COLREGs 搜索。

## 不变的接口与约束

- 输入世界系米/弧度，输出独立 `float (N,3)` 数组和既有 `PlanResult/AlgorithmStatus`。
- 新算法保留精确输入首尾位置与等价艏向；同格不同位姿不能原地改变朝向。
- 仅有数值 NumPy/标准库依赖，无新增 ROS 消息。
- 默认仍是 astar；原 A*/Dijkstra/GBFS 行为、软起点逃逸规则保持不变。
- Hybrid 起点超过中心软阈值时明确拒绝，第一版不复制栅格 A* 的软起点逃逸。
- 物理船体检查只针对致命/按策略未知格；中心软阈值单独处理，避免重复膨胀船体。
- 4 m 默认回转半径是待验证仿真起点，实际航速下的回转能力仍需试验标定。
- 工作树已有大量用户修改；只变更本功能文件，不恢复、提交或覆盖其他修改。

## 实施与验证

1. **Dubins 几何**（独立实现任务）
   - 新增 `graph_planner/dubins.py` 和 `test/test_dubins.py`。
   - API：`DubinsPath(modes,lengths,radius).length`；`dubins_paths(start,goal,radius)`；`advance_pose(pose,curvature,distance)`；`sample_dubins(start,path,spacing)`。
   - 先验证缺失接口失败，再验证六类曲线终点、随机固定种子、尺度/镜像、退化直线和同位反向。
2. **沿途碰撞**（独立实现任务）
   - 新增 `graph_planner/hybrid_collision.py` 和 `test/test_hybrid_collision.py`。
   - API：`HybridCollisionChecker.pose_is_clear(pose)` 和 `motion_is_clear(start,curvature,distance,deadline)`。
   - 以圆弧采样及半步角点运动上界扩张样本船体，检测端点之间的障碍及地图边界。
3. **搜索与工厂**（主任务）
   - 新增 `graph_planner/hybrid_astar_planner.py` 和 `test/test_hybrid_planning.py`。
   - 工厂按名称选择；核心对 `preserves_kinematics=True` 的路径保留圆弧采样，不执行通用直线重采样。
   - 验证非中心起终点、四分之一圆、绕障、不可通行、真实半径约束、未知策略、预算、失败无旧路径。
4. **运行层接入**（独立实现任务）
   - 共享配置及普通/任务 launch 名称校验增加 hybrid_astar；参数加入声明及校验。
   - 从 `navigation.footprint_length/width` 传入唯一船体尺寸；Hybrid 单目标从当前姿态重搜；多航点保留原始圆弧。
   - LOS 仅在 Hybrid 的 `follow_dense_path` 模式下按投影推进已走过的密集段，保留原模式。
   - 新增配置、运行层和密集 LOS 的真实行为测试。
5. **复核与交付**
   - 运行规划模块、配置、运行层、LOS 的相关回归。
   - WSL/ROS 可用时验证 launch 参数与源码安装入口；未验证的 Gazebo 闭环/实艇能力明确列出。
   - 更新模块 README，提供纯 Python 调用、项目 YAML、ROS launch 切换方式和参数含义。

## 推荐验证命令

```text
python -m pytest src/core/decision_module/path_planner/test -q
python -m pytest src/core/decision_module/trajectory_planner/test src/core/decision_module/usv_decision/test src/sim_env/test/test_configuration.py src/sim_env/test/test_hybrid_configuration.py -q
```

## 2026-09-16 完成记录

- 几何、碰撞、搜索/工厂、当前决策运行层、密集 LOS 及说明文档均已实现。
- 复核并修复三圆弧近同圆数值消减、密集路径提前停车两个问题，新增复现测试。
- 旧任务应用有独立配置及值守/绕圈规划链，因此本轮没有向其开放 Hybrid 名称；
  新参数使用独立 YAML，保持旧共用 YAML 可读取。
- Windows 318 通过/7 ROS 依赖跳过；WSL 数值及 launch 273 通过；真实 ROS 节点 13 通过。
- 在 ASCII 验证目录成功构建七个相关包，并验证安装后的规划器和配置文件。
- 详细命令、额外发现的两条旧 PID 默认值断言问题及未验证项见
  `docs/validation/hybrid-astar-001/README.md`。

测试通过不等于 Gazebo/实艇闭环达到回转半径和避障指标。
