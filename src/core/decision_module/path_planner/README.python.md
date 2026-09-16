# Python 路径规划模块

保留 ROS 包名 `path_planner`，通过 `ament_python` 安装。数值模块仅依赖
NumPy 和 `common`；ROS 参数、消息、TF、可视化及服务由外部适配层处理。
旧 C++ 源码和 CMakeLists.txt 暂时保留用于迁移审查，当前构建类型不会编译它们。

```python
from common.models import GridSnapshot
from path_planner import PathPlannerCore

planner = PathPlannerCore({"planner_name": "astar", "enable_resample": True})
result = planner.make_plan((0.5, 0.5, 0.0), (4.5, 4.5, 0.0), grid_snapshot)
```

输入起终点为 `(x, y, yaw)`，单位为米、弧度，必须与栅格坐标系一致。
`GridSnapshot.data` 使用 `[y,x]` 布局和 Nav2 原始代价值 `0..255`，不是
`OccupancyGrid` 的 `-1..100` 数值。路径为独立的 `float (N,3)` 数组，位置
保持原实现的网格中心语义，首尾航向取起终点输入；同一格时返回单个目标航向。
搜索错误不会发布或缓存上一条路径，失败结果路径为空。

| 配置 | 默认值 | 含义 |
| --- | --- | --- |
| planner_name | astar | astar、dijkstra、gbfs 共用八邻域搜索；hybrid_astar 使用下述连续位姿搜索 |
| obstacle_factor | 0.5 | 障碍阈值为 `254 * obstacle_factor`，范围 `(0,1]` |
| unknown_is_obstacle | true | 是否将值为 255 的未知格视作障碍 |
| allow_corner_cutting | true | 保留原对角移动规则；false 禁止从两个障碍角之间穿过 |
| outline_map | false | 在本次搜索的掩码上封闭四周边界，不修改地图 |
| timeout | 1.0 | 搜索时间上限，秒；含地图预处理，不含后续路径检查和重采样 |
| max_expansions | 200000 | 搜索展开节点上限 |
| enable_resample | false | 是否重采样输出路径 |
| resample_spacing | 0.5 | 等弧长采样间距，米；兼容旧名 resample_ratio |

`SUCCESS` 表示找到路径；阻塞端点或不连通返回 `NO_PATH`；端点越界、格式
错误返回 `INVALID_INPUT`；搜索时间或展开数耗尽返回 `TIMEOUT`。
配置错误在构造时抛出 `ValueError`。状态中的 `elapsed` 是本次 make_plan
总时长，`expanded` 为真实展开数。网格快照时间戳的新鲜度由调用者管理。

`prune(path,state)` 保留最近航点及之后路径，等距离时选择最后一个，兼容
原裁剪策略；可选 `prune_distance` 按路径弧长保留一段已走路径。
`resample(path,spacing)` 保留首尾姿态，内部航向按三点圆弧切向计算。
重采样可能跨越原折点，因此 `make_plan` 使用 `common.math_helper.path_is_clear`
对原路径和候选路径进行分段采样碰撞检查；候选失败时返回原网格路径。
该检查是半格采样检查，不代表船体扫掠体或连续几何碰撞证明。

迁移回归以成功状态、端点、路径可行性和代价为准；堆的平局顺序固定，但
不保证与 C++ 选择同一条等代价路径。Dijkstra/A* 在有效端点场景保持原
代价语义。原有索引跨行、非正重采样间距、阻塞端点处理按修复后契约验收。

在仓库根目录执行：

```text
python -m pytest src/core/decision_module/path_planner/test -q
```

## 无人艇 Hybrid A*（Python）

通过相同的工厂和 `PathPlannerCore` 选择 `hybrid_astar`，默认算法仍是 `astar`。
这是前进限定、固定最小回转半径的全局几何规划器，使用连续 `(x,y,yaw)`
位姿、直线/左圆弧/右圆弧动作和 Dubins 目标连接。没有倒车、原地旋转、
速度状态、水动力转向延迟、动态目标或 COLREGs 决策。

```python
import numpy as np
from common.models import GridSnapshot
from path_planner import PathPlannerCore

grid = GridSnapshot(np.zeros((120, 120), dtype=np.uint8),
                    resolution=0.25, origin_x=-5.0, origin_y=-5.0)
planner = PathPlannerCore({
    "planner_name": "hybrid_astar",
    "minimum_turning_radius": 4.0,
    "footprint_length": 2.2,
    "footprint_width": 1.2,
    "timeout": 2.0,
})
result = planner.make_plan((0.0, 0.0, 0.0), (4.0, 4.0, np.pi/2), grid)
if result.success:
    path = result.path  # N×3，米/弧度，按真实运动段采样
else:
    print(result.status, result.message)
```

**与八邻域 A* 的区别：** Hybrid 保留真实输入端点，不投影到格中心。
同格不同姿态仍然需要可前进的连接，不会直接在原地覆盖终点航向。
位置格、朝向分箱和前一转向类型只用于搜索判重，节点位姿保持连续。
它严格拒绝超过中心软阈值的起点，第一版不使用 A* 的软膨胀起点逃逸。

| Hybrid 配置 | 默认值 | 含义 |
| --- | --- | --- |
| minimum_turning_radius | 4.0 | 最小转弯半径，米；待验证仿真起点，非实艇测量值 |
| angle_bins | 72 | 艏向分箱数；72 对应每箱 5°，至少 8 |
| heuristic_weight | 1.2 | 搜索优先级 `g+w*h` 的权重，至少 1；1.2 减少长航段展开，可能略增路径长度；1 恢复原排序 |
| motion_step | 1.0 | 搜索基元长度下限，米；实际至少为一格对角线长度 |
| collision_step | 0.2 | 碰撞/输出采样的最大间隔，米；结合地图分辨率和转角进一步细分 |
| analytic_expansion_max_length | 20.0 | 直接连接目标的完整 Dubins 曲线最大长度，米 |
| non_straight_penalty | 1.05 | 圆弧长度的乘数，至少 1 |
| change_penalty | 0.2 | 每次转向类型改变的附加代价，非负 |
| cost_penalty | 0.0 | 沿途归一化中心栅格代价的积分权重，非负 |
| footprint_length / footprint_width | 2.2 / 1.2 | 数值接口的矩形船体长宽，米，参考点为矩形中心 |
| footprint_padding | 0.0 | 船体各侧额外安全余量，米 |
| reuse_max_deviation | 2.0 | 实测船位到缓存路径投影的位置误差上限，米；超出后全局重搜 |
| reuse_max_heading_error | π/3 | 实测艏向与路径切线方向差上限，弧度；超出后全局重搜 |
| reuse_search_distance | 10.0 | 只在剩余路径前10米内找进度投影，避免跳到自交路线后段 |

通用 `obstacle_factor`、`unknown_is_obstacle`、`outline_map`、`timeout` 和
`max_expansions` 同样生效。矩形船体与完整致命栅格相交时拒绝；未知格按
策略处理。软代价带只约束中心，不再叠加整个船体。圆弧沿途采样使用半步
中心/角点运动上界的额外裕量覆盖采样间隙，因此非常贴边的路径可能保守失败。
默认第一条通过检查的解析连接终止搜索，不保证连续空间全局最优或分辨率完备。

### 运行层切换

现有入口接受 `planner_name:=hybrid_astar`，其余启动参数沿用当前任务：

```bash
ros2 launch sim_env main.launch.py planner_name:=hybrid_astar
# 分布式部署仅启动决策侧时：
ros2 launch usv_decision decision.launch.py planner_name:=hybrid_astar
```

独立示例配置为 `src/core/shared/usv_navigation/config/path_planner/hybrid_astar_planner_params.yaml`。
通过现有 `planner_config:=配置文件绝对路径` 载入，可在 `path_planner` 下调整相应参数。
只需切换算法时也可将原 `common_planner_params.yaml` 的顶层 `planner_name` 改为
`hybrid_astar`；未指定的新参数使用共享默认值。原共用 YAML 保持旧任务应用可读取。
ROS 运行层从 `navigation.footprint_length/width` 统一传入船体尺寸。
新规划参数是启动参数，需要重启后生效。本次接入的是当前决策节点的单目标/多航点
链路；独立旧 `usv_mission` 的值守/绕圈应用仍使用其原有算法选择。

Hybrid 返回已采样的真实圆弧，`enable_resample` 不会再对其应用通用直线重采样；
调整输出密度使用 `collision_step`。运行层对新全局路线尝试切向一致、整段无障碍的
直线快捷连接；转弯弧和必经点保留。正常航行复用全局路线，在原几何上投影、裁掉已走
前缀，并按最新地图复查剩余中心代价和船体；首次目标、剩余路段失效、目标变化或偏差
过大才全局重搜。多航点按各自必经点切分缓存，不能因快捷连接跳过必经点。
任务层根据实测船位进入航点到达盘、确认进入下一航段时，清除已完成阶段缓存一次，
从当前实测位姿建立新阶段航路；过渡期间保留已经验收的局部参考，后续正常周期重新复用。
这避免船已确认完成旧点、几何投影却仍位于旧入航段而反复回贴旧参考。

`navigation.planning_frequency` 是局部参考更新/缓存复检频率（默认5 Hz），不是
强制全局重搜频率。每次已验收的局部更新都及时发布发生变化的剩余path，船后方已走过的
线段与该周期的裁剪结果同步消失；路径未变化不重发，也不先清空再画。移除了此前额外的
1 Hz显示限制及 `navigation.global_path_publish_frequency` 参数。
新全局搜索结果、从空显示恢复、失效或取消时同样立即更新/清空。
这套显示规则由公共决策节点执行，对A*和Hybrid A*一致，不改变两者搜索触发策略。
global_path采用可靠TransientLocal、深度1，后订阅的RViz应选择
`Transient Local`才能读取已发布的最后一条航路；源码RViz配置已设置。

LOS 从每次最新实测船位生成带新state.stamp的局部参考，并独立检查预测船体。
缓存全局路径起点位于参考曲线，不能把小跟踪偏差强行拼成全局折角；局部轨迹负责
接近并跟踪参考。LOS仍生成自己的时间参考，因此全局路径的几何半径约束不是最终
LOS参考或实船轨迹的严格半径保证。密集路径的 LOS 停车同时检查终点距离与剩余路径长度，防止接近
终点时截断尚未走完的绕行。现有控制器/任务完成仍按位置与速度判定，不要求精确
终点艏向。运行层继续执行预测轨迹船体检查，闭环跟踪能力需另做仿真验证。

### 无需 ROS 的运行示例

在仓库根目录运行（仅依赖 Python 和 NumPy）：

```text
python src/core/decision_module/path_planner/examples/hybrid_astar_demo.py
python src/core/decision_module/path_planner/examples/hybrid_astar_demo.py --radius 3 --output hybrid_path.npz
python -m pytest src/core/decision_module/path_planner/test -q
```

示例规划空地图上的四分之一圆，输出状态、节点数和路径长度；可选保存路径与地图，
其中 `path` 为 `(N,3)`，`data` 为原始代价矩阵。绕障与失败场景由模块测试覆盖。
