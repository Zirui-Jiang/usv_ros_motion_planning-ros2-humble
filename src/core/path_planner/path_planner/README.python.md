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
| planner_name | astar | astar、dijkstra、gbfs 共用八邻域搜索实现 |
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
python -m pytest src/core/path_planner/path_planner/test -q
```
