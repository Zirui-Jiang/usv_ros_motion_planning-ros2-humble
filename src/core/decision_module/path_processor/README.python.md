# Python 路径处理模块

保留 ROS 包名 `path_processor`，通过 `ament_python` 安装，数值计算仅依赖
NumPy。旧 C++ 实现暂留作迁移对照，当前构建不编译它们。

```python
from path_processor import ramer_douglas_peucker, savitzky_golay

simplified = ramer_douglas_peucker(path, epsilon=0.2, max_interval=1.0)
smoothed = savitzky_golay(path, window_length=7, polyorder=2)
```

函数接受有限值 `(N,2)` 或 `(N,3)` 数组，返回独立 `(N,3)` 数组。
坐标单位为米，航向单位为弧度。空输入返回 `(0,3)` 数组。
类形式为 `RDPPathProcessor(delta,max_interval).process(path)` 和
`SavitzkyGolayPathProcessor(window_length,polyorder).process(path)`。

RDP 使用点到有限线段的距离，使用迭代栈避免深递归。`epsilon>=0`；
`max_interval=None` 表示仅简化，指定时必须为正有限值。插值沿最短角度方向，
分段数保留旧实现 `floor(length/max_interval)+1` 的语义。

SG 默认七点二阶核为 `[-2,3,6,7,6,3,-2]/21`，内部点位置与原核一致；
边界使用重复端点，首尾位置和航向固定，内部航向由平滑路径切向重算。
少于窗口长度时原样返回，修复原实现返回空路径的问题。

这两个算法均为无约束几何处理：不会读取障碍地图，不证明结果无碰撞，
也不表示满足船舶动力学。调用者必须在接受修改后路径前验证碰撞和运动约束。
默认导航链路不会自动启用 RDP 或 SG。兼容 `set_obstacles()` 仅保存障碍副本，
与旧算法一样不参与处理，不应据此推断具备避障能力。

## 带碰撞检查的直连剪枝

`ShortcutPathProcessor` 与上面的无约束 RDP/SG 不同：每次删除中间节点前，
必须调用调用方传入的碰撞判定。实现只依赖 NumPy，不反向导入规划器或 ROS。

```python
from path_processor import ShortcutPathProcessor

# is_path_clear(path) 由规划层提供，检查同一地图快照上的整条路径。
# deadline 是调用方已有的 time.perf_counter() 绝对截止时间，不是新增时长。
processor = ShortcutPathProcessor(is_path_clear)
shortened = processor.process(path, deadline=deadline)
```

也可直接调用 `shortcut_path(path, is_path_clear, deadline=deadline)`。
处理器从当前保留点向后寻找最远可直连点；先验证原路径，再尝试捷径，最后
复检完整结果。没有合法捷径时保留原相邻边；这对仍在软膨胀区中的逃逸前缀
尤其重要。最终复检失败回退到已验证原路径；超时抛出 `TimeoutError`，非法
输入或原路径不可通行抛出 `ValueError`，不返回未经验证的部分捷径。

调用方的校验函数应当无副作用，覆盖未知格、膨胀阈值、切角及地图边界规则，
并在较长的内部遍历中检查同一截止时间。独立调用时可省略 deadline；导航中
始终传入共享预算。贪心候选数最坏为 O(N²)，不保证最少拐点或全局最短路径。

输出保留首尾完整位姿，不修改输入；本功能只删冗余节点，不平滑转弯、不约束
最小转弯半径。应逐个必经航段调用，不能对多个必经点拼成的整条路线直接剪枝。
Hybrid A* 圆弧不要使用这个处理器，船体/轨迹安全检查仍由外层保留。

普通 A* 的工程接入与开关见 [路径规划说明](../path_planner/README.python.md#普通-a-路径剪枝)。

在仓库根目录执行：

```text
python -m pytest src/core/decision_module/path_processor/test -q
```
