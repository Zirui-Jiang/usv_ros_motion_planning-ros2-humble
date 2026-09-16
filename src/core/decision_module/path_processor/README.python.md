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

在仓库根目录执行：

```text
python -m pytest src/core/decision_module/path_processor/test -q
```
