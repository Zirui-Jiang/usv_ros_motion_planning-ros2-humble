# Hybrid A* 参数观察台

离线网页，用来在手绘海图上拧规划器参数、看仓库里真实的 `HybridAStarPathPlanner` 走出什么路。不启动 ROS、Gazebo 或仿真节点。Windows 上的 Anaconda Python 即可。

这不是 `docs/explainers/hybrid-astar-explained.html` 那种教学重写：搜索、碰撞、Dubins 连接都走 `src/core/decision_module/path_planner`。

## 打开

在仓库根目录：

```powershell
python scripts/hybrid_tuner/server.py
```

浏览器会打开 `http://127.0.0.1:8765/`。不要直接双击 `page.html`，没有本地服务就调不到规划器。

```powershell
python scripts/hybrid_tuner/server.py --port 8770 --no-browser
```

## 建议先做的三个实验

1. 打开预设 **贴岸靠泊**，点「规划」，再点「终点艏向自由 vs 固定」。如果固定艏向明显更长，多出来的就是终点 yaw 硬约束的税。
2. 把 `heuristic_weight` 从 1.2 拖到 1.0，钉住对照后再规划，看长度和耗时怎么换。
3. 勾选转向半径乘数 2× / 3×，看圆弧会不会从“最小半径折线”变成更缓的弯。

手绘障碍按 otter 全局代价地图的 InflationLayer 合成：致命 254、内切 253、再按 `252 * exp(-2.0 * (d - 1.3))` 在 3 m 内衰减。`obstacle_factor` 和 `cost_penalty` 只有在这张有梯度的图上才有意义。

## 边界

- 地图是 64×40 m、分辨率 0.5 m 的手绘图，不是 rolling costmap。
- 不含洋流、水动力、动态障碍或控制器跟踪误差。这里看起来更好的参数，仍要用 `scripts/validation/ros_acceptance.py` 在 Gazebo 里过一遍。
- 「同时跑决策链」会再走 `plan_navigation`（LOS 扫掠），时间大约翻倍。

## 文件

| 文件 | 职责 |
| --- | --- |
| `engine.py` | 膨胀、规划、扫描、三个实验 |
| `server.py` | stdlib HTTP，无第三方 Web 框架 |
| `page.html` `app.js` `style.css` | 工作台界面 |
| `test_engine.py` | 膨胀公式与空图规划合同 |
