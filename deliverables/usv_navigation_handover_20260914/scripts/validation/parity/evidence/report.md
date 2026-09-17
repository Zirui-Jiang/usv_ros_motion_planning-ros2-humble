# 原C++ / Python 数值对照

正常用例通过 9/9；语义探针发现 2 处差异。

C++ 输出来自实际执行已构建基线共享库；atol=1e-06、rtol=1e-05。语义探针不计正常用例通过数。

|用例|结果|最大绝对差|范围/说明|
|---|---|---|---|
|resample_straight|一致|1.7763568394002505e-15|all_output_values|
|resample_bend|一致|8.881784197001252e-16|all_output_values|
|rdp_bend|一致|2.220446049250313e-16|all_output_values|
|rdp_straight|一致|8.881784197001252e-16|all_output_values|
|sg_straight_interior|一致|8.881784197001252e-16|interior_xy|
|sg_full_pose_probe|差异|0.2857142857142856|原SG末点未固定，航向置零；Python修复端点并重算内部航向|
|trajectory_time_affine|一致|2.7755575615628914e-16|all_output_values|
|trajectory_distance_affine|一致|2.7755575615628914e-16|all_output_values|
|trajectory_time_bracket_probe|差异|1.0|原lower_bound使用上界及下一点，Python修复为相邻包围区间；查询远离末段避免原越界|
|pid_two_ticks|一致|2.220446049250313e-16|all_output_values|
|lqr_forward_output|一致|3.0531133177191805e-15|all_output_values|

未覆盖：

- A*/Dijkstra/GBFS require Costmap2DROS lifecycle initialization; bounded tool does not instantiate them
- LQR A/B/K are private/local; only public control outputs executed, matrices not independently compared
- LOS rollouts and LiDAR mapping not included
- Trajectory nonlinear last interval is excluded because original upper bracket can access out of bounds
- PID wrap discontinuity/reverse/lateral LQR feedback semantic repairs excluded from normal cases

完整输入、原C++ TSV、编译/执行日志、ldd与提交/库SHA256见同目录。测试不启动Gazebo，不证明闭环性能。
