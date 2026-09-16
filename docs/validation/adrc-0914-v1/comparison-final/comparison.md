# PID/LQR/ADRC Gazebo 验收证据汇总

共保留 27 次试验，到达 27 次，通过 27 次，控制器身份核验一致 27 次。
本报告仅整理验收证据，不据此声明性能优势。故障试验保留为独立行，但不进入跨试验性能指标聚合。

控制耗时只提供各 trial 最终上报窗口的 P99/最大值；没有逐周期原始时序。组级字段 `max_trial_reported_p99_ms` 取 trial P99 最大值，不计算 P99 的平均值，也不解释为合并样本 P99。

## 分组

| 算法/场景 | 到达/总数/通过 | 正常指标试验 | 初始路径 RMSE 均值/最大误差 m | 最小间隙 m | 推进器 mean-abs 均值/peak-abs 最大 | max trial-reported P99/控制最大 ms | overruns 合计（有值试验数） |
| --- | --- | --- | --- | --- | --- | --- | --- |
| adrc/single_obstacle | 3/3/3 | 3 | 0.183056/0.516587 | 1.01914 | 0.040102/0.341243 | 0.345712/0.441536 | 缺失 (0) |
| adrc/straight | 3/3/3 | 3 | 0.155826/0.353287 | 缺失 | 0.0378852/0.136429 | 0.365103/0.436856 | 缺失 (0) |
| adrc/turn | 3/3/3 | 3 | 0.47164/0.924611 | 缺失 | 0.0444862/0.285272 | 0.392933/0.621838 | 缺失 (0) |
| lqr/single_obstacle | 3/3/3 | 3 | 0.353172/0.675221 | 0.999216 | 0.0908287/1 | 40.784/51.8053 | 缺失 (0) |
| lqr/straight | 3/3/3 | 3 | 0.115759/0.353146 | 缺失 | 0.0891062/0.774529 | 24.8612/79.2733 | 缺失 (0) |
| lqr/turn | 3/3/3 | 3 | 0.982257/2.19007 | 缺失 | 0.103451/1 | 51.2303/57.1154 | 缺失 (0) |
| pid/single_obstacle | 3/3/3 | 3 | 0.185887/0.58847 | 0.702736 | 0.182444/1 | 0.432536/0.481619 | 缺失 (0) |
| pid/straight | 3/3/3 | 3 | 0.159185/0.353439 | 缺失 | 0.0708104/0.965909 | 0.402933/0.5616 | 缺失 (0) |
| pid/turn | 3/3/3 | 3 | 0.12005/0.35318 | 缺失 | 0.256324/1 | 0.42698/0.498257 | 缺失 (0) |

## 每次试验

| trial | 算法/场景 | 故障 | 结果/到达/通过 | 仿真时长 s | 初始 RMSE/最大 m | 间隙 m | 推进器 peak/mean abs | 控制器 requested/actual/一致 | 控制 P99/最大 ms | overruns | config SHA |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| python_adrc_single_obstacle_01 | adrc/single_obstacle | 缺失 | completed/True/True | 38.1 | 0.169157/0.370083 | 1.01914 | 0.333172/0.0415149 | adrc/adrc/True | 0.317922/0.350759 | 缺失 | 1ac971d9244ba1ae409648b5c17dee62a6fa79087a0f939a93c5912921c98767 |
| python_adrc_single_obstacle_02 | adrc/single_obstacle | 缺失 | completed/True/True | 42.6 | 0.154669/0.372226 | 1.0309 | 0.333146/0.0409844 | adrc/adrc/True | 0.345712/0.441536 | 缺失 | 1ac971d9244ba1ae409648b5c17dee62a6fa79087a0f939a93c5912921c98767 |
| python_adrc_single_obstacle_03 | adrc/single_obstacle | 缺失 | completed/True/True | 43.3 | 0.225341/0.516587 | 1.07362 | 0.341243/0.0378068 | adrc/adrc/True | 0.345527/0.393802 | 缺失 | 1ac971d9244ba1ae409648b5c17dee62a6fa79087a0f939a93c5912921c98767 |
| python_adrc_straight_01 | adrc/straight | 缺失 | completed/True/True | 34.8 | 0.161887/0.353046 | 缺失 | 0.134895/0.0411959 | adrc/adrc/True | 0.333664/0.436856 | 缺失 | 1ac971d9244ba1ae409648b5c17dee62a6fa79087a0f939a93c5912921c98767 |
| python_adrc_straight_02 | adrc/straight | 缺失 | completed/True/True | 38.7 | 0.150588/0.353287 | 缺失 | 0.136429/0.0355988 | adrc/adrc/True | 0.33619/0.426942 | 缺失 | 1ac971d9244ba1ae409648b5c17dee62a6fa79087a0f939a93c5912921c98767 |
| python_adrc_straight_03 | adrc/straight | 缺失 | completed/True/True | 37.2 | 0.155004/0.352705 | 缺失 | 0.127035/0.036861 | adrc/adrc/True | 0.365103/0.409363 | 缺失 | 1ac971d9244ba1ae409648b5c17dee62a6fa79087a0f939a93c5912921c98767 |
| python_adrc_turn_01 | adrc/turn | 缺失 | completed/True/True | 39.7 | 0.44628/0.920999 | 缺失 | 0.282711/0.0445025 | adrc/adrc/True | 0.387793/0.432448 | 缺失 | 1ac971d9244ba1ae409648b5c17dee62a6fa79087a0f939a93c5912921c98767 |
| python_adrc_turn_02 | adrc/turn | 缺失 | completed/True/True | 39.3 | 0.528725/0.924611 | 缺失 | 0.271325/0.0461079 | adrc/adrc/True | 0.392933/0.459151 | 缺失 | 1ac971d9244ba1ae409648b5c17dee62a6fa79087a0f939a93c5912921c98767 |
| python_adrc_turn_03 | adrc/turn | 缺失 | completed/True/True | 41.5 | 0.439916/0.922363 | 缺失 | 0.285272/0.0428483 | adrc/adrc/True | 0.36768/0.621838 | 缺失 | 1ac971d9244ba1ae409648b5c17dee62a6fa79087a0f939a93c5912921c98767 |
| python_lqr_single_obstacle_01 | lqr/single_obstacle | 缺失 | completed/True/True | 18.2 | 0.323365/0.66992 | 1.0116 | 1/0.0886373 | lqr/lqr/True | 38.0416/49.9219 | 缺失 | 1ac971d9244ba1ae409648b5c17dee62a6fa79087a0f939a93c5912921c98767 |
| python_lqr_single_obstacle_02 | lqr/single_obstacle | 缺失 | completed/True/True | 18 | 0.357251/0.672053 | 1.00218 | 1/0.0946673 | lqr/lqr/True | 40.784/51.7451 | 缺失 | 1ac971d9244ba1ae409648b5c17dee62a6fa79087a0f939a93c5912921c98767 |
| python_lqr_single_obstacle_03 | lqr/single_obstacle | 缺失 | completed/True/True | 18.4 | 0.3789/0.675221 | 0.999216 | 1/0.0891816 | lqr/lqr/True | 32.8331/51.8053 | 缺失 | 1ac971d9244ba1ae409648b5c17dee62a6fa79087a0f939a93c5912921c98767 |
| python_lqr_straight_01 | lqr/straight | 缺失 | completed/True/True | 15.9 | 0.116544/0.352724 | 缺失 | 0.772607/0.0881511 | lqr/lqr/True | 24.8612/79.2733 | 缺失 | 1ac971d9244ba1ae409648b5c17dee62a6fa79087a0f939a93c5912921c98767 |
| python_lqr_straight_02 | lqr/straight | 缺失 | completed/True/True | 15.8 | 0.108847/0.353146 | 缺失 | 0.774529/0.0826649 | lqr/lqr/True | 24.5899/37.5311 | 缺失 | 1ac971d9244ba1ae409648b5c17dee62a6fa79087a0f939a93c5912921c98767 |
| python_lqr_straight_03 | lqr/straight | 缺失 | completed/True/True | 15.8 | 0.121887/0.352872 | 缺失 | 0.770739/0.0965026 | lqr/lqr/True | 21.6184/32.2385 | 缺失 | 1ac971d9244ba1ae409648b5c17dee62a6fa79087a0f939a93c5912921c98767 |
| python_lqr_turn_01 | lqr/turn | 缺失 | completed/True/True | 18.1 | 0.981027/2.17931 | 缺失 | 1/0.109965 | lqr/lqr/True | 51.2303/57.1154 | 缺失 | 1ac971d9244ba1ae409648b5c17dee62a6fa79087a0f939a93c5912921c98767 |
| python_lqr_turn_02 | lqr/turn | 缺失 | completed/True/True | 18.1 | 1.03868/2.19007 | 缺失 | 1/0.103417 | lqr/lqr/True | 43.1881/53.0693 | 缺失 | 1ac971d9244ba1ae409648b5c17dee62a6fa79087a0f939a93c5912921c98767 |
| python_lqr_turn_03 | lqr/turn | 缺失 | completed/True/True | 18.3 | 0.927063/2.18585 | 缺失 | 1/0.0969706 | lqr/lqr/True | 43.2729/53.6951 | 缺失 | 1ac971d9244ba1ae409648b5c17dee62a6fa79087a0f939a93c5912921c98767 |
| python_pid_single_obstacle_01 | pid/single_obstacle | 缺失 | completed/True/True | 23.5 | 0.180838/0.571902 | 0.830137 | 1/0.156439 | pid/pid/True | 0.3553/0.388621 | 缺失 | 1ac971d9244ba1ae409648b5c17dee62a6fa79087a0f939a93c5912921c98767 |
| python_pid_single_obstacle_02 | pid/single_obstacle | 缺失 | completed/True/True | 23.3 | 0.180509/0.553606 | 0.702736 | 1/0.210349 | pid/pid/True | 0.432536/0.481619 | 缺失 | 1ac971d9244ba1ae409648b5c17dee62a6fa79087a0f939a93c5912921c98767 |
| python_pid_single_obstacle_03 | pid/single_obstacle | 缺失 | completed/True/True | 23.4 | 0.196315/0.58847 | 0.721479 | 1/0.180545 | pid/pid/True | 0.390042/0.460146 | 缺失 | 1ac971d9244ba1ae409648b5c17dee62a6fa79087a0f939a93c5912921c98767 |
| python_pid_straight_01 | pid/straight | 缺失 | completed/True/True | 21.3 | 0.159959/0.352663 | 缺失 | 0.965909/0.0724109 | pid/pid/True | 0.372652/0.487001 | 缺失 | 1ac971d9244ba1ae409648b5c17dee62a6fa79087a0f939a93c5912921c98767 |
| python_pid_straight_02 | pid/straight | 缺失 | completed/True/True | 21.6 | 0.15672/0.353166 | 缺失 | 0.94202/0.0689197 | pid/pid/True | 0.402933/0.5616 | 缺失 | 1ac971d9244ba1ae409648b5c17dee62a6fa79087a0f939a93c5912921c98767 |
| python_pid_straight_03 | pid/straight | 缺失 | completed/True/True | 21.6 | 0.160875/0.353439 | 缺失 | 0.960167/0.0711008 | pid/pid/True | 0.349595/0.464094 | 缺失 | 1ac971d9244ba1ae409648b5c17dee62a6fa79087a0f939a93c5912921c98767 |
| python_pid_turn_01 | pid/turn | 缺失 | completed/True/True | 20.9 | 0.123449/0.35318 | 缺失 | 1/0.272398 | pid/pid/True | 0.42698/0.482461 | 缺失 | 1ac971d9244ba1ae409648b5c17dee62a6fa79087a0f939a93c5912921c98767 |
| python_pid_turn_02 | pid/turn | 缺失 | completed/True/True | 20.5 | 0.116726/0.352359 | 缺失 | 1/0.277079 | pid/pid/True | 0.390083/0.498257 | 缺失 | 1ac971d9244ba1ae409648b5c17dee62a6fa79087a0f939a93c5912921c98767 |
| python_pid_turn_03 | pid/turn | 缺失 | completed/True/True | 20.7 | 0.119974/0.352943 | 缺失 | 1/0.219495 | pid/pid/True | 0.387089/0.466149 | 缺失 | 1ac971d9244ba1ae409648b5c17dee62a6fa79087a0f939a93c5912921c98767 |
