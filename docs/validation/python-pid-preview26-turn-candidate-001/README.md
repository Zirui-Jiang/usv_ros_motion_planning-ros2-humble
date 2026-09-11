# PID 2.6 s 预瞄转弯单例候选

固定安装为 `273a1fb8d1ccccbd32cea25b6a30aad01265a473`，通过独立完整 controller YAML 将 PID `lookahead_time` 从 3.0 改为 2.6 s，其他参数不变；生产配置在该运行期间未修改。实际节点参数读回确认为 2.6 s，YAML SHA-256 与 manifest 一致。

PID turn repeat=1 自然完成并通过导航门槛：仿真耗时 20.5 s、固定参考路径 RMSE 0.1362254264 m、最大横偏 0.3527261327 m，最终位置距目标约 0.7010 m、速度低于 0.25 m/s，全局搜索一次。状态未记录 NO_PATH，未生成失败快照。八个 launch 进程正常退出。

对同编号 C++ baseline-matrix-002 turn01，RMSE 约从 0.151514 降至 0.136225 m，改善约 10.1%；时间比为 20.5/22.6≈0.90708，两项均满足 Python/C++≤1.10。这是单例候选证据，不代表三次重复或三类场景已经完成。

`trial_summary.json`、`summary.json` 与 `manifest.json` 逐字复制原始数据；原配置、实际参数读回、退出健康和 provenance 核对分别保存。前后源码、安装、验收脚本及 Git 状态完全一致。`source_index.json` 给出全部 WSL 原始文件的绝对路径、字节数和 SHA-256。完整原始事件流、地图及日志仍在 `/home/usv/usv_migration/results/python-pid-preview26-turn-candidate-001`，没有覆盖此前候选。
