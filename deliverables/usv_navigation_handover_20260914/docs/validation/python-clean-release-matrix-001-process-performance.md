# Clean-release 正式矩阵进程资源旁路

本报告对应固定 `dfb1fb7fcb77feb868ee87ac986c70d7c4cab537`、新工作区 `/home/usv/usv_migration/python-clean-final-001`、正式输出 `/home/usv/usv_migration/results/python-clean-release-matrix-001`。只有只读 /proc 采样和完成后的统计，未改源码、安装、阈值或进程信号。完整数值、每例覆盖和原始文件 SHA 见 [JSON 收据](python-clean-release-matrix-001-process-performance.json)。

**609 个连续样本覆盖全部 18 个导航窗口**；首样本早于首 goal **8.807595315 s**。操作系统资源数据不是闭环、时延或退出健康的通过判据；另见 [正式矩阵原始摘要](python-clean-release-matrix-001/summary.json) 和 [运行审计](python-clean-release-matrix-001/run_audit.json)。

## 实际覆盖

等待 wrapper 于 UTC 2026-09-11 05:33:40.620 就绪，唯一 harness PID **123115**、starttime **8461018**，argv 明确来自新工作区。采样 PID **123069**，源采样脚本 SHA `ac6a30158f2e51d4b54f9f030a543418b3d3a1d45c2e0187c0077b3fc59657dc`；复用既有统计脚本 SHA `5f38fcca964ebbdc0a27854c0c8face6bd89d4af52fdd3efe5cc6c2d0f15ff39`，没有修改这两份算法。

首样本 mono `84610.272352245`，首个 goal 在首直航 events.jsonl 第 300 行、mono `84619.07994756`。最后样本 mono `85218.273298244`。序号严格为 0–608，无重启；实际样本跨度 **608.000946 s**。UTC 标签首尾分别是 2026-09-11T05:34:12.155059+00:00 与 2026-09-11T05:43:43.918136+00:00；持续时间只用 monotonic，不使用可能调整的 UTC。

|范围|实际覆盖与限制|
|---|---|
|PID/LQR 各 3 场景 × 3 次|18/18 的 goal_published→acquisition_finished 窗口均在采样首尾内；379 个窗口快照、361 个完全处于窗口内的 CPU 区间|
|导航窗口总墙时|380.890321 s；跨窗口边界的 CPU 区间不计入导航统计|
|goal_change、cancel|两窗口都被采样范围包围，分别 28、5 个窗口快照|
|独立 odom_dropout|有实际采样，但事件没有 acquisition_finished，故不计入同定义的完整导航/故障 CPU 窗口；最后样本比 trial_finished 早 **0.181475 s**，这段尾部未采|

样本间隔 min/P95/max 为 **0.925301 / 1.001518 / 1.060800 s**。累计 **1 次 census 读取错误**，发生于 sample 584、mono 85194.273612715，位于 cancel acquisition 结束之后。采样器只记录错误数，具体 PID/原因没有被记录；不推断错误来自被测节点，也不补估缺失值。

## 导航窗口 CPU 与内存

CPU=Δ(utime+stime)/单调时钟区间，**100%=一个逻辑核**；本机暴露 12 个逻辑核。表内均值是观测 CPU 总秒数/有效进程区间总秒数，P95 为逐区间百分比的线性插值分位数。RSS 和 VM 是 1 Hz 瞬时样本，不能作为未观测瞬时峰值。

|进程角色|CPU 均值 %|CPU P95 %|CPU 最大 %|RSS 最小–最大 MiB|VM 最大 MiB|
|---|---:|---:|---:|---:|---:|
|导航主进程|7.10|14.98|25.01|66.57–70.34|780.45|
|规划工作进程|2.80|5.00|25.01|8.75–45.50|72.51|
|LiDAR 建图|33.22|41.95|53.93|64.66–68.24|636.37|
|Nav2 costmap|0.86|2.00|2.00|35.30–36.18|1015.04|
|Gazebo|43.56|50.05|61.92|525.27–534.64|5664.18|
|验收 harness|5.28|7.00|8.00|73.83–81.74|649.25|
|定位|1.36|2.00|3.00|24.38–25.12|644.64|
|joint_state_publisher|0.76|1.00|2.00|57.19–58.19|626.37|
|robot_state_publisher|0.38|1.00|2.00|26.61–27.22|647.11|
|ROS launch|0.01|0.00|1.00|50.46–50.71|654.28|
|Python resource_tracker|0.00|0.00|0.00|10.75–11.00|18.96|

导航与规划工作进程同时观测的 CPU 求和：均值 **9.90%**、P95 **17.01%**、最大 **31.01%**；RSS 求和最大 **114.93 MiB**。这是 361 个有效快照的求和统计，RSS 可能重复计数共享页。完整运行（含启动/清理）和 fault scope 的分角色统计另保存在 JSON，不混入本表。

## 开销、退出与性能边界

采样自身墙钟耗时均值/P95/最大为 **1.800 / 2.447 / 7.104 ms**，累计 1.096334 s，占采样器墙时 **0.180%**。这是采集代码墙钟开销，不是独立测出的采样器 CPU 使用率。

原 session 3924 自然 exit 0，原始结束原因保留为 `harness_exited_or_identity_changed`；该原因本身无法给出 harness 返回码。执行任务另确认原 session 41630 自然 exit 0、21/21 功能通过、运行前后 435 源文件/680 安装文件/90 验收脚本一致；采样和 harness 的原 PID 事后均不存在。

退出健康独立记录：20 个 Gazebo 试验中 18 个试验的全部 8 个 launch 进程正常退出；PID 圆柱 01 的 gzserver 124411、圆柱 03 的 gzserver 124819 在 SIGINT 后出现 boost self-join，exit -6，各其余 7 个正常。全部定位/joint-state 正常；独立 odom 的直接退出码未记录，仍 unknown。执行方另记录 LQR straight03、circle01、circle03 各一次控制求解超时计数（control_result_timeout，80 ms LQR 求解期限）。不得将 21/21 功能通过写成全生命周期零异常。

[正常 TF 450 帧独立基准](normal-tf-throughput-001/README.md) 中的 mapping.py、mapping_node.py、scan_queue.py 三个源 SHA 与本次新源码和安装副本逐字节相同，详情见 JSON。该基准只支持 TF 预先可查条件下的吞吐能力；它不回写或替代本轮 live TF/processing 判据中的 unknown。1 Hz CPU/RSS 也不能自行把时延 unknown 改成通过。

## 原始证据与复现

原始 sidecar、统计脚本、生命周期与启动收据均位于 `/home/usv/usv_migration/results/`，以前缀 `python-clean-release-matrix-001-` 命名；它们的字节数和 SHA 收录在 JSON。原始 process JSONL SHA 为 `406488f94f1175caa5f36efef9c9a3a720b0d1d2cab87f04593743ed7b1678ac`，文件不作补样或编辑。

```bash
python3 /home/usv/usv_migration/results/python-clean-release-matrix-001-processes-summarize.py \
  /home/usv/usv_migration/results/python-clean-release-matrix-001-processes.jsonl \
  /home/usv/usv_migration/results/python-clean-release-matrix-001 \
  /home/usv/usv_migration/results/NEW-processes-summary.json
```

输出必须为新文件。统计只覆盖已记录样本、同 PID+starttime 连续观测的 CPU 区间；不补算漏过的短寿命进程、首个进程观测区间或尾部空缺。
