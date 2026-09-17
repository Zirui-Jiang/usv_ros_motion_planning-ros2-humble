# 格子中心引导末点修复：五条候选比较

本目录原样归档 `3afb25f84232e560c62599a721dc0bfbe1a21657` 的预固定 PID 候选比较：直航重复 1/2/3、转弯重复 1、单圆柱重复 1。PID preview=2.6，原 grid-interior 场景、obstacle_factor=0.25 和既定阈值不变。源时间协议在运行之前冻结，本组不是完整 PID/LQR 18+3 正式矩阵。

**候选源时间配对 5/5 通过；独立性能 5/5 通过；legacy_wall 4/5 通过。**straight02 的 legacy 比例 1.334265 仍判失败，不能用新主指标抹掉这项并列结果。其源时间比例 1.096310 接近 1.10，完整重复验证仍然必要。

| 固定配对 | Python 源时间 RMSE/m | C++ 源时间 RMSE/m | 源时间比例 | legacy 比例 | 到达时间比例 |
| --- | ---: | ---: | ---: | ---: | ---: |
| straight 01 | 0.059768396 | 0.056497947 | 1.057886152 | 1.043972886 | 0.889361702 |
| straight 02 | 0.061445813 | 0.056047836 | 1.096310161 | **1.334264831** | 1.009615385 |
| straight 03 | 0.059457250 | 0.057398993 | 1.035858763 | 1.084862336 | 0.887931034 |
| turn 01 | 0.073302248 | 0.094218160 | 0.778005512 | 0.650671641 | 0.907079646 |
| single_obstacle 01 | 0.197981216 | 0.488123639 | 0.405596452 | 0.403832333 | 0.740131579 |

全部配对按 controller/scenario/repeat 与 `baseline-grid-interior-001` 一一对应，没有重新选择基线、替换重复或删除样本。10 条双方源时间记录均 valid，最大源间隔约 0.1 s，首端覆盖缺口约为零，末端不超过 0.1 s。五条 Python 的控制耗时、扫描处理耗时与积压子项均为 pass，具体窗口、样本数、计数守恒和分位值见 report.json。性能窗口统计不冒充池化分位数。

`source_time_trapezoid` 为唯一源 odom 误差平方的离散梯形近似。窗口保留 goal 前最新 odom 和随后至 acquisition_finished 的全部 odom，路径按接收顺序冻结首条非空结果，积分端点使用实际首末源时刻，不外推。源间隔和首尾覆盖门槛固定 0.5 s，比例门槛固定 1.10。P95/max 仅描述已观测状态，不能保证连续真实误差或无碰撞。

显式 frame 契约为 `otter_fake_localization`，仅限本项目已核查的 `otter/map → otter/odom` 恒等关系。report.method 中保存旧两版源码和 frame 日志证据；3afb25f 的实际正常安装及启动 frame 证据另见 [normal-grid-endpoint-launch-frame-receipt.json](../localization-boundary-001/normal-grid-endpoint-launch-frame-receipt.json)。不能默认任意同名 frame 也恒等。

## 归档文件与来源

以下三份文件从 `/home/usv/usv_migration/results/grid-endpoint-candidate-comparison-001/` 原样复制，逐文件比对 SHA256 一致：

| 文件 | SHA256 |
| --- | --- |
| [report.json](report.json) | `b0366b4f253bdc36f1d1f6d2cdbd95e35974e856b2a1357f4f4d79f08c6fd453` |
| [report.md](report.md) | `f1ee9b2a18862de1cb488528740d84585f43ab202cbba893294d06a2be6f7479` |
| [candidate_scope.json](candidate_scope.json) | `4639a4ef0c1ed4e84ee4583f6bae2deb99faffd5121be1371f1ad4a08432f787` |

原始运行目录均位于 `/home/usv/usv_migration/results/`：

- `python-grid-endpoint-straight-candidate-001`
- `python-grid-endpoint-turn-candidate-001`
- `python-grid-endpoint-obstacle-candidate-001`
- `baseline-grid-interior-001`

逐例原始 summary 路径、SHA256 和提交号在 `report.trials[].source`；events 路径、SHA256、两种 tracking 分数及覆盖在 `report.trials[].observations`。原始大 events 不在此目录重复复制。

完整 report 读取了基线的全部 9 条 PID，因此还保留 4 对未提供本次 Python 候选的不可比项：`pairs=9, comparable_pairs=5, passed_pairs=5, not_comparable_pairs=4, all_passed=false`。**这个 false 不被改写。**`candidate_scope.json` 仅从完整报告中选取预先确定的五个候选编号，明确其有限范围，不能解释为全部正式验收通过。LQR、另外的重复、故障用例不在本候选组内。

control 的独立退出审计记录首直航系统 `joint_state_publisher` 退出 -11，其余四例进程退出检查正常，五例定位进程均 clean；该收尾异常与功能/数值比较分开保留。本目录不替代原始运行的环境、安装、before/after 指纹和退出审计。

## 复算命令

在原始结果可读的 WSL 内，使用已冻结比较器，输出必须是尚不存在且位于原始运行目录之外的新目录：

```bash
python3 scripts/validation/compare_results.py \
  --python /home/usv/usv_migration/results/python-grid-endpoint-straight-candidate-001 \
  --python /home/usv/usv_migration/results/python-grid-endpoint-turn-candidate-001 \
  --python /home/usv/usv_migration/results/python-grid-endpoint-obstacle-candidate-001 \
  --baseline /home/usv/usv_migration/results/baseline-grid-interior-001 \
  --rmse-metric source_time_trapezoid \
  --frame-contract otter_fake_localization \
  --output /tmp/grid-endpoint-candidate-comparison-recomputed
```

完整报告因保留四对未请求候选而返回退出码 1，这是预期范围结果，不能通过删除这四对或改写 all_passed 来伪造全矩阵通过。原始候选与旧失败报告保持不变。
