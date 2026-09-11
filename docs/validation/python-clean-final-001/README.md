# M5 后首次全新构建与安装验收

固定提交 **dfb1fb7fcb77feb868ee87ac986c70d7c4cab537**，2026-09-11 在此前不存在的 `/home/usv/usv_migration/python-clean-final-001` 完成。M5 删除提交为 `3e00ec1`，本次没有复用旧 Python 的 build/install。正式 Gazebo 18+3 由后续单独报告覆盖。

|阶段|实际结果|退出码|
|---|---|---|
|本地 clone、detach、13 包普通隔离构建|13 packages finished，colcon 报告 1 min 29 s|0|
|全新 shell 的实际安装审计|33 项检查，all_passed|0|
|全仓源码 pytest|439 passed，JUnit 8.501 s；errors/failures/skipped 均 0|0|
|独立原 C++ 库数值对照|9/9 正常用例通过；2 个语义探针差异另列|0|
|新安装定位节点 green|5 项检查通过；自有节点 PID 122923 退出 0|0|

439 是本次实际 JUnit 计数，未沿用先前估计的 433。三个保留的插件包产生 stderr，构建没有失败；原日志保留。

[总收据](acceptance-receipt.json) 包含原始文件绝对路径、字节数、SHA256、模块路径、库指纹和测试分布。[安装审计](install-audit.json)、[JUnit](source-tests.xml)、[构建请求](build-request.txt)、[构建退出记录](build-result.txt) 随报告归档。大日志及三个完整来源快照保留于 `/home/usv/usv_migration/python-clean-final-001/validation`，原文件没有截短。

## 来源与隔离

调用脚本 SHA256 为 `c0eec132b62b8b3918d41eeb681aa22587f0546e5f4d958344886b21414a9d76`，启动前已逐字节验证等于固定提交的脚本。本地 clone 使用 `--no-hardlinks --no-checkout` 后 detach；普通隔离 install，没有 symlink-install、merge-install、已有目录覆盖或删除。包顺序构建、最多 2 个编译任务。各 shell 清空旧 overlay，只 source 所需 ROS 与安装前缀。

安装审计的六个纯 Python 包和 sim_env 的实际 import origin 全部位于新工作区 install；13 个包都来自该前缀。定位与 costmap 的动态库检查见安装审计，无已移除的原导航算法库链接；插件在本审计中核对包发现路径，未额外做全部插件的 ldd。源码 pytest 另启 shell 使用新 clone 源码；这与实际安装 import 审计是两项独立检查。

构建前后 src 435 文件、scripts/validation 90 文件的全部已记录属性一致；parity/定位 green 后，这两组以及 680 个安装文件仍与构建后记录一致。构建前 install 不存在，不声称前后 install 相同。采集器排除字节码和 cache。最终 HEAD 精确匹配、detached、Git status 为空；生成的根 validation/ 按仓库锚定规则忽略。

## 独立数值对照和定位回归

[parity 报告](parity/report.md) 实际执行基线 `632734ba31478bdb833e15949287b53ae9334cf3` 的已有共享库；[ldd](parity/linked_libraries.txt) 的 path_planner、path_processor、trajectory_planner、controller 来自 baseline/install，库 SHA 见 [JSON](parity/report.json)。Python 侧使用新 clone 的纯模块；atol=1e-6、rtol=1e-5。SG 端点/航向与轨迹括区间两个语义探针差异单列；A*/Dijkstra/GBFS、LOS rollout、LiDAR 建图、全部闭环性能不在这九个 fixture 的覆盖范围。

[定位收据](localization/receipt.json) 的实际安装二进制 SHA256 为 `4c9c583ebbf9eeed0db5c05e41587ada7ce2443def653744be8afbda63a58378`，源 SHA256 `ec5f0d59cc35e6f76066add8cd0e3531d422ae3d7b6f5edfc6566bf800a36068`。domain 188、独立 namespace `localization_smoke_69f38d3f651a`，不启动 Gazebo。两个畸形并行数组被拒绝且节点存活；yaw=π/2 时世界速度 (3,4,.2) 正确转为艇体 (4,-3,.2)；重复 clock 被抑制，回退后恢复。合法输入流期间只向已核验 starttime 的自有 PID 122923 发一次 SIGINT，退出 0，事后对应 /proc 路径不存在。

[node.log 第 1 行](localization/node.log) 的实际链为 `localization_smoke_69f38d3f651a/map → localization_smoke_69f38d3f651a/odom → localization_smoke_69f38d3f651a/base_link`；这是独立随机 namespace，不冒称已在本批运行普通 otter launch。这个 green 不证明先前 shutdown -11 的根因，也不代表全部依赖进程退出健康。

## 复现命令

目标目录必须不存在。现有证据不可覆盖；再次运行须用新目录。完整构建命令也在 build-request.txt：

```bash
ROS_DOMAIN_ID=174 bash '/mnt/d/ROS2轻量仿真系统/usv_ros_motion_planning-ros2-humble/scripts/validation/build_clean.bash' \
  '/mnt/d/ROS2轻量仿真系统/usv_ros_motion_planning-ros2-humble' \
  dfb1fb7fcb77feb868ee87ac986c70d7c4cab537 /home/usv/usv_migration/python-clean-final-001
```

parity/green 分别在 `env -i` 新 shell 执行，保留 HOME/USER、标准 PATH、UTF-8，设置 PYTHONNOUSERSITE=1、PYTHONDONTWRITEBYTECODE=1、OPENBLAS_NUM_THREADS=1、OMP_NUM_THREADS=1、ROS_LOCALHOST_ONLY=1；source 后再启用 nounset：

```bash
# parity shell only
source /opt/ros/humble/setup.bash
source /home/usv/usv_migration/baseline/install/setup.bash
export ROS_DOMAIN_ID=198
python3 /home/usv/usv_migration/python-clean-final-001/scripts/validation/parity/run.py \
  --baseline /home/usv/usv_migration/baseline \
  --python-source /home/usv/usv_migration/python-clean-final-001 \
  --output /home/usv/usv_migration/results/python-clean-final-001-parity

# a separate localization shell only
source /opt/ros/humble/setup.bash
source /home/usv/usv_migration/python-clean-final-001/install/setup.bash
python3 /home/usv/usv_migration/python-clean-final-001/scripts/validation/localization_smoke.py \
  --workspace /home/usv/usv_migration/python-clean-final-001 \
  --mode acceptance --domain-id 188 \
  --output /home/usv/usv_migration/results/localization-green-python-clean-final-001
```
