# LaDDS 0211 与 ROS 2 通信验证

2026-09-15，本机 Windows + WSL2 Ubuntu 22.04 / ROS 2 Humble。

## 结论

**Windows LaDDS SDK → 本机 TCP → ROS 2 双向通信通过。** 用户正在发送的 `0211` JSON 已在 ROS 中原样收到，ROS 发出的唯一测试标记被独立 DDS 订阅端收到。退出并重新连接后，独立 DDS 测试端还返回了关联标记，ROS 再次收到，排除了仅观察本地回环的假阳性。

当前配置：LaDDS 域 1、主题字符串 `0211`、QoS best_effort、加密未启用；Windows 中继 `172.27.128.1:17880`，WSL `172.27.138.43`；ROS 域 196、ROS_LOCALHOST_ONLY=1。未修改 WSL NAT 模式或防火墙。IP 仅代表本次环境，重启后可能变化。

## 真实数据格式

- 第一轮独立 Windows 抓包 15 秒收到 15 条报文，每条 638 字节，可严格按 UTF-8 解码并解析为 JSON。
- 顶层 `head` 和 `content`；`head.packageSeq=3`、`sendPlatform=1000`、`recvPlatform=1001`；内容有 `travelId=53`、`taskType=16` 和三个经纬度航点。
- JSON `head.time` 为 `2026-09-02 13:44:49.573`，属于包内字段，不是本次接收时间。
- SDK 外层回调的 `src=0,dst=0`，与 JSON 内 `sendPlatform/recvPlatform` 是不同字段，桥不自动互相替换。
- 任务类型枚举、航点 type 和 speed 单位尚未由业务协议确认；本轮不执行任务或坐标转换。

[格式化单包样本](sample-0211.json)；[原始抓包及字节](capture-0211.json)。

## 双向验收记录

| 检查 | 结果 | 证据 |
| --- | --- | --- |
| 用户报文到 ROS，12 秒窗口 | 12 条业务 JSON；另有 3 条本轮标记；15 条文本与对应原始字节一致 | `ros-live-probe.json` |
| ROS 到独立 DDS 订阅端 | 3 次唯一 probe_id 均被收到 | `peer-capture-0211.json` |
| ROS 节点退出 | 对本次域 196、独立安装路径的 PID 4696 发 SIGINT，退出码 0 | 本轮终端记录 |
| 使用交付脚本重新启动 | 重新连接同一 Windows 中继，节点 PID 4991 | 本轮终端记录 |
| 重连后独立 DDS 往返 | 独立端收到 3 次 ROS 标记，回复 1 次；ROS 收到关联回复，4 条文本/字节匹配 | `roundtrip-peer-warm.json`、`ros-roundtrip-warm.json` |

首次发送唯一标记：`9360c549-9cd9-40cf-8c1d-3b8d285da7b7`。
重连往返唯一标记：`6d52ee2a-7cfa-42a3-a28f-ce771f3b0169`；DDS 回复 ID：`4b128825-d545-4fc7-a4bb-5c6722b6d035`。

## 非通过窗口和判断边界

- `ros-reconnect-probe.json` 与 `peer-after-reconnect.json` 是业务发送暂未观测到数据的窗口，两端均为 0 条；不能记为接收测试通过。
- `ros-roundtrip-after-reconnect.json` / `roundtrip-peer.json` 为同时启动独立 DDS 端时的尝试：ROS 本地看到自己的 3 个标记，独立端未收到，故返回失败；未将其当互通通过。
- 随后先启动独立 DDS 测试端，让其进行发现，再发测试标记，完整往返通过。该对照支持初期发现时序的解释，不给出所有环境固定的发现耗时保证。Best Effort 首包可能丢失；要用独立端实际接收判定成功。

## SDK 平台检查

- Windows：联调工具目录的 LaDDS.dll，版本 1.0.3；创建句柄成功，实际字节收发和销毁均成功，使用原有授权。
- Ubuntu：所给麒麟 x64 SDK 直接加载报 `undefined symbol: SSL_CTX_free`；单独进程预加载系统 libssl.so.3 后能读版本，创建句柄仍返回 0 并报告 `linence failed!`。从原有授权目录运行也相同。
- 没有安装替代 OpenSSL、没有修改或复制许可证、没有通过补丁改变授权检查。Linux 原生后端的运行授权和完整兼容性仍待厂商支持。

## 代码检查

- Windows Python 3.12：通信包 66 项 + 模块布局/依赖 10 项，合计 **76 passed**。
- Ubuntu Python 3.10 / colcon：通信包 **65 passed、1 skipped**；跳过项为 Windows 专属 `os.add_dll_directory` 生命周期检查。CTest 包装检查也通过，因此 colcon 汇总显示 67 项、0 错误、0 失败、1 跳过。
- Ubuntu 模块布局/依赖：**10 passed**。
- 初次 Ubuntu 测试暴露 SO_LINGER 测试结构平台差异，已修为 Windows 两个 short、Linux 两个 int，并重跑通过。
- 独立 `colcon build` 通过。安装区为 `/tmp/usv_ladds_20260915/install`，未改现有仿真安装区。
- Python 编译、PowerShell 语法及 Bash 语法检查通过。

单元测试的 SDK 替身仅验证适配层；真实厂商互通证据是上面的 Windows 抓包和 ROS 报文。未向本次用户业务主题发送任意非 UTF-8 数据；这些边界在替身 SDK 和真实 loopback TCP 的测试中覆盖。

## 复现入口

完整命令见 [通信桥 README](../../../src/core/communication_module/usv_ladds_bridge/README.md)。
抓包脚本 `scripts/validation/ladds_capture.py` 只订阅；`ladds_ros_probe.py` 和 `ladds_roundtrip_peer.py` 仅在显式运行时发通信测试标记。中继和 ROS 节点默认不自动发包，也不转发到导航/控制话题。

本目录原始报文仅用于用户本地联调，未上传外部服务。
