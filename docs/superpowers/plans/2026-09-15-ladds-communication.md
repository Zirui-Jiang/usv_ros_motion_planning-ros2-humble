# LaDDS 与 ROS 2 最小双向通信实施计划

用户已在上一轮桥接方案后要求先实现通信。本阶段为独立测试桥，不连接导航或控制话题。

## 设计与范围

- 使用 Python rclpy + ctypes，调用用户本地 LaDDS C SDK。
- 新增功能包 `src/core/communication_module/usv_ladds_bridge`；SDK 和许可证仍在用户原目录，不纳入仓库。
- ROS 文本话题 `/ladds/tx`、`/ladds/rx`，字节话题 `/ladds/tx_bytes`、`/ladds/rx_bytes`，接收元数据 `/ladds/rx_info`。
- 用户现场确认 DDS 域 1、尽力传输、发送和接收均为字符串 `0211`；均可参数化。
- 真实 Linux SDK 建域授权失败，而 Windows SDK 授权有效。当前采用 Windows SDK + 本机 TCP 中继，保留原生 SDK 后端。
- 先验证本机 ROS→桥→SDK→独立 DDS 端及反向，再明确 Windows 联调工具验收结果。
- 不修改系统网络模式，不改许可证，不对仿真/实船控制发布命令。

## 工作项

- [x] SDK 封装：64 位句柄、二进制长度、回调生命周期、有界接收队列、启动失败回收、幂等关闭；单元测试先验证失败再实现。
- [x] ROS 节点：配置校验、文本/二进制接口、日志、关闭流程；独立构建功能包。
- [x] 真实 SDK 探针：Ubuntu 原生 SDK 加载缺依赖、预加载后授权失败；Windows SDK 建域和收发成功。当前交付采用后者。
- [x] 集成验收：真实域 1 / 0211 JSON 输入与唯一标记双向收发、正常退出和重新连接；NUL/非 UTF-8 用单元测试与本机 TCP 验证，未向用户业务主题发送任意二进制。
- [x] 编写启动/联调说明与当前环境参数；区分 SDK 替身、真实 SDK、工具人工确认三个证据层级。

## 接口约定

`LaDDSClient(...).start()` 打开资源；`send(bytes)` 发送；`receive()` 非阻塞返回 `ReceivedPacket` 或 `None`；`close()` 幂等释放。
接收包包含 `domain_id`、`topic`、`payload`、`src`、`dst`。回调内按长度复制数据并入队，由 ROS timer 处理。
默认只在 ROS 用户发布时发送 DDS 报文，不自动回声，避免形成环路。
