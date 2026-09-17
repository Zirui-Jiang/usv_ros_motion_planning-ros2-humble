# M5 旧 C++ 算法源码清理

按用户确认的原包 Python 迁移范围，已精确移除78个被替换的C++文件：六个纯Python
包的旧算法src/include/CMake，以及sim_env旧导航调度实现。没有递归删除目录。
完整逐文件路径、删除前SHA/字节数、时间与结果见 [receipt.json](receipt.json)。

执行前两次核对了21个分组中的78个唯一文件，内容与
[原清单](../baseline/m5-old-cpp-removal-manifest.json)全部相符；主任务紧邻删除前
再次检查138个唯一文件/祖先路径，确认不存在ReparsePoint，全部待删路径位于
指定Windows工作区内且受Git跟踪。全部检查完成后，使用PowerShell
`Remove-Item -LiteralPath`逐个处理。结果为78/78成功，receipt状态`completed`。

原`main`和`cpp-baseline`仍为`ed4ed1803b1d520e19712df82f54ffac6ff313b3`。
Python数值实现、配置、模型、世界、材质、Gazebo与RViz插件均保留；sim_env的
`fake_localization_node.cpp`、`costmap_support_node.cpp`及混合构建CMake保留。
不属于活跃导航链的通用曲线实现可在原始基线中查看。

剩余测试和构建无需本工作区的已删除C++文件。正常数值对照从独立`--baseline`
读取原源码并链接它的安装库。一个指向旧LOS文件的Markdown链接已改为明确的
`git show cpp-baseline:...`来源，原始诊断JSON及SHA未改。

清理依据是用户明确采用三次场景均值的10%门槛，既有完整矩阵具备18条导航及3条
故障功能证据；可形成完整三重复基线的PID直航/转弯均值均在门槛内。其他基线不足
范围和旧逐次失败不改判。新统计及重新构建/完整复验按提前提交的
[收尾协议](../scenario-mean-release-protocol.md)执行。

本记录只证明源码清理完成，不能代替清理后的新安装构建或闭环验收。
