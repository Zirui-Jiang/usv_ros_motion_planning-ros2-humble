# WSLg / RViz 验证范围

本目录是早期初始化检查的历史记录。2026-09-11 的白模/生成失败修复已经增加
Gazebo 与 RViz 实际画面验证，见 [后续 GUI 修复记录](../otter-gui-fix-001/README.md)。
以下历史检查结果保持原样。

已实测通过：DISPLAY=:0，X11 与 Wayland socket 可用，D3D12 RTX 5060
硬件加速 OpenGL 4.2，项目原 RViz 配置加载，goal_set_plugin 与默认显示
插件实际载入，目标发布和地图/扫描/路径订阅注册，独立 RViz 正常退出 0。

尚未完成：画面核验和与实时仿真联动的可视验收。X11/Qt 窗口截图全黑；
Windows 侧一次尝试未找到可唯一归属的目标窗口，因此未执行截屏。
本目录不包含黑色图片，也不把初始化通过表述为可见画面通过。

initialization-summary.json 中 passed 指初始化检查；其中已明确
screenshot_usable=false 和 validation_scope。verification-scope.json 是
本目录的汇总判定。rviz-initialization.log 很小，保留 OpenGL 初始化及退出
信息；Windows 尝试的失败原因与自身 PID 正常退出单列。

完整原始证据：
- /home/usv/usv_migration/results/wslg-rviz-001
- /home/usv/usv_migration/results/wslg-rviz-windows-002

copied-evidence-manifest.json 保存精简副本的原始路径与 SHA256。
GUI 检查没有自行启动 Gazebo，也未改项目生产配置。
