# 按业务模块整理功能包的已确认需求

2026-09-14。用户已结束本轮需求澄清，并确认“整理，新任务目前不接入，保持现在的导航任务”。

## 范围与组织

三个模块分别拥有节点代码、独立 launch 和模块专属配置。算法库保持现有包名和接口，在普通模块目录下与节点包并列。普通模块目录不含 package.xml。

| 模块目录 | ROS 功能包 |
|---|---|
| src/core/perception_module | usv_perception、perception |
| src/core/decision_module | usv_decision、path_planner、path_processor、trajectory_planner |
| src/core/control_module | usv_control、controller |
| src/core/shared | usv_navigation、usv_navigation_interfaces、common |
| src/core/mission_module | usv_mission、usv_mission_interfaces |

仿真定位 fake_localization_node.cpp 留在 sim_env，其启动由 simulation_support 负责。Gazebo、船模、场景、RViz 和主组合入口继续属于 sim_env。usv_perception 管理 mapping、costmap、感知身份监测和相关 ROS 辅助入口。

usv_navigation 保留为公共支持包，业务节点及规划 worker 移出。公共包不依赖三个业务包或 sim_env；各业务包不依赖 sim_env，也不互相导入业务节点实现。共享心跳/节点运行支持避免加载无关控制和规划模块。

## 参数与接口

决策和航控共同读取的四份项目 YAML 保持字节内容、格式、默认值、加载顺序和覆盖语义，只维护一份，归 usv_navigation/config。感知的 mapping/costmap YAML 归 usv_perception/config。所有原配置路径均仍可通过对应 launch 参数显式指定；包内默认路径改到新归属。没有独有 YAML 的业务包在 config/README.md 说明共享配置入口，不复制共享 YAML 或人为增加参数。

ROS 节点名、namespace、话题、服务、消息、任务编号、参考更新、取消确认、热更新、时效判断和重启后需要重新下发目标的行为保持。主入口 ros2 launch sim_env main.launch.py 保留；旧模块 launch 转发到新的包内入口。sim_env_node 兼容入口仍转发决策节点。旧 usv_navigation 的业务 console 入口随所属包迁移，不在公共包反向导入业务包。

路径/轨迹碰撞检查保留决策侧，当前实际船体检查保留航控侧，三路推进器由独立航控节点唯一发布。A*、LOS、PID/LQR、建图算法保持。已有 usv_mission 的行为保留，只适配包路径和被迁移的支撑入口；普通导航启动不接入 mission_node。

## 本轮不包含的未来需求

用户已说明后续需要指定时间到达、指定速度航渡、定点值守和本船跟随目标船。本轮只保留任务模块的目录与已有代码，不解耦或接入新的任务管理节点，不增加 Action、任务类型、时间约束规划或目标跟随。

## 验收

检查包发现与本地依赖无环、从各业务包独立启动、四份 YAML 自定义路径和在线参数兼容、完整主入口、算法保留性及已有任务入口。独立干净构建后运行全源码/ROS 回归、分进程取消/断流/重启、实际 YAML 探针、PID/LQR 的三种 Gazebo 场景与故障回归。历史验证材料保持原样，新记录位于 docs/validation/module-layout-001。
