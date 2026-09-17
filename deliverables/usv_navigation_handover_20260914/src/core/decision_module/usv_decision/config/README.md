# usv_decision 参数配置

本模块当前没有独有 YAML。决策与航控共同读取的四份项目配置只维护一份，安装在 `usv_navigation/config`：

- `robots/otter/dynamics_otter.yaml`
- `robots/otter/controller_params_otter.yaml`
- `trajectory_planner/trajectory_planner_params.yaml`
- `path_planner/common_planner_params.yaml`

独立入口 `ros2 launch usv_decision decision.launch.py` 使用相同的共享加载器。可通过 `dynamics_config`、`controller_config`、`trajectory_config`、`planner_config` 显式指定已有项目 YAML；格式、默认值、覆盖顺序和校验规则保持。

已有在线参数与导出接口仍属于决策节点 `/<namespace>/usv_simulation`，接受的热参数通过导航参考同步到航控；本目录不复制共享 YAML，也不增加新的参数。
