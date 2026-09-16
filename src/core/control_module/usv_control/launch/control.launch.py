# 独立启动普通导航控制节点及推进输出适配器，复用共享的项目 YAML、命名空间与仿真时钟参数。
"""Start only the control business module with shared navigation settings."""
from launch import LaunchDescription
from launch.actions import OpaqueFunction
from usv_navigation.launch_support import navigation_arguments, navigation_settings, navigation_node


# 在 launch 上下文可用后解析参数，返回配置来源日志及本业务节点动作。
def _launch(context):
    settings = navigation_settings(context)
    return [*settings.logs, navigation_node(settings, 'usv_control', 'control_node', 'usv_control')]


# 先声明共享参数，再用 OpaqueFunction 延迟求值，确保覆盖参数已经生效。
def generate_launch_description():
    return LaunchDescription([*navigation_arguments(), OpaqueFunction(function=_launch)])
