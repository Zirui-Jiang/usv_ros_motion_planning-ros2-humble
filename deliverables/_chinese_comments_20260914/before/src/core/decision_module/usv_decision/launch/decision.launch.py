"""Start only the decision business module with shared navigation settings."""
from launch import LaunchDescription
from launch.actions import OpaqueFunction
from usv_navigation.launch_support import navigation_arguments, navigation_settings, navigation_node


def _launch(context):
    settings = navigation_settings(context)
    return [*settings.logs, navigation_node(settings, 'usv_decision', 'decision_node', 'usv_simulation')]


def generate_launch_description():
    return LaunchDescription([*navigation_arguments(), OpaqueFunction(function=_launch)])
