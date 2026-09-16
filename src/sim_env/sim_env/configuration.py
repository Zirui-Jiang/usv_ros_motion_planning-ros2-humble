# 兼容旧的配置导入路径；实际参数解析与校验集中在公共 usv_navigation 包。
"""Compatibility import for the shared onboard navigation package."""
from usv_navigation.configuration import *  # noqa: F401,F403
