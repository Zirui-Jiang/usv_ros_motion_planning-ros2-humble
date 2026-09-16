# 兼容旧的规划执行器导入路径；实际异步规划逻辑位于 usv_decision.runtime。
"""Compatibility import for the decision module's planning runtime."""
from usv_decision.runtime import *  # noqa: F401,F403
