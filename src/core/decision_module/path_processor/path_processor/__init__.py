# path_processor Python 包入口：提供RDP 路径简化与 SG 几何平滑的模块边界。
"""Optional path simplification and smoothing, independent of ROS.

Ported from Yang Haodong's C++ path processors; original sources are retained.
"""

from .ramer_douglas_peucker import RDPPathProcessor, ramer_douglas_peucker
from .savitzky_golay import SavitzkyGolayPathProcessor, savitzky_golay

__all__ = ["RDPPathProcessor", "ramer_douglas_peucker", "SavitzkyGolayPathProcessor", "savitzky_golay"]
