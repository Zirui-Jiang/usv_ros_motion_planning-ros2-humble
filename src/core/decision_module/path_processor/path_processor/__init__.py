# path_processor Python 包入口：提供带碰撞判定的直连剪枝、RDP 简化与 SG 平滑。
"""Optional path simplification and smoothing, independent of ROS.

Ported from Yang Haodong's C++ path processors; original sources are retained.
"""

from .ramer_douglas_peucker import RDPPathProcessor, ramer_douglas_peucker
from .savitzky_golay import SavitzkyGolayPathProcessor, savitzky_golay
from .shortcut import ShortcutPathProcessor, shortcut_path

__all__ = ["RDPPathProcessor", "ramer_douglas_peucker", "SavitzkyGolayPathProcessor", "savitzky_golay",
           "ShortcutPathProcessor", "shortcut_path"]
