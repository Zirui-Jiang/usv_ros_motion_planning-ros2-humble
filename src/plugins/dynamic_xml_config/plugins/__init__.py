# 导出 XML 生成基类和障碍物生成器，供主入口通过 plugins 包统一导入。
from .xml_generate import XMLGenerator
from .obstacles_generate import ObstacleGenerator

__all__ = [
    "XMLGenerator",
    "ObstacleGenerator"
]
