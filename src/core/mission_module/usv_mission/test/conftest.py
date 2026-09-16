# 仅为任务包测试把本包源码目录加入 Python 搜索路径，便于测试当前工作区实现。
"""Task-package tests resolve their source without changing project-wide pytest."""
from pathlib import Path
import sys


PACKAGE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE))
