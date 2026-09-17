"""Task-package tests resolve their source without changing project-wide pytest."""
from pathlib import Path
import sys


PACKAGE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE))
