"""Source-tree test imports, including Ubuntu 22.04's older pytest release."""
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parent
for relative in ('src/core/common', 'src/core/path_planner/path_planner',
                 'src/core/path_planner/path_processor', 'src/core/trajectory_planner',
                 'src/core/controller/controller', 'src/core/perception', 'src/sim_env'):
    sys.path.insert(0, str(ROOT / relative))
