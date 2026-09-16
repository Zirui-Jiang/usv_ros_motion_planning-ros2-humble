# 为源码测试加入各 Python 功能包的导入根目录，使测试无需先安装这些源码模块。
"""Source-tree test imports, including Ubuntu 22.04's older pytest release."""
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parent
for relative in ('src/core/shared/common', 'src/core/decision_module/path_planner',
                 'src/core/decision_module/path_processor', 'src/core/decision_module/trajectory_planner',
                 'src/core/control_module/controller', 'src/core/perception_module/perception',
                 'src/core/shared/usv_navigation', 'src/core/decision_module/usv_decision',
                 'src/core/control_module/usv_control', 'src/core/perception_module/usv_perception',
                 'src/core/mission_module/usv_mission', 'src/sim_env',
                 'src/sim_env/test'):
    sys.path.insert(0, str(ROOT / relative))
