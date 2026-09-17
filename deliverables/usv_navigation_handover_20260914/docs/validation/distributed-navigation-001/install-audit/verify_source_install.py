"""Record SHA256 equality between the validated install and final source files."""
import hashlib
import json
from pathlib import Path

SOURCE = Path('/mnt/d/ROS2轻量仿真系统/usv_ros_motion_planning-ros2-humble')
INSTALL = Path('/home/usv/distributed-navigation-build-20260913/install')
OUTPUT = SOURCE / 'docs/validation/distributed-navigation-001/install-audit/source-install-final-002.json'


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest() if path.is_file() else None


checks = []


def compare_tree(source, installed, pattern):
    for path in sorted(source.rglob(pattern)):
        if '__pycache__' in path.parts:
            continue
        destination = installed / path.relative_to(source)
        expected, actual = digest(path), digest(destination)
        checks.append({'source': str(path.relative_to(SOURCE)), 'installed': str(destination),
                       'source_sha256': expected, 'installed_sha256': actual,
                       'passed': expected is not None and expected == actual})


for package, source in (
        ('usv_navigation', 'src/core/usv_navigation/usv_navigation'),
        ('sim_env', 'src/sim_env/sim_env'),
        ('common', 'src/core/common/common'),
        ('controller', 'src/core/controller/controller/controller'),
        ('path_planner', 'src/core/path_planner/path_planner/path_planner'),
        ('trajectory_planner', 'src/core/trajectory_planner/trajectory_planner')):
    # sim_env uses ament_cmake_python; the algorithm packages use ament_python.
    python_directory = ('local/lib/python3.10/dist-packages' if package == 'sim_env'
                        else 'lib/python3.10/site-packages')
    compare_tree(SOURCE / source, INSTALL / package / python_directory / package, '*.py')
for directory, pattern in (('launch', '*.py'), ('config', '*.yaml')):
    compare_tree(SOURCE / 'src/sim_env' / directory,
                 INSTALL / 'sim_env/share/sim_env' / directory, pattern)
compare_tree(SOURCE / 'src/core/usv_navigation_interfaces/msg',
             INSTALL / 'usv_navigation_interfaces/share/usv_navigation_interfaces/msg', '*.msg')
result = {'passed': bool(checks) and all(row['passed'] for row in checks),
          'source': str(SOURCE), 'install': str(INSTALL), 'checks': checks}
with OUTPUT.open('x', encoding='utf-8') as stream:
    json.dump(result, stream, ensure_ascii=False, indent=2)
print(json.dumps({'passed': result['passed'], 'checks': len(checks),
                  'failures': [row for row in checks if not row['passed']]}, ensure_ascii=False))
raise SystemExit(0 if result['passed'] else 1)
