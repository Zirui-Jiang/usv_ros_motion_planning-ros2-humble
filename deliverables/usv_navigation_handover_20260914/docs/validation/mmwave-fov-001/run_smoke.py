"""Run the read-only FOV probe against an owned, isolated Gazebo instance."""
import json
import os
from pathlib import Path
import subprocess
import sys

from ament_index_python.packages import get_package_share_directory

repo = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(repo / 'scripts/validation'))
from mmwave_smoke import close_launch, fixture_world, new_port

output = Path(__file__).resolve().parent / 'runtime'
output.mkdir(exist_ok=False)
world = output / 'fixture.world'
fixture_world(Path(get_package_share_directory('sim_env')) / 'worlds/ocean.world', world)
environment = dict(os.environ, ROS_DOMAIN_ID='194', ROS_LOCALHOST_ONLY='1',
                   GAZEBO_MASTER_URI=f'http://127.0.0.1:{new_port()}',
                   OPENBLAS_NUM_THREADS='1', OMP_NUM_THREADS='1')
command = ['ros2', 'launch', 'sim_env', 'main.launch.py', 'obstacle_sensor:=mmwave',
           'headless:=true', 'robot_x:=0', 'robot_y:=0', 'world:=' + str(world)]
with (output / 'launch.log').open('w') as stream:
    process = subprocess.Popen(command, stdout=stream, stderr=subprocess.STDOUT,
                               env=environment, start_new_session=True)
    try:
        result = subprocess.run(
            [sys.executable, str(repo / 'scripts/validation/mmwave_fov_probe.py'),
             '--duration', '25', '--output', str(output / 'probe.json')],
            env=environment, timeout=40, check=False)
    finally:
        close_launch(process)
report = {'command': command, 'domain': 194, 'launch_exit_code': process.returncode,
          'probe_exit_code': result.returncode, 'passed': result.returncode == 0}
(output / 'run.json').write_text(json.dumps(report, indent=2))
raise SystemExit(result.returncode)
