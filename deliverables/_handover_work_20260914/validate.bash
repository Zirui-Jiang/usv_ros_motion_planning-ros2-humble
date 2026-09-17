#!/usr/bin/env bash
set -eo pipefail
work="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
kit="$(cd -- "$work/../usv_navigation_handover_20260914" && pwd)"
out="$kit/handover/validation"
mkdir -p "$out"
test_root="$(mktemp -d /tmp/usv-handover-20260914-XXXXXX)"
cp -a "$kit/src" "$kit/scripts" "$kit/docs" "$kit/conftest.py" "$kit/pytest.ini" "$kit/README.md" "$kit/CONTEXT.md" "$test_root/"
source /opt/ros/humble/setup.bash
source /home/usv/module-layout-20260914/install/setup.bash
export USV_DISTRIBUTED_INSTALL=/home/usv/module-layout-20260914/install
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1
export ROS_DOMAIN_ID=231 ROS_LOCALHOST_ONLY=1 PYTHONDONTWRITEBYTECODE=1
export PYTHONPATH="$test_root/src/core/mission_module/usv_mission:${PYTHONPATH:-}"
export PYTEST_ADDOPTS='-o testpaths=src/common/test'
cd "$test_root"
python3 - "$out" <<'PY'
import json, os, pathlib, platform, sys, numpy, yaml, pytest
out = pathlib.Path(sys.argv[1])
report = {'test_root': str(pathlib.Path.cwd()), 'python': sys.version,
          'platform': platform.platform(), 'numpy': numpy.__version__,
          'pyyaml': yaml.__version__, 'pytest': pytest.__version__,
          'command': ['python3', '-m', 'pytest', 'src', 'scripts', '-q', '-rs', '--confcutdir=.',
                      '-p', 'no:cacheprovider', '--junitxml=' + str(out / 'source-tests.xml')],
          'environment': {k: os.environ[k] for k in ['USV_DISTRIBUTED_INSTALL', 'ROS_DOMAIN_ID',
                          'ROS_LOCALHOST_ONLY', 'PYTEST_ADDOPTS', 'OPENBLAS_NUM_THREADS', 'OMP_NUM_THREADS']},
          'scope': 'Fresh tests of copied handover source in an independent /tmp directory. Existing ROS installation reused; no fresh colcon build or Gazebo matrix in this packaging task.'}
(out / 'command.json').write_text(json.dumps(report, indent=2) + '\n')
PY
set +e
python3 -m pytest src scripts -q -rs --confcutdir=. -p no:cacheprovider \
  --junitxml="$out/source-tests.xml" > "$out/source-tests.log" 2>&1
test_exit=$?
set -e
printf '%s\n' "$test_exit" > "$out/exit-code.txt"
tail -35 "$out/source-tests.log"
exit "$test_exit"
