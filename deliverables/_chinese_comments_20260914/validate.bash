#!/usr/bin/env bash
set -eo pipefail
work="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
repo="$(cd -- "$work/../.." && pwd)"
output="${1:-$work/validation}"
if [[ -e "$output" ]]; then
    printf 'Refusing existing validation output: %s\n' "$output" >&2
    exit 2
fi
mkdir -p "$output"
test_root="$(mktemp -d /tmp/usv-comments-20260914-XXXXXX)"
cp -a "$repo/src" "$repo/scripts" "$repo/conftest.py" "$repo/pytest.ini" "$repo/README.md" "$repo/CONTEXT.md" "$test_root/"
source /opt/ros/humble/setup.bash
source /home/usv/module-layout-20260914/install/setup.bash
export USV_DISTRIBUTED_INSTALL=/home/usv/module-layout-20260914/install
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1
export ROS_DOMAIN_ID=232 ROS_LOCALHOST_ONLY=1 PYTHONDONTWRITEBYTECODE=1
export PYTHONPATH="$test_root/src/core/mission_module/usv_mission:${PYTHONPATH:-}"
export PYTEST_ADDOPTS='-o testpaths=src/common/test'
cd "$test_root"
python3 - "$output" <<'PY'
from pathlib import Path
import hashlib,json,os,platform,sys
import numpy,pytest,yaml
output=Path(sys.argv[1])
files={p.as_posix():hashlib.sha256(p.read_bytes()).hexdigest()
       for top in ['src','scripts'] for p in Path(top).rglob('*')
       if p.is_file() and '__pycache__' not in p.parts and p.suffix not in ['.pyc','.pyo']}
(output/'tested-source-hashes.json').write_text(json.dumps(files,indent=2)+'\n')
report={'source':str(Path.cwd()),'python':sys.version,'platform':platform.platform(),
        'numpy':numpy.__version__,'pytest':pytest.__version__,'pyyaml':yaml.__version__,
        'ROS_DOMAIN_ID':os.environ['ROS_DOMAIN_ID'],
        'reused_install':os.environ['USV_DISTRIBUTED_INSTALL'],
        'command':['python3','-m','pytest','src','scripts','-q','-rs','--confcutdir=.',
                   '-p','no:cacheprovider','--junitxml='+str(output/'tests.xml')],
        'scope':'Tests use commented source and existing ROS installation. No new C++ build or Gazebo scenario matrix.'}
(output/'command.json').write_text(json.dumps(report,indent=2)+'\n')
PY
while IFS= read -r -d '' script; do
    bash -n "$script"
done < <(find src scripts -type f \( -name '*.sh' -o -name '*.bash' \) -print0)
printf 'All source Bash scripts passed bash -n.\n' > "$output/bash-syntax.txt"
set +e
python3 -m pytest src scripts -q -rs --confcutdir=. -p no:cacheprovider \
    --junitxml="$output/tests.xml" > "$output/tests.log" 2>&1
test_exit=$?
set -e
printf '%s\n' "$test_exit" > "$output/exit-code.txt"
tail -25 "$output/tests.log"
exit "$test_exit"
