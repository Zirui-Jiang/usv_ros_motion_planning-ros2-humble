#!/usr/bin/env bash
# 从当前源码建立全新外部验证工作区，保留基线来源后构建、审计安装并运行源码测试。
set -euo pipefail
# Usage: build_missions.bash [SOURCE_CHECKOUT] NEW_WORKSPACE
source "$(dirname -- "${BASH_SOURCE[0]}")/_environment.bash"
if [[ $# -eq 1 ]]; then
  source_repo="$task_source_root"
  mission_ws="$(realpath -m -- "$1")"
elif [[ $# -eq 2 ]]; then
  source_repo="$(realpath -- "$1")"
  mission_ws="$(realpath -m -- "$2")"
else
  echo 'Usage: build_missions.bash [SOURCE_CHECKOUT] NEW_WORKSPACE' >&2
  exit 2
fi
# 拒绝已有文件、目录或符号链接，确保构建不会覆盖已验证的源码和证据。
if [[ -e "$mission_ws" || -L "$mission_ws" ]]; then
  echo "Refusing existing output: $mission_ws" >&2
  exit 2
fi
if [[ ! -d "$source_repo/src" || ! -f "$source_repo/$task_validation_relative/verify_install.py" ]]; then
  echo "Task source checkout is incomplete: $source_repo" >&2
  exit 2
fi
case "$mission_ws/" in
  "$source_repo/"*) echo 'Output must be outside the source checkout.' >&2; exit 2 ;;
esac
# Preserve Git baseline provenance without checking out obsolete source files.
git clone --no-checkout --no-hardlinks "$source_repo" "$mission_ws"
mkdir -p -- "$mission_ws/validation"
rsync -a --exclude=__pycache__ --exclude=.pytest_cache "$source_repo/src" "$mission_ws/"
rsync -a --exclude=__pycache__ --exclude=.pytest_cache "$source_repo/scripts" "$mission_ws/"
for file in conftest.py pytest.ini; do
  if [[ -f "$source_repo/$file" ]]; then cp -- "$source_repo/$file" "$mission_ws/$file"; fi
done
git -C "$source_repo" rev-parse HEAD > "$mission_ws/validation/base-commit.txt"
git -C "$source_repo" diff --binary > "$mission_ws/validation/source.patch"
git -C "$source_repo" status --porcelain=v1 > "$mission_ws/validation/source-status.txt"
printf '%s\n' "$source_repo" > "$mission_ws/validation/source-checkout.txt"
cd -- "$mission_ws"
set +u
source /opt/ros/humble/setup.bash
set -u
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1
colcon build --parallel-workers 3 --cmake-args -DCMAKE_BUILD_TYPE=Release > validation/build.log 2>&1
task_source_environment "$mission_ws"
python3 "$task_validation_relative/verify_install.py" --source "$mission_ws" --install "$mission_ws/install" \
  --output "$mission_ws/validation/install-audit.json"
bash "$task_validation_relative/run_source_tests.bash" > validation/source-tests.log 2>&1
cat validation/source-tests.log
