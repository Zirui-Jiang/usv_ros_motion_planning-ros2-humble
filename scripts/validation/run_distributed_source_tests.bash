#!/usr/bin/env bash
# 加载选定安装区并运行完整源码测试，保存命令、日志及 JUnit 结果。
# Run all source tests with the same exporter-fixture environment as the baseline.
# 让失败命令或管道失败传播为非零退出，避免后续步骤掩盖前一步错误。
set -eo pipefail
repo="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
install="${1:?usage: run_distributed_source_tests.bash /absolute/install /absolute/new/output}"
output="${2:?an output directory is required}"
case "$install:$output" in /*:/*) ;; *) echo 'Paths must be absolute' >&2; exit 2 ;; esac
if [[ -e "$output" ]]; then
  echo "Refusing existing output directory: $output" >&2
  exit 2
fi
# 先加载系统 ROS，再加载工作区 overlay，使包与消息类型按选定环境解析。
source /opt/ros/humble/setup.bash
source "$install/setup.bash"
set -u
mkdir -p "$output"
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-192}" ROS_LOCALHOST_ONLY=1
export PYTHONPATH="$repo/src/core/mission_module/usv_mission:${PYTHONPATH:-}"
# The outer invocation explicitly collects src and scripts. This setting only
# changes default discovery in the legacy exported-workspace test's child
# pytest, whose fixture deliberately contains common/test only. It does not
# exclude any outer tests and is identical to the pre-split baseline setting.
# 该默认发现路径用于导出工作区的子测试；外层显式收集 src/scripts，不缩减全量范围。
export PYTEST_ADDOPTS='-o testpaths=src/common/test'
cd "$repo"
python3 -c 'import json,os,sys; print(json.dumps({"argv":["python3","-m","pytest","src","scripts","-q","--junitxml="+sys.argv[1]+"/tests.xml"],"python":sys.executable,"ROS_DOMAIN_ID":os.environ["ROS_DOMAIN_ID"],"PYTEST_ADDOPTS":os.environ["PYTEST_ADDOPTS"],"USV_DISTRIBUTED_INSTALL":os.environ.get("USV_DISTRIBUTED_INSTALL")},indent=2))' "$output" > "$output/command.json"
set +e
python3 -m pytest src scripts -q --junitxml="$output/tests.xml" > "$output/tests.log" 2>&1
task_exit=$?
set -e
tail -15 "$output/tests.log"
exit "$task_exit"
