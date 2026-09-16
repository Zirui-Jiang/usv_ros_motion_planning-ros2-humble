#!/usr/bin/env bash
# 在独立输出目录构建源码并记录过程，避免把旧安装产物误当作新构建结果。
# Build a pinned local revision in a new workspace; never remove existing paths.
# This script runs a real build when given all three arguments.
# 让失败命令或管道失败传播为非零退出，避免后续步骤掩盖前一步错误。
set -eo pipefail

# 该工具构建指定的已提交版本；不会自动带入当前工作树的未提交代码。
usage() {
    printf 'Usage: bash %s SOURCE_REPO REV NEW_WORKSPACE\n' "$0"
    printf 'SOURCE_REPO must be local; NEW_WORKSPACE and its build/install must be new.\n'
    printf 'The parent directory must already exist outside any Git worktree.\n'
}

if [[ $# -eq 1 && ( "$1" == --help || "$1" == -h ) ]]; then
    usage
    exit 0
fi
if [[ $# -ne 3 ]]; then
    usage >&2
    exit 2
fi

fail() { printf 'ERROR: %s\n' "$*" >&2; exit 2; }

# 先检查依赖、固定提交和新目录边界，避免在预检失败时创建半成品工作区。
for executable in git realpath env bash python3 tee sha256sum; do
    command -v "$executable" >/dev/null || fail "missing executable: $executable"
done
[[ -r /opt/ros/humble/setup.bash ]] || fail 'ROS Humble setup is unavailable'
[[ -d "$1" ]] || fail 'SOURCE_REPO must be an existing local directory'
[[ -n "$2" && -n "$3" ]] || fail 'REV and NEW_WORKSPACE must not be empty'
source_repo="$(realpath -e -- "$1")"
revision="$2"
commit="$(git -C "$source_repo" rev-parse --verify --end-of-options "$revision^{commit}")" \
    || fail 'REV does not resolve to a commit in SOURCE_REPO'
[[ ! -e "$3" && ! -L "$3" ]] || fail 'NEW_WORKSPACE already exists (including symlinks)'
new_parent="$(cd -- "$(dirname -- "$3")" && pwd -P)" \
    || fail 'NEW_WORKSPACE parent directory must already exist'
new_workspace="$(realpath -m -- "$new_parent/$(basename -- "$3")")"
[[ ! -e "$new_workspace" && ! -L "$new_workspace" ]] || fail 'resolved NEW_WORKSPACE already exists'
if [[ "$(git -C "$new_parent" rev-parse --is-inside-work-tree 2>/dev/null || true)" == true ]]; then
    fail 'NEW_WORKSPACE must be outside existing Git worktrees, including source and baseline'
fi
case "$new_workspace/" in
    "$source_repo/"*) fail 'NEW_WORKSPACE must be separate from SOURCE_REPO' ;;
esac

for relative in scripts/validation/verify_install.py \
                scripts/validation/run_source_tests.bash \
                scripts/validation/capture_provenance.py; do
    git -C "$source_repo" cat-file -e "$commit:$relative" \
        || fail "fixed revision is missing $relative"
done
for generated in build install log validation; do
    if git -C "$source_repo" cat-file -e "$commit:$generated" 2>/dev/null; then
        fail "fixed revision already tracks reserved output path: $generated"
    fi
done

script_path="$(realpath -e -- "${BASH_SOURCE[0]}")"
printf -v invocation '%q ' bash "$script_path" "$source_repo" "$commit" "$new_workspace"
printf 'Fixed commit: %s\nNew workspace: %s\n' "$commit" "$new_workspace"
# 克隆固定提交到全新目录并避免硬链接对象，原仓库和既有构建目录保持独立。
git clone --no-hardlinks --no-checkout -- "$source_repo" "$new_workspace"
git -C "$new_workspace" checkout --detach "$commit"
[[ "$(git -C "$new_workspace" rev-parse HEAD)" == "$commit" ]] \
    || fail 'cloned HEAD differs from the fixed commit; workspace preserved'

validation="$new_workspace/validation"
mkdir -- "$validation"
cp -- "$script_path" "$validation/build_clean.invoked.bash"
printf '{}\n' > "$validation/colcon-defaults.yaml"
{
    printf 'source_repo=%s\nrequested_revision=%s\ncommit=%s\nworkspace=%s\n' \
        "$source_repo" "$revision" "$commit" "$new_workspace"
    printf 'invocation=%s\nstarted_utc=%s\n' "$invocation" "$(date -u +%FT%TZ)"
    printf 'invoked_script_sha256='
    sha256sum -- "$validation/build_clean.invoked.bash"
    printf 'build_policy=isolated install; sequential packages; at most 2 compile jobs; Release; BUILD_TESTING=OFF\n'
    printf 'test_policy=source pytest with JUnit, after installed-package audit in a separate shell\n'
} > "$validation/build-request.txt"

# Do not inherit source PYTHONPATH, ROS overlays, CMake prefixes, or user colcon
# defaults. Each stage below gets a new shell with only the declared environment.
# 清空继承的环境，再显式设置必要变量，防止旧 ROS overlay 或用户 colcon 默认值污染构建。
clean_shell() {
    env -i HOME="$HOME" USER="${USER:-usv}" LOGNAME="${LOGNAME:-${USER:-usv}}" \
        PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin \
        LANG=C.UTF-8 LC_ALL=C.UTF-8 PYTHONNOUSERSITE=1 \
        ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-174}" \
        OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 \
        MAKEFLAGS=-j2 CMAKE_BUILD_PARALLEL_LEVEL=2 \
        COLCON_HOME="$validation/colcon-home" \
        COLCON_DEFAULTS_FILE="$validation/colcon-defaults.yaml" \
        bash --noprofile --norc "$@"
}

# 分别在构建前后记录文件身份；事后快照只能证明采集时刻的内容。
capture() {
    clean_shell -c 'exec python3 "$1/scripts/validation/capture_provenance.py" \
        --workspace "clean=$1" --phase "$2" --run-directory "$1/validation" \
        --output "$1/validation/provenance-$2.json" --invocation "$3"' \
        build-clean-provenance "$new_workspace" "$1" "$invocation"
}

stage=provenance-before
# 退出陷阱保留原失败码并尝试补齐最终来源记录，不用日志输出成功掩盖构建失败。
finish() {
    exit_code=$?
    trap - EXIT
    set +e
    capture after 2>&1 | tee "$validation/provenance-after.log"
    provenance_exit_code=$?
    if [[ "$exit_code" -eq 0 && "$provenance_exit_code" -ne 0 ]]; then
        exit_code="$provenance_exit_code"
        stage=provenance-after
    fi
    printf 'last_stage=%s\nexit_code=%s\nprovenance_exit_code=%s\nfinished_utc=%s\n' \
        "$stage" "$exit_code" "$provenance_exit_code" "$(date -u +%FT%TZ)" \
        > "$validation/build-result.txt"
    printf 'Build validation finished: stage=%s exit=%s; evidence: %s\n' \
        "$stage" "$exit_code" "$validation"
    exit "$exit_code"
}
trap finish EXIT
capture before 2>&1 | tee "$validation/provenance-before.log"

stage=colcon-build
# 构建阶段仅加载系统 ROS，不继承旧 overlay；后续独立审计再加载新生成的 install。
clean_shell -c '
    set -eo pipefail
    source /opt/ros/humble/setup.bash
    set -u
    cd -- "$1"
    exec colcon --log-base "$1/log" build --base-paths "$1/src" \
        --build-base "$1/build" --install-base "$1/install" \
        --executor sequential --event-handlers console_cohesion+ \
        --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
' build-clean-colcon "$new_workspace" 2>&1 | tee "$validation/build.log"

# Installed imports must resolve without the source-test runner or conftest.
# 在不加入源码导入路径的新 shell 中检查实际安装产物。
stage=installed-package-audit
clean_shell -c '
    set -eo pipefail
    source /opt/ros/humble/setup.bash
    source "$1/install/setup.bash"
    set -u
    cd -- "$1/validation"
    exec python3 "$1/scripts/validation/verify_install.py" \
        --source "$1" --install "$1/install" \
        --output "$1/validation/install-audit.json"
' build-clean-install-audit "$new_workspace" 2>&1 | tee "$validation/install-audit.log"

# 安装审计后再执行源码测试，两类结果各自保存，不互相代替。
stage=source-pytest
clean_shell -c '
    set -eo pipefail
    # The runner sources ROS before enabling nounset and adds the source roots.
    exec bash "$1/scripts/validation/run_source_tests.bash" \
        --junitxml="$1/validation/source-tests.xml"
' build-clean-source-tests "$new_workspace" 2>&1 | tee "$validation/source-tests.log"
stage=complete
