#!/usr/bin/env python3
"""Archive a stopped migration workspace and its evidence without changing it.

Standard-library only. The output directory must be new. The complete selected
source tree is a build snapshot, not a list of modifications made by this task.
Failed, blocked, interrupted and not-run records are preserved and never scored
again. Archive success is distinct from acceptance success.
"""
import argparse
from collections import Counter
from datetime import datetime, timezone
import gzip
import hashlib
import json
import os
from pathlib import Path, PurePosixPath
import re
import shutil
import stat
import sys
import tarfile


SOURCE_ROOTS = ("src", "scripts", "conftest.py", "pytest.ini")
BUILD_EVIDENCE = ("build.log", "source-tests.log", "install-audit.json")
REQUIRED_RUNS = ("mission-matrix-001", "navigation-mission-001", "navigation-simulation-001")
RECEIPTS = ("boundary-check.json", "moved-files.json")
EXCLUDED_DIRECTORIES = {
    ".git", "build", "install", "log", "__pycache__", ".pytest_cache",
    ".mypy_cache", ".ruff_cache", ".tox", ".venv", "venv", "dist",
}
EXCLUDED_SUFFIXES = {".pyc", ".pyo"}


def digest(path):
    result = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            result.update(block)
    return result.hexdigest()


def write_json(path, value):
    Path(path).write_text(json.dumps(value, indent=2, ensure_ascii=False, allow_nan=False) + "\n",
                          encoding="utf-8")


def read_json(path):
    with Path(path).open(encoding="utf-8") as stream:
        value = json.load(stream)
    if not isinstance(value, dict):
        raise ValueError(f"Expected a JSON object: {path}")
    return value


def inside(path, root):
    try:
        path.relative_to(root)
        return True
    except ValueError:
        return False


def safe_name(value):
    """Permit only unambiguous relative POSIX archive/checksum paths."""
    path = PurePosixPath(value)
    if (not value or path.is_absolute() or ".." in path.parts or ":" in path.parts[0]
            or any(character in value for character in ("\\", "\n", "\r", "\0"))
            or path.as_posix() != value):
        raise ValueError(f"Unsafe archive path: {value!r}")
    return value


def regular_files(root, source=False):
    """Do not follow or silently drop symlinks from a purported full snapshot."""
    root = Path(root)
    if root.is_symlink():
        raise ValueError(f"Symbolic links are not supported in snapshot inputs: {root}")
    if root.is_file():
        yield root
        return
    if not root.is_dir():
        raise ValueError(f"Required input does not exist: {root}")
    for current, directories, files in os.walk(root, followlinks=False):
        directory = Path(current)
        kept = []
        for name in sorted(directories):
            if source and (name in EXCLUDED_DIRECTORIES or name.endswith(".egg-info")):
                continue
            child = directory / name
            if child.is_symlink():
                raise ValueError(f"Symbolic-link directory requires explicit handling: {child}")
            kept.append(name)
        directories[:] = kept
        for name in sorted(files):
            child = directory / name
            if source and (name in EXCLUDED_DIRECTORIES or child.suffix in EXCLUDED_SUFFIXES):
                continue
            if child.is_symlink() or not child.is_file():
                raise ValueError(f"Non-regular snapshot input: {child}")
            yield child


def inventory(paths, root):
    result = {}
    for path in paths:
        name = safe_name(path.relative_to(root).as_posix())
        result[name] = {"sha256": digest(path), "bytes": path.stat().st_size,
                        "mode": stat.S_IMODE(path.stat().st_mode)}
    return dict(sorted(result.items()))


def source_inventory(workspace):
    files = []
    for name in SOURCE_ROOTS:
        files.extend(regular_files(workspace / name, source=True))
    return inventory(files, workspace)


def tree_digest(files):
    encoded = json.dumps(files, sort_keys=True, separators=(",", ":"), ensure_ascii=False).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()


def copy_verified(source, destination, expected):
    destination.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(source, destination)
    if digest(source) != expected["sha256"] or digest(destination) != expected["sha256"]:
        raise ValueError(f"Input changed or copy differs: {source}")


def create_tar(path, root, files, directories=()):
    """Write regular files with normalized owner/time and safe relative names."""
    with path.open("wb") as raw:
        with gzip.GzipFile(filename="", fileobj=raw, mode="wb", mtime=0) as compressed:
            with tarfile.open(fileobj=compressed, mode="w", format=tarfile.PAX_FORMAT) as archive:
                for name in directories:
                    safe_name(name)
                    info = tarfile.TarInfo(name)
                    info.type = tarfile.DIRTYPE
                    info.mode = 0o755
                    info.uid = info.gid = info.mtime = 0
                    info.uname = info.gname = ""
                    archive.addfile(info)
                for name, entry in files.items():
                    safe_name(name)
                    info = tarfile.TarInfo(name)
                    info.size = entry["bytes"]
                    info.mode = entry["mode"]
                    info.uid = info.gid = info.mtime = 0
                    info.uname = info.gname = ""
                    with (root / name).open("rb") as stream:
                        archive.addfile(info, stream)
    verify_tar(path, files, directories)
    return digest(path)


def verify_tar(path, expected, directories=()):
    """Re-read every tar member and check its bytes; never extract a tar."""
    seen = set()
    seen_directories = set()
    expected_directories = set(directories)
    with tarfile.open(path, mode="r:gz") as archive:
        for member in archive:
            name = safe_name(member.name)
            if member.isdir():
                if name not in expected_directories or name in seen_directories:
                    raise ValueError(f"Unexpected or duplicate tar directory: {name}")
                seen_directories.add(name)
                continue
            if not member.isfile() or name not in expected or name in seen:
                raise ValueError(f"Unexpected or duplicate tar member: {name}")
            seen.add(name)
            actual = hashlib.sha256()
            stream = archive.extractfile(member)
            if stream is None:
                raise ValueError(f"Tar member unreadable: {name}")
            with stream:
                for block in iter(lambda: stream.read(1024 * 1024), b""):
                    actual.update(block)
            entry = expected[name]
            if member.size != entry["bytes"] or actual.hexdigest() != entry["sha256"]:
                raise ValueError(f"Tar member verification failed: {name}")
    if seen != set(expected) or seen_directories != expected_directories:
        raise ValueError("Tar inventory differs from the original files")


def no_active_run(path):
    """A stopped interrupted matrix may retain not_run rows, but no running row."""
    summary = read_json(path)
    counts = summary.get("counts", {})
    rows = summary.get("results", [])
    if ((isinstance(counts, dict) and counts.get("running", 0))
            or summary.get("running_trials") or summary.get("result") == "running"
            or (isinstance(rows, list) and any(isinstance(row, dict) and row.get("result") == "running"
                                              for row in rows))):
        raise ValueError(f"Run still reports active trials; archive after it stops: {path}")


def summarize_run(summary_path, results_root):
    summary = read_json(summary_path)
    rows = summary.get("results")
    valid_rows = isinstance(rows, list) and all(isinstance(row, dict) for row in rows)
    rows = rows if valid_rows else []
    result_counts = Counter(str(row.get("result", "unknown")) for row in rows)
    passed = sum(row.get("passed") is True or (row.get("result") == "passed"
                                              and row.get("passed") is not False) for row in rows)
    failed = sum(row.get("result") == "failed" or
                 (row.get("passed") is False and row.get("result") not in ("blocked", "not_run", "running"))
                 for row in rows)
    reported_counts = summary.get("counts")
    planned = summary.get("planned_trials", summary.get("planned_navigation_trials"))
    consistent = not any((row.get("passed") is False and row.get("result") == "passed")
                         or (row.get("passed") is True and row.get("result") in
                             ("failed", "blocked", "not_run", "running")) for row in rows)
    if isinstance(reported_counts, dict):
        consistent &= all(type(count) is int and count == result_counts[state]
                          for state, count in reported_counts.items())
    if type(summary.get("passed")) is int:
        consistent &= summary["passed"] == passed
    row_complete = (valid_rows and bool(rows) and passed == len(rows)
                    and (planned is None or planned == len(rows)))
    all_passed = (summary.get("all_passed") is True and row_complete and consistent
                  and summary.get("interrupted") is not True
                  and summary.get("run_finished") is not False
                  and summary.get("source_unchanged") is not False
                  and summary.get("installation_unchanged") is not False)
    return {
        "summary": summary_path.relative_to(results_root).as_posix(),
        "sha256": digest(summary_path), "reported_all_passed": summary.get("all_passed"),
        "all_passed": all_passed, "reported_result": summary.get("result"),
        "planned_trials": planned, "recorded_trials": len(rows),
        "observed_passed": passed, "observed_failed": failed,
        "observed_result_counts": dict(sorted(result_counts.items())),
        "reported_counts": reported_counts, "reported_passed": summary.get("passed"),
        "reported_completed": summary.get("completed_trials", summary.get("completed")),
        "counts_consistent_with_rows": consistent,
        "interrupted": summary.get("interrupted"), "run_finished": summary.get("run_finished"),
        "source_unchanged": summary.get("source_unchanged"),
        "installation_unchanged": summary.get("installation_unchanged"),
    }


def pytest_log_summary(path):
    lines = Path(path).read_text(encoding="utf-8", errors="replace").splitlines()
    for original in reversed(lines):
        line = re.sub(r"\x1b\[[0-9;]*m", "", original).strip()
        if re.search(r"\b\d+ (?:passed|failed|error|errors|skipped)\b", line):
            counts = {name: int(number) for number, name in re.findall(
                r"\b(\d+) (passed|failed|errors?|skipped|deselected|xfailed|xpassed)\b", line)}
            return {"reported_terminal_line": line, "reported_counts": counts,
                    "all_reported_tests_passed": bool(counts.get("passed"))
                    and not any(counts.get(name, 0) for name in ("failed", "error", "errors"))}
    return {"reported_terminal_line": None, "reported_counts": {}, "all_reported_tests_passed": False}


def write_checksums(output):
    excluded = {"SHA256SUMS", "sha256-index.json"}
    files = inventory((p for p in regular_files(output)
                       if p.relative_to(output).as_posix() not in excluded), output)
    write_json(output / "sha256-index.json", {"algorithm": "sha256", "files": files})
    checksums = {name: entry["sha256"] for name, entry in files.items()}
    checksums["sha256-index.json"] = digest(output / "sha256-index.json")
    (output / "SHA256SUMS").write_text("".join(
        checksum + "  " + name + "\n" for name, checksum in sorted(checksums.items())), encoding="utf-8")
    for line in (output / "SHA256SUMS").read_text(encoding="utf-8").splitlines():
        checksum, name = line.split("  ", 1)
        safe_name(name)
        if digest(output / name) != checksum:
            raise ValueError(f"Final checksum verification failed: {name}")
    return len(checksums)


def archive(workspace, output, receipt):
    workspace, output, receipt = (Path(value).resolve() for value in (workspace, output, receipt))
    if output.exists():
        raise ValueError(f"Refusing existing output directory: {output}")
    inputs = [workspace / name for name in (*SOURCE_ROOTS, "validation", "results")] + [receipt]
    if any(inside(output, path) or inside(path, output) for path in inputs):
        raise ValueError("Output must not overlap source, evidence or receipt input directories")
    for name in BUILD_EVIDENCE:
        if not (workspace / "validation" / name).is_file():
            raise ValueError(f"Missing completed build evidence: validation/{name}")
    for name in RECEIPTS:
        if not (receipt / name).is_file():
            raise ValueError(f"Missing migration receipt: {receipt / name}")
    result_files = list(regular_files(workspace / "results"))
    for path in result_files:
        if path.name in ("summary.json", "progress.json"):
            no_active_run(path)
    sources = source_inventory(workspace)
    # Preserve earlier failed build/test attempts as well as the final root logs.
    evidence_paths = list(regular_files(workspace / "validation")) + result_files
    evidence = inventory(evidence_paths, workspace)
    receipts = inventory([receipt / name for name in RECEIPTS], receipt)

    output.mkdir(parents=True, exist_ok=False)
    marker = output / "ARCHIVE_INCOMPLETE"
    marker.write_text("Archive creation is incomplete. Do not treat this directory as verified.\n", encoding="utf-8")
    try:
        for name, entry in evidence.items():
            copy_verified(workspace / name, output / "raw" / name, entry)
        # Preserve even empty diagnostic directories, so discovery is auditable.
        for root_name in ("validation", "results"):
            for current, _, _ in os.walk(workspace / root_name):
                relative = Path(current).relative_to(workspace)
                safe_name(relative.as_posix())
                (output / "raw" / relative).mkdir(parents=True, exist_ok=True)
        for name, entry in receipts.items():
            copy_verified(receipt / name, output / "receipts" / name, entry)
        shutil.copy2(Path(__file__).resolve(), output / "archive_migration.py")

        source_tar = output / "build-source-snapshot.tar.gz"
        source_sha = create_tar(source_tar, workspace, sources)
        if source_inventory(workspace) != sources:
            raise ValueError("Source changed during archive creation")
        write_json(output / "source-snapshot.json", {
            "workspace": str(workspace), "roots": list(SOURCE_ROOTS), "files": sources,
            "file_count": len(sources), "tree_sha256": tree_digest(sources),
            "archive": source_tar.name, "archive_sha256": source_sha,
            "archive_members_verified": True,
            "scope": "Full selected build source, including existing dependencies and parallel radar source; not this task's change list.",
            "excluded_directories": sorted(EXCLUDED_DIRECTORIES),
            "additional_exclusions": ["*.egg-info directories", *sorted(EXCLUDED_SUFFIXES)],
        })
        raw_files = inventory(regular_files(output / "raw"), output)
        raw_directories = ["raw"] + sorted(path.relative_to(output).as_posix()
                                          for path in (output / "raw").rglob("*") if path.is_dir())
        raw_sha = create_tar(output / "raw-evidence.tar.gz", output, raw_files, raw_directories)
        if inventory(evidence_paths, workspace) != evidence:
            raise ValueError("Build/result evidence changed during archive creation")
        if inventory([receipt / name for name in RECEIPTS], receipt) != receipts:
            raise ValueError("Boundary receipts changed during archive creation")

        results_root = output / "raw/results"
        summaries = [summarize_run(path, results_root) for path in sorted(results_root.rglob("summary.json"))]
        run_names = sorted(path.name for path in (workspace / "results").iterdir() if path.is_dir())
        by_name = {row["summary"].split("/")[0]: row for row in summaries
                   if len(PurePosixPath(row["summary"]).parts) == 2}
        top_level = [{"run": name, "summary_present": name in by_name,
                      **by_name.get(name, {"all_passed": False})} for name in run_names]
        required_passed = all(name in by_name and by_name[name]["all_passed"] for name in REQUIRED_RUNS)
        audit = read_json(output / "raw/validation/install-audit.json")
        tests = pytest_log_summary(output / "raw/validation/source-tests.log")
        summary = {
            "created_utc": datetime.now(timezone.utc).isoformat(), "workspace": str(workspace),
            "archive_complete": True, "required_runs": list(REQUIRED_RUNS),
            "missing_required_run_summaries": [name for name in REQUIRED_RUNS if name not in by_name],
            "required_runs_all_passed": required_passed,
            "all_discovered_top_level_runs_passed": bool(top_level) and all(row["all_passed"] for row in top_level),
            "top_level_runs": top_level,
            "all_run_summaries": summaries,
            "counting_note": "Top-level run counts only; nested harness summaries are listed separately, never added again.",
            "source_tests": tests, "install_audit_reported_all_passed": audit.get("all_passed"),
            "source_archive": source_tar.name, "source_archive_sha256": source_sha,
            "source_file_count": len(sources), "source_tree_sha256": tree_digest(sources),
            "raw_archive": "raw-evidence.tar.gz", "raw_archive_sha256": raw_sha,
            "raw_file_count": len(raw_files), "receipt_files": receipts,
            "raw_directories": raw_directories,
            "integrity": {"source_tar_members_verified": True, "raw_tar_members_verified": True,
                          "source_unchanged_during_archive": True, "evidence_unchanged_during_archive": True},
            "scope_note": "The build snapshot contains current dependencies and parallel radar source; boundary receipts identify migration scope. Historical station-guard-001 is not modified or merged into these results.",
        }
        write_json(output / "archive-summary.json", summary)
        rows = []
        for run in top_level:
            counts = run.get("observed_result_counts", {})
            rows.append(f"| `{run['run']}` | {run.get('observed_passed', 'unknown')} / "
                        f"{run.get('planned_trials') or run.get('recorded_trials', 'unknown')} | "
                        f"`{json.dumps(counts, ensure_ascii=False)}` | {run['all_passed']} |")
        report = "\n".join(rows) or "| No run directories | unknown | unknown | False |"
        (output / "README.md").write_text(
            "# core 任务迁移验证证据\n\n"
            f"来源工作区：`{workspace}`。归档时间：`{summary['created_utc']}`。\n\n"
            f"所要求三组运行是否全部通过：**{required_passed}**。"
            f"发现的全部顶层运行是否都通过：**{summary['all_discovered_top_level_runs_passed']}**。\n"
            "归档成功只说明证据已复制并核对哈希；失败、中断、阻塞、未运行或缺少摘要不能解释为验收通过。\n\n"
            "| 原运行目录 | 记录通过数 / 计划数（无计划数时用记录数） | 原 result 分布 | 核对后全通过 |\n"
            "|---|---:|---|---|\n" + report + "\n\n"
            "- [archive-summary.json](archive-summary.json)：逐运行真实摘要、嵌套摘要清单及记录数核对，不重复累计子运行。\n"
            "- [raw](raw)：完整保留 validation 全目录（包括较早失败的构建或测试尝试）及 results 全目录；已有图件原样复制。\n"
            "- [raw-evidence.tar.gz](raw-evidence.tar.gz)：上述 raw 内容的压缩副本。\n"
            "- [build-source-snapshot.tar.gz](build-source-snapshot.tar.gz)：src、scripts、conftest.py、pytest.ini 全量构建源码。\n"
            "- [source-snapshot.json](source-snapshot.json)：每个源码文件的 SHA256、大小、权限与总归档 SHA256。\n"
            "- [receipts/boundary-check.json](receipts/boundary-check.json) 与 [receipts/moved-files.json](receipts/moved-files.json)：独立修改边界与迁移清单证据。\n"
            "- [SHA256SUMS](SHA256SUMS)、[sha256-index.json](sha256-index.json)：本目录归档时全部交付文件的哈希。\n\n"
            "源码快照包含现有依赖和并行雷达源码，以便从源码构建；快照内容不等于本次修改清单，也不表示所有并行功能都经过本轮验收。"
            "不包含编译安装产物和缓存；外部 ROS/Gazebo 系统依赖仍需按项目环境安装。\n\n"
            "旧 `docs/validation/station-guard-001` 是独立迁移前历史证据，本工具没有读取并合并它的闭环结果，也没有修改该目录。"
            "新的指标表和迁移报告由独立统计流程生成。若随后增加图表或报告，应重新调用本工具的 `write_checksums(output)` 更新清单。\n\n"
            "在当前目录执行 `sha256sum --check SHA256SUMS` 可复核文件完整性；两个 tar 已在制作后逐成员复读校验，没有执行解包。\n",
            encoding="utf-8")
        marker.unlink()
        checksum_count = write_checksums(output)
        return {"output": str(output), "archive_complete": True,
                "required_runs_all_passed": required_passed,
                "source_files": len(sources), "raw_files": len(raw_files),
                "verified_checksums": checksum_count}
    except BaseException as error:
        marker.write_text(f"Archive incomplete: {type(error).__name__}: {error}\n", encoding="utf-8")
        raise


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--workspace", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--receipt", type=Path, default=Path(__file__).resolve().parent,
                        help="Directory containing boundary-check.json and moved-files.json")
    args = parser.parse_args(argv)
    try:
        result = archive(args.workspace, args.output, args.receipt)
    except (OSError, ValueError, tarfile.TarError) as error:
        print(f"Archive failed: {error}", file=sys.stderr)
        return 2
    print(json.dumps(result, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
