#!/usr/bin/env python3
"""Archive completed station-guard evidence without modifying its workspace.

Python dependencies are standard-library only; the Git executable is required.
Run in WSL after the selected matrix has written a final summary with no running
trials. Raw evidence is append-only: an existing different file is an error.
Derived source archives, patch, audit copies and summaries replace older output.
"""
import argparse
from collections import Counter
import csv
from datetime import datetime, timezone
import gzip
import hashlib
import io
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tarfile
import tempfile


BASELINE = "81ab8ed"
EXCLUDED_DIRECTORIES = {".git", "build", "install", "__pycache__", ".pytest_cache"}
SOURCE_SUFFIXES = {".py", ".yaml", ".srv", ".world", ".xml", ".cpp", ".hpp", ".h", ".bash"}


def digest(path):
    result = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            result.update(block)
    return result.hexdigest()


def inside(path, root):
    try:
        path.relative_to(root)
        return True
    except ValueError:
        return False


def workspace_path(workspace, value):
    path = (workspace / value).resolve()
    if not inside(path, workspace):
        raise ValueError(f"evidence path escapes workspace: {value}")
    return path


def json_bytes(value):
    return (json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False) + "\n").encode("utf-8")


def load_json(path):
    with path.open(encoding="utf-8") as stream:
        result = json.load(stream)
    if not isinstance(result, dict):
        raise ValueError(f"JSON object required: {path}")
    return result


def atomic_bytes(path, content):
    path.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.NamedTemporaryFile(dir=path.parent, prefix=".archive-", delete=False) as stream:
        temporary = Path(stream.name)
        try:
            stream.write(content)
            stream.flush()
            os.fsync(stream.fileno())
        except BaseException:
            temporary.unlink(missing_ok=True)
            raise
    try:
        temporary.replace(path)
    finally:
        temporary.unlink(missing_ok=True)


def require_finished_matrix(matrix, expected_trials):
    path = matrix / "summary.json"
    if not path.is_file():
        raise ValueError(f"matrix summary does not exist; do not archive a running matrix: {path}")
    summary = load_json(path)
    rows = summary.get("results")
    if not isinstance(rows, list) or not all(isinstance(row, dict) for row in rows):
        raise ValueError("matrix summary must contain a results list")
    if (summary.get("counts", {}).get("running") != 0
            or summary.get("running_trials")
            or any(row.get("result") == "running" for row in rows)):
        raise ValueError("matrix still has running trials or lacks an explicit running=0 count")
    if len(rows) != expected_trials or summary.get("planned_trials") != expected_trials:
        raise ValueError(f"expected {expected_trials} trial rows, received {len(rows)}; no rows fabricated")
    ids = [row.get("trial_id") for row in rows]
    if not all(isinstance(name, str) and name for name in ids) or len(set(ids)) != len(ids):
        raise ValueError("matrix trial identifiers are missing or duplicated")
    if (matrix / "progress.json").is_file():
        progress = load_json(matrix / "progress.json")
        if (progress.get("counts", {}).get("running") != 0 or progress.get("running_trials")
                or any(row.get("result") == "running" for row in progress.get("results", []))):
            raise ValueError("progress.json still reports running trials")
    return summary, digest(path)


def git(workspace, *arguments, index=None, data=None):
    environment = dict(os.environ, GIT_OPTIONAL_LOCKS="0")
    environment.pop("GIT_INDEX_FILE", None)
    if index is not None:
        environment["GIT_INDEX_FILE"] = str(index)
    result = subprocess.run(
        ["git", "-c", "core.filemode=false", "-c", "core.autocrlf=false", "-C", str(workspace), *arguments],
        input=data, stdout=subprocess.PIPE, stderr=subprocess.PIPE, env=environment, check=False)
    if result.returncode:
        raise RuntimeError(f"git {' '.join(arguments[:3])} failed: {result.stderr.decode('utf-8', 'replace')}")
    return result.stdout


def null_paths(raw):
    return [os.fsdecode(name) for name in raw.split(b"\0") if name]


def tree_entries(raw):
    entries = {}
    for record in raw.split(b"\0"):
        if record:
            metadata, name = record.split(b"\t", 1)
            mode, kind, object_id = metadata.decode("ascii").split()
            if kind != "blob":
                raise ValueError(f"source archive does not support {kind}: {os.fsdecode(name)}")
            entries[os.fsdecode(name)] = (mode, object_id)
    return entries


def source_fingerprint(workspace):
    """Match mission_acceptance.fingerprint's source inventory, without ROS imports."""
    files = {}
    for folder in (workspace / "src", workspace / "scripts/validation"):
        if folder.is_dir():
            for path in sorted(folder.rglob("*")):
                if path.is_file() and (path.suffix in SOURCE_SUFFIXES or path.name == "CMakeLists.txt"):
                    files[str(path.relative_to(workspace))] = digest(path)
    return {"files": files, "source_sha256": hashlib.sha256(
        json.dumps(files, sort_keys=True).encode()).hexdigest()}


def build_source_artifacts(workspace, output, temporary, baseline_ref):
    baseline = git(workspace, "rev-parse", "--verify", baseline_ref + "^{commit}").decode().strip()
    base_entries = tree_entries(git(workspace, "ls-tree", "-r", "-z", baseline, "--", "src", "scripts"))
    tracked_delta = null_paths(git(workspace, "diff", "--name-only", "-z", "--no-renames",
                                  baseline, "--", "src", "scripts"))
    untracked = null_paths(git(workspace, "ls-files", "--others", "--exclude-standard", "-z",
                              "--", "src", "scripts"))
    selected = sorted(set(tracked_delta + untracked))
    index = temporary / "source.index"
    git(workspace, "read-tree", baseline, index=index)
    if selected:
        pathspecs = b"".join(os.fsencode(":(literal)" + name) + b"\0" for name in selected)
        git(workspace, "add", "-A", "--pathspec-from-file=-", "--pathspec-file-nul",
            index=index, data=pathspecs)
    staged_tree = git(workspace, "write-tree", index=index).decode().strip()
    entries = tree_entries(git(workspace, "ls-tree", "-r", "-z", staged_tree, "--", "src", "scripts"))
    changes, normalizations = [], []
    for name in selected:
        if name not in entries:
            changes.append({"path": name, "status": "deleted", "baseline_mode": base_entries[name][0]})
            continue
        mode, object_id = entries[name]
        if mode not in ("100644", "100755", "120000"):
            raise ValueError(f"unsupported source mode {mode}: {name}")
        # Preserve deliberate baseline executable modes; Windows/rsync chmod is
        # not a content change. New regular files use deterministic 100644.
        if mode != "120000":
            previous_mode = base_entries.get(name, ("100644", None))[0]
            mode = previous_mode if previous_mode in ("100644", "100755") else "100644"
        entries[name] = (mode, object_id)
        normalizations.append(f"{mode} {object_id}\t".encode() + os.fsencode(name) + b"\0")
        changes.append({"path": name, "status": "modified" if name in base_entries else "added",
                        "mode": mode, "git_blob": object_id})
    if normalizations:
        git(workspace, "update-index", "-z", "--index-info", index=index, data=b"".join(normalizations))
    patch = git(workspace, "diff", "--cached", "--binary", "--full-index", "--no-renames",
                "--no-ext-diff", "--no-textconv", baseline, "--", "src", "scripts", index=index)
    patch_file = temporary / "feature.patch"
    patch_file.write_bytes(patch)
    checker = temporary / "verify.index"
    git(workspace, "read-tree", baseline, index=checker)
    if patch:
        git(workspace, "apply", "--cached", "--check", "--binary", str(patch_file), index=checker)
        git(workspace, "apply", "--cached", "--binary", str(patch_file), index=checker)
    if git(workspace, "write-tree", index=checker) != git(workspace, "write-tree", index=index):
        raise ValueError("patch applied tree differs from the staged source snapshot")
    archive_path = temporary / "validated-feature-source.tar.gz"
    with archive_path.open("wb") as raw, gzip.GzipFile(fileobj=raw, mode="wb", filename="", mtime=0) as compressed:
        with tarfile.open(fileobj=compressed, mode="w") as archive:
            for change in changes:
                if change["status"] == "deleted":
                    continue
                name = change["path"]
                mode, object_id = entries[name]
                content = git(workspace, "cat-file", "blob", object_id)
                change.update(sha256=hashlib.sha256(content).hexdigest(), bytes=len(content))
                info = tarfile.TarInfo(name)
                info.mtime, info.uid, info.gid = 0, 0, 0
                if mode == "120000":
                    info.type, info.linkname, info.mode = tarfile.SYMTYPE, os.fsdecode(content), 0o777
                    archive.addfile(info)
                else:
                    info.mode, info.size = int(mode[-3:], 8), len(content)
                    archive.addfile(info, io.BytesIO(content))
    atomic_bytes(output / "feature.patch", patch)
    atomic_bytes(output / archive_path.name, archive_path.read_bytes())
    return {"classification": "src/scripts content delta plus untracked files from the isolated workspace",
            "baseline_commit": baseline, "workspace_head": git(workspace, "rev-parse", "HEAD").decode().strip(),
            "mode_policy": "core.filemode=false; tracked regular modes from baseline; new regular modes 100644",
            "archive_sha256": digest(output / archive_path.name), "patch_sha256": hashlib.sha256(patch).hexdigest(),
            "patch_cached_check_passed": True, "patch_applied_tree_matches": True,
            "files": [row["path"] for row in changes if row["status"] != "deleted"],
            "changes": changes, "untracked_files": untracked}


def evidence_files(directory):
    def fail(error):
        raise error
    for root, folders, files in os.walk(directory, followlinks=False, onerror=fail):
        base = Path(root)
        folders[:] = sorted(name for name in folders if name not in EXCLUDED_DIRECTORIES)
        if any((base / name).is_symlink() for name in folders):
            raise ValueError(f"refusing a symlinked evidence directory under {base}")
        for name in sorted(files):
            path = base / name
            if path.is_symlink() or not path.is_file():
                raise ValueError(f"raw evidence must be a regular file: {path}")
            yield path


def immutable_copy(source, destination, output):
    if not inside(destination.resolve(), output) or destination.is_symlink():
        raise ValueError(f"archive destination escapes output: {destination}")
    source_hash = digest(source)
    if destination.exists():
        if not destination.is_file() or digest(destination) != source_hash:
            raise ValueError(f"existing raw evidence differs; refusing overwrite: {destination}")
        return
    destination.parent.mkdir(parents=True, exist_ok=True)
    # Exclusive creation protects raw evidence even against a concurrent writer.
    with source.open("rb") as original, destination.open("xb") as copied:
        shutil.copyfileobj(original, copied, 1024 * 1024)
    if digest(destination) != source_hash or digest(source) != source_hash:
        raise ValueError(f"evidence changed during copy: {source}; incomplete output requires inspection")


def collect_evidence(workspace, output, directories):
    records = []
    for directory in sorted(set(directories)):
        relative = directory.relative_to(workspace)
        if not directory.is_dir():
            records.append({"source": str(relative), "available": False, "reason": "directory missing"})
            continue
        progress_file = directory / "progress.json"
        if progress_file.is_file():
            progress = load_json(progress_file)
            if progress.get("counts", {}).get("running", 0) or progress.get("running_trials"):
                raise ValueError(f"auxiliary evidence is still running: {directory}")
        count = 0
        for source in evidence_files(directory):
            immutable_copy(source, output / "raw" / source.relative_to(workspace), output)
            count += 1
        records.append({"source": str(relative), "archive": str(Path("raw") / relative),
                        "available": True, "file_count": count})
    return records


def trial_csv(summary):
    fields = ["trial_id", "scenario", "matrix_repeat", "domain_id", "result", "passed", "returncode",
              "reason", "measured_result", "duration_sim_s", "sample_count",
              "minimum_controlled_obstacle_footprint_gap_m", "child_summary", "measurement_json"]
    stream = io.StringIO(newline="")
    writer = csv.DictWriter(stream, fields, lineterminator="\n")
    writer.writeheader()
    for task in summary["results"]:
        measurement = task.get("measurement") or {}
        if not isinstance(measurement, dict):
            raise ValueError(f"measurement is not an object for {task['trial_id']}")
        row = {name: task.get(name) for name in fields}
        for name in ("duration_sim_s", "sample_count", "minimum_controlled_obstacle_footprint_gap_m"):
            row[name] = measurement.get(name)
        row["measured_result"] = measurement.get("result")
        row["measurement_json"] = json.dumps(measurement, ensure_ascii=False, allow_nan=False)
        writer.writerow(row)
    return stream.getvalue().encode("utf-8-sig")


def write_inventory(output):
    omitted = {"sha256-index.json", "SHA256SUMS", ".archive.lock"}
    rows = []
    for path in evidence_files(output):
        relative = path.relative_to(output).as_posix()
        if relative not in omitted:
            rows.append({"path": relative, "bytes": path.stat().st_size, "sha256": digest(path)})
    atomic_bytes(output / "sha256-index.json", json_bytes({
        "algorithm": "sha256", "excluded_self_referential_files": sorted(omitted), "files": rows}))
    # GNU sha256sum escaping preserves spaces, backslashes and unusual filenames.
    lines = []
    for row in rows:
        name = row["path"]
        escaped = "\\" in name or "\n" in name
        name = name.replace("\\", "\\\\").replace("\n", "\\n")
        lines.append(("\\" if escaped else "") + row["sha256"] + "  " + name + "\n")
    atomic_bytes(output / "SHA256SUMS", "".join(lines).encode("utf-8"))
    return len(rows)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--workspace", required=True, type=Path)
    parser.add_argument("--matrix", required=True, type=Path)
    parser.add_argument("--build-evidence", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--baseline", default=BASELINE)
    parser.add_argument("--expected-trials", default=21, type=int)
    parser.add_argument("--diagnostic", action="append", type=Path, default=[],
                        help="additional workspace-relative raw evidence directory; repeatable")
    parser.add_argument("--no-smokes", action="store_true", help="omit automatic results/*smoke* discovery")
    args = parser.parse_args(argv)
    workspace, output = args.workspace.resolve(), args.output.resolve()
    if not workspace.is_dir() or args.expected_trials < 1:
        parser.error("workspace must exist and expected-trials must be positive")
    matrix = workspace_path(workspace, args.matrix)
    build = workspace_path(workspace, args.build_evidence)
    summary, summary_hash = require_finished_matrix(matrix, args.expected_trials)
    before = summary.get("source_before")
    after = summary.get("source_after")
    if not all(isinstance(record, dict) and isinstance(record.get("files"), dict)
               and isinstance(record.get("installed_files"), dict) for record in (before, after)):
        raise ValueError("complete source_before/source_after inventories are required")
    current = source_fingerprint(workspace)
    if current["source_sha256"] != after.get("source_sha256") or current["files"] != after["files"]:
        raise ValueError("current workspace source differs from matrix source_after; archive the matching source")
    required = [build / "source-tests.log", build / "install-audit.json"]
    if not all(path.is_file() for path in required):
        raise ValueError(f"build evidence must contain source-tests.log and install-audit.json: {build}")
    directories = [matrix, build, workspace / "results/mission-matrix-001",
                   workspace / "results/navigation-regression-001"]
    if not args.no_smokes:
        directories.extend(path.resolve() for path in (workspace / "results").glob("*smoke*") if path.is_dir())
    directories.extend(workspace_path(workspace, value) for value in args.diagnostic)
    if any(not inside(path, workspace) or inside(output, path) or inside(path, output) for path in directories):
        raise ValueError("output and evidence directories must be disjoint, and evidence must remain within workspace")
    index_name = git(workspace, "rev-parse", "--git-path", "index").decode().strip()
    default_index = (workspace / index_name).resolve()
    index_hash = digest(default_index) if default_index.exists() else None
    output.mkdir(parents=True, exist_ok=True)
    lock = output / ".archive.lock"
    with lock.open("x", encoding="utf-8") as stream:
        stream.write(str(os.getpid()))
    try:
        with tempfile.TemporaryDirectory(prefix="station-guard-archive-") as directory:
            staging = Path(directory) / "artifacts"
            staging.mkdir()
            source = build_source_artifacts(workspace, staging, Path(directory), args.baseline)
            artifacts = {name: (staging / name).read_bytes()
                         for name in ("feature.patch", "validated-feature-source.tar.gz")}
        copied = collect_evidence(workspace, output, directories)
        if digest(matrix / "summary.json") != summary_hash or source_fingerprint(workspace) != current:
            raise ValueError("matrix summary or source changed during archiving; output is incomplete")
        if (digest(default_index) if default_index.exists() else None) != index_hash:
            raise ValueError("workspace default index changed during archiving; inspect before accepting the archive")
        source.update(matrix=str(matrix.relative_to(workspace)),
                      source_before=before, source_after=after, current_source=current)
        for name, content in artifacts.items():
            atomic_bytes(output / name, content)
        atomic_bytes(output / "source-snapshot.json", json_bytes(source))
        atomic_bytes(output / "final-source-tests.log", required[0].read_bytes())
        atomic_bytes(output / "install-audit.json", required[1].read_bytes())
        atomic_bytes(output / "matrix-trials.csv", trial_csv(summary))
        concise = {"classification": "archive of recorded results; failures, blocked and not_run rows retained",
                   "created_utc": datetime.now(timezone.utc).isoformat(), "workspace": str(workspace),
                   "matrix": str(matrix.relative_to(workspace)), "summary_sha256": summary_hash,
                   "baseline_commit": source["baseline_commit"], "planned_trials": len(summary["results"]),
                   "counts": dict(Counter(row.get("result", "missing") for row in summary["results"])),
                   "result": summary.get("result"), "all_passed": summary.get("all_passed"),
                   "interrupted": summary.get("interrupted"), "source_unchanged": summary.get("source_unchanged"),
                   "installation_unchanged": summary.get("installation_unchanged"),
                   "source_sha256": after.get("source_sha256"), "install_sha256": after.get("install_sha256"),
                   "build_evidence": str(build.relative_to(workspace)), "raw_evidence": copied,
                   "missing_evidence": [row["source"] for row in copied if not row["available"]],
                   "results": [{key: row.get(key) for key in
                                ("trial_id", "scenario", "matrix_repeat", "domain_id", "result", "passed", "reason")}
                               for row in summary["results"]]}
        atomic_bytes(output / "archive-summary.json", json_bytes(concise))
        count = write_inventory(output)
        print(json.dumps({"output": str(output), "indexed_files": count,
                          "trial_rows": len(summary["results"]), "matrix_all_passed": summary.get("all_passed"),
                          "missing_evidence": concise["missing_evidence"]}, ensure_ascii=False))
    finally:
        lock.unlink(missing_ok=True)
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (OSError, ValueError, RuntimeError, subprocess.SubprocessError) as error:
        print(f"Archive not completed: {error}", file=sys.stderr)
        raise SystemExit(2)
