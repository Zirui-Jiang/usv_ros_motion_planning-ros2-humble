#!/usr/bin/env python3
"""Record source and installed artifacts without importing ROS or mutating them.

Use --phase before/during/after to describe when the snapshot was collected.
An after-run snapshot cannot prove that the same files were loaded during a run.
"""
import argparse
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import subprocess
import time


def git(workspace, *arguments):
    result = subprocess.run(["git", "-C", str(workspace), *arguments],
                            capture_output=True, text=True, encoding="utf-8", errors="replace")
    return {"exit_code": result.returncode, "stdout": result.stdout, "stderr": result.stderr}


def files_under(root):
    """Follow ament symlinked package directories, but avoid directory cycles."""
    visited = set()
    for current, directories, filenames in os.walk(root, followlinks=True):
        real = Path(current).resolve()
        if real in visited:
            directories[:] = []
            continue
        visited.add(real)
        directories[:] = sorted(name for name in directories if name not in (".git", "__pycache__", ".pytest_cache"))
        for name in sorted(filenames):
            if not name.endswith((".pyc", ".pyo")):
                yield Path(current) / name


def fingerprint(path):
    try:
        before = path.stat()
        digest = hashlib.sha256()
        with path.open("rb") as stream:
            for block in iter(lambda: stream.read(1024 * 1024), b""):
                digest.update(block)
        after = path.stat()
        return {"resolved_path": str(path.resolve()), "bytes": after.st_size,
                "sha256": digest.hexdigest(), "mtime_ns": after.st_mtime_ns,
                "changed_during_read": (before.st_size, before.st_mtime_ns) != (after.st_size, after.st_mtime_ns)}
    except OSError as error:
        return {"error": str(error)}


def capture(workspace):
    result = {"path": str(workspace), "commit": git(workspace, "rev-parse", "HEAD"),
              "status": git(workspace, "status", "--porcelain"),
              "diff": git(workspace, "diff", "HEAD", "--binary"), "artifacts": {}}
    for directory in ("src", "install", "scripts/validation"):
        root = workspace / directory
        result["artifacts"][directory] = {str(path.relative_to(workspace)): fingerprint(path)
                                             for path in files_under(root)} if root.is_dir() else None
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--workspace", action="append", required=True, metavar="LABEL=PATH")
    parser.add_argument("--phase", choices=("before", "during", "after"), required=True)
    parser.add_argument("--run-directory", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path, help="new JSON file")
    parser.add_argument("--invocation", help="reported run command; not reconstructed from a process")
    args = parser.parse_args()
    if args.output.exists():
        parser.error("output already exists")
    workspaces = {}
    for value in args.workspace:
        label, separator, path = value.partition("=")
        if not separator or not label or label in workspaces:
            parser.error("each workspace needs a unique LABEL=PATH")
        workspaces[label] = Path(path).resolve()
        if not workspaces[label].is_dir():
            parser.error("workspace does not exist: " + path)
    begun = time.time()
    report = {"schema_version": 1, "phase": args.phase,
              "collected_start_utc": datetime.now(timezone.utc).isoformat(),
              "run_directory": str(args.run_directory.resolve()),
              "reported_invocation": args.invocation,
              "scope": "Source, install and validation files at collection time; symlink targets followed. No assertion that after-run files equal files loaded during execution.",
              "workspaces": {label: capture(path) for label, path in workspaces.items()}}
    report["collected_end_utc"] = datetime.now(timezone.utc).isoformat()
    report["collection_wall_s"] = time.time() - begun
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open("x", encoding="utf-8") as stream:
        json.dump(report, stream, ensure_ascii=False, indent=2, allow_nan=False)
    print(json.dumps({"output": str(args.output), "phase": args.phase,
                      "collection_wall_s": report["collection_wall_s"],
                      "workspaces": {label: {kind: len(files) if files is not None else None
                                               for kind, files in value["artifacts"].items()}
                                     for label, value in report["workspaces"].items()}}))


if __name__ == "__main__":
    main()
