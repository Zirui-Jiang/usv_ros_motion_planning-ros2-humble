#!/usr/bin/env python3
# 从当前源码提取控制算法及其依赖，保留文件来源与哈希；不复制无关传感器资料或运行缓存。
"""Export the current control source tree without copying unrelated project data."""

import argparse
import hashlib
import json
from pathlib import Path
import re
import subprocess
import sys
import zipfile


PACKAGES = {
    "common": "src/core/shared/common",
    "trajectory_planner": "src/core/decision_module/trajectory_planner",
    "controller": "src/core/control_module/controller",
}
METADATA_FILES = {"setup.py", "setup.cfg", "package.xml", "README.md", "LICENSE", "LICENSE.txt"}
EXCLUDED_DIRS = {"__pycache__", ".pytest_cache", ".mypy_cache", ".ruff_cache", ".git",
                 "build", "install", "log", "logs"}
GENERATED_FILES = {
    "requirements.txt": "numpy\npytest\n",
    "conftest.py": '''"""Local source imports for the independent control workspace."""
from pathlib import Path
import sys
import pytest

ROOT = Path(__file__).resolve().parent
for package in ("common", "trajectory_planner", "controller"):
    sys.path.insert(0, str(ROOT / "src" / package))


def pytest_collection_modifyitems(items):
    world_tests = ROOT / "src/common/test/test_geodesy.py"
    names = {"test_world_reference_matches_current_world_and_example_point",
             "test_unsupported_or_incomplete_world_rejected"}
    for item in items:
        # Exact source file and function names, retaining all other numerical tests.
        if (Path(str(item.fspath)).resolve() == world_tests
                and getattr(item, "originalname", item.name) in names):
            item.add_marker(pytest.mark.skip(
                reason="Integration fixture sim_env/worlds/ocean.world is not included in the control snapshot"))
''',
    "pytest.ini": "[pytest]\ntestpaths = src/common/test src/trajectory_planner/test src/controller/test\naddopts = --import-mode=importlib\n",
}


# 将导出包内可用的源码链接改为相对路径，未导出的依赖明确标注仅在原仓库中。
def portable_interface_document(payload):
    text = payload.decode("utf-8")
    for package, original in PACKAGES.items():
        text = text.replace("../" + original + "/", "../src/" + package + "/")
    text = re.sub(r"\[([^\]]+)\]\(\.\./(src/sim_env/[^)]+)\)",
                  lambda match: f"{match[1]}（仅原仓库：`{match[2]}`）", text)
    note = ("\n> 快照链接说明：三个数值包的源码链接已适配本工作区；"
            "sim_env 证据路径仅在原仓库中可访问，本快照不包含该仿真包。\n")
    title, separator, remainder = text.partition("\n")
    return (title + "\n" + note + (remainder if separator else "")).encode("utf-8")


# 按文件类型和目录过滤复制对象，防止缓存、私有环境配置与采集数据混入导出物。
def eligible(relative, package):
    """Copy Python sources/tests and known ROS package metadata only."""
    if any(part in EXCLUDED_DIRS or part.startswith(".") or part.endswith(".egg-info")
           for part in relative.parts):
        return False
    if len(relative.parts) == 1:
        return relative.name in METADATA_FILES
    if relative.parts == ("resource", package):
        return True
    return relative.parts[0] in {package, "test"} and relative.suffix == ".py"


# 记录源包所在工作树的版本与修改状态，不用 Git HEAD 替代尚未提交的文件内容。
def git_provenance(root):
    def run(*arguments):
        try:
            result = subprocess.run(
                ["git", "-C", str(root), "-c", "core.quotepath=false", *arguments],
                capture_output=True, encoding="utf-8", errors="replace", timeout=10,
            )
        except (OSError, subprocess.TimeoutExpired):
            return None
        return result.stdout.rstrip("\r\n") if result.returncode == 0 else None

    head = run("rev-parse", "HEAD")
    status = run("status", "--short", "--untracked-files=all", "--", *PACKAGES.values())
    return {
        "head": head,
        "dirty_source_packages": bool(status) if status is not None else None,
        "status_scope": list(PACKAGES.values()),
        "status_short": status.splitlines() if status else [],
        "note": "Snapshot uses current filesystem contents, including uncommitted source changes. "
                "Git status is limited to the three source packages; null means unavailable.",
    }


# 先检查输出边界，再复制当前文件并记录哈希；归档已有时拒绝覆盖。
def export_workspace(repo_root, output, archive):
    root = Path(repo_root).resolve(strict=True)
    output = Path(output).absolute()
    archive = Path(archive).absolute()
    # Reject before mkdir/read/copy, including existing files and dangling links.
    for target in (output, archive):
        if target.exists() or target.is_symlink():
            raise FileExistsError(f"Destination already exists: {target}")
    output, archive = output.resolve(), archive.resolve()
    if archive == output or output in archive.parents:
        raise ValueError("Archive must be outside the output directory")

    contents = {}
    entries = []

    def add_source(source, destination, transform=None):
        resolved = source.resolve(strict=True)
        if source.is_symlink() or root not in resolved.parents:
            raise ValueError(f"Source must be a regular file inside the repository: {source}")
        payload = source.read_bytes()
        source_sha256 = hashlib.sha256(payload).hexdigest()
        if transform is not None:
            payload = transform(payload)
        contents[destination] = payload
        entry = {"relative_path": destination,
                 "source_path": source.relative_to(root).as_posix(),
                 "sha256": hashlib.sha256(payload).hexdigest()}
        if transform is not None:
            entry.update(source_sha256=source_sha256,
                         transformation="Remap exported package links; annotate omitted sim_env evidence as original-repository paths.")
        entries.append(entry)

    for package, source_relative in PACKAGES.items():
        source_root = root / source_relative
        if not source_root.is_dir():
            raise FileNotFoundError(f"Required package directory is missing: {source_root}")
        if any(target == source_root.resolve() or source_root.resolve() in target.parents
               for target in (output, archive)):
            raise ValueError("Output and archive must be outside source package directories")
        for source in sorted(source_root.rglob("*")):
            relative = source.relative_to(source_root)
            if source.is_file() and eligible(relative, package):
                # Reject directory junctions and symlinks escaping the package as well.
                if source_root.resolve() not in source.resolve().parents:
                    raise ValueError(f"Source escapes its package directory: {source}")
                add_source(source, (Path("src") / package / relative).as_posix())

    add_source(root / "scripts/control/README.md", "README.md")
    add_source(root / "scripts/control/control_demo.py", "examples/control_demo.py")
    interface_doc = root / "docs/control-autopilot-interface.md"
    if interface_doc.is_file():
        add_source(interface_doc, "docs/control-autopilot-interface.md", portable_interface_document)
    for name, text in GENERATED_FILES.items():
        payload = text.encode("utf-8")
        contents[name] = payload
        entries.append({"relative_path": name, "source_path": None,
                        "generated_by": "scripts/control/export_control_workspace.py",
                        "sha256": hashlib.sha256(payload).hexdigest()})

    manifest = {
        "schema_version": 1,
        "description": "Independent development snapshot; control algorithms are copied unchanged.",
        "git": git_provenance(root),
        "package_source_paths": PACKAGES,
        "scope": "Python modules/tests and known ROS package metadata in the three package paths; "
                 "no simulator, radar packages, logs, caches, bytecode, egg-info, or .git directory.",
        "files": sorted(entries, key=lambda entry: entry["relative_path"]),
        "manifest_note": "SHA256 covers every exported file except this manifest. "
                         "Generated helper files have null source_path. "
                         "No export timestamp is used so identical inputs and Git state produce identical archives.",
    }
    contents["provenance.json"] = (json.dumps(manifest, ensure_ascii=False, indent=2) + "\n").encode("utf-8")

    # Exclusive creation; existing destinations are never deleted or replaced.
    output.mkdir(parents=True, exist_ok=False)
    for name, payload in sorted(contents.items()):
        target = output / name
        target.parent.mkdir(parents=True, exist_ok=True)
        with target.open("xb") as handle:
            handle.write(payload)
    archive.parent.mkdir(parents=True, exist_ok=True)
    with zipfile.ZipFile(archive, mode="x", compression=zipfile.ZIP_DEFLATED, compresslevel=9) as bundle:
        for name, payload in sorted(contents.items()):
            info = zipfile.ZipInfo("control_workspace/" + name, date_time=(1980, 1, 1, 0, 0, 0))
            info.create_system = 3
            info.external_attr = 0o100644 << 16
            info.compress_type = zipfile.ZIP_DEFLATED
            bundle.writestr(info, payload, compresslevel=9)
    return {"output": str(output), "archive": str(archive), "file_count": len(contents),
            "archive_sha256": hashlib.sha256(archive.read_bytes()).hexdigest()}


# 命令行入口：解析本脚本选项并执行上述流程，运行结果由退出码及输出记录共同说明。
def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--repo-root", type=Path, default=Path(__file__).resolve().parents[2])
    parser.add_argument("--output", type=Path, help="New directory; defaults to REPO/deliverables/control_workspace")
    parser.add_argument("--archive", type=Path, help="New ZIP file outside output; defaults to OUTPUT.zip")
    args = parser.parse_args()
    output = args.output if args.output is not None else args.repo_root / "deliverables/control_workspace"
    archive = args.archive if args.archive is not None else output.with_name(output.name + ".zip")
    try:
        result = export_workspace(args.repo_root, output, archive)
    except (OSError, ValueError) as error:
        print(f"Export failed: {error}", file=sys.stderr)
        return 1
    print(json.dumps(result, ensure_ascii=True, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
