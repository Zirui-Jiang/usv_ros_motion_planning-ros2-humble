# 用临时工程验证导出边界、拒绝覆盖、链接适配和归档可重复性，不修改真实源包。
"""Exercise the export boundary without copying unrelated repository content."""

import hashlib
import json
from pathlib import Path
import subprocess
import sys
import zipfile

import pytest


EXPORT_SCRIPT = Path(__file__).with_name("export_control_workspace.py")


# 建立含源码、缓存和私有文件的临时夹具，用来验证导出器只选取允许的文件。
@pytest.fixture
def source_tree(tmp_path):
    root = tmp_path / "source"
    files = {
        "src/core/shared/common/common/models.py": "MODEL = 'working-tree-value'\n",
        "src/core/shared/common/common/dynamics.py": "DYNAMICS = True\n",
        "src/core/shared/common/test/test_models.py": "def test_model(): pass\n",
        "src/core/shared/common/resource/common": "",
        "src/core/shared/common/package.xml": "<package><name>common</name></package>\n",
        "src/core/decision_module/trajectory_planner/trajectory_planner/trajectory.py": "TRAJECTORY = True\n",
        "src/core/control_module/controller/controller/station_keeping_controller.py": "KEEPING = True\n",
        "src/core/control_module/controller/controller/pid_controller.py": "PID = True\n",
        "scripts/control/README.md": "# Control workspace\n",
        "scripts/control/control_demo.py": "print('demo')\n",
        "docs/control-autopilot-interface.md": "# Interface\n",
        "src/core/shared/common/common/__pycache__/models.pyc": "cache",
        "src/core/shared/common/common/local.egg-info/private.py": "metadata",
        "src/core/shared/common/.git/config": "secret",
        "src/core/shared/common/log/private.log": "private log",
        "src/core/shared/common/test/run_capture.csv": "private capture",
        "src/core/shared/common/common/.env": "private environment",
        "src/core/radar/private.py": "private radar",
        "private.log": "private root log",
    }
    for relative, content in files.items():
        target = root / relative
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_text(content, encoding="utf-8")
    return root


# 以独立 Python 进程运行真实导出脚本，保存退出码与输出，不依赖当前进程导入状态。
def run_export(source, output, archive):
    return subprocess.run(
        [sys.executable, "-X", "utf8", str(EXPORT_SCRIPT), "--repo-root", str(source),
         "--output", str(output), "--archive", str(archive)],
        capture_output=True, text=True, encoding="utf-8",
    )


# 核对保留的算法和包文件、排除的私有数据，以及逐文件来源和哈希记录。
def test_exports_only_control_sources_and_verifiable_provenance(source_tree, tmp_path):
    output, archive = tmp_path / "kit", tmp_path / "kit.zip"
    result = run_export(source_tree, output, archive)
    assert result.returncode == 0, result.stderr
    assert (output / "src/controller/controller/station_keeping_controller.py").is_file()
    assert (output / "src/common/common/dynamics.py").is_file()
    assert (output / "docs/control-autopilot-interface.md").is_file()
    manifest = json.loads((output / "provenance.json").read_text(encoding="utf-8"))
    entries = {entry["relative_path"]: entry for entry in manifest["files"]}
    model = output / "src/common/common/models.py"
    assert model.read_text(encoding="utf-8") == "MODEL = 'working-tree-value'\n"
    assert entries["src/common/common/models.py"]["source_path"] == "src/core/shared/common/common/models.py"
    assert entries["src/common/common/models.py"]["sha256"] == hashlib.sha256(model.read_bytes()).hexdigest()
    actual = {path.relative_to(output).as_posix() for path in output.rglob("*") if path.is_file()}
    assert actual == set(entries) | {"provenance.json"}
    assert not any(token in path for path in actual for token in
                   ("__pycache__", ".pyc", ".egg-info", ".git", "private", "capture", ".env", "radar"))
    assert manifest["git"]["head"] is None
    with zipfile.ZipFile(archive) as bundle:
        assert set(bundle.namelist()) == {"control_workspace/" + name for name in actual}


# 目标目录或归档已存在时必须拒绝操作，哨兵内容保持不变。
@pytest.mark.parametrize("existing", ["directory", "archive"])
def test_existing_destination_is_not_overwritten(source_tree, tmp_path, existing):
    output, archive = tmp_path / "kit", tmp_path / "kit.zip"
    if existing == "directory":
        output.mkdir()
        sentinel = output / "keep.txt"
    else:
        sentinel = archive
    sentinel.write_bytes(b"user-owned")
    result = run_export(source_tree, output, archive)
    assert result.returncode != 0
    assert "already exists" in result.stderr
    assert sentinel.read_bytes() == b"user-owned"
    if existing == "archive":
        assert not output.exists()
    else:
        assert sorted(p.name for p in output.iterdir()) == ["keep.txt"]
        assert not archive.exists()


# 相同输入两次导出应生成相同 ZIP 字节，避免时间戳等无关因素影响结果。
def test_repeated_export_has_identical_archive_bytes(source_tree, tmp_path):
    archives = []
    for name in ("first", "second"):
        archive = tmp_path / (name + ".zip")
        result = run_export(source_tree, tmp_path / name, archive)
        assert result.returncode == 0, result.stderr
        archives.append(archive.read_bytes())
    assert archives[0] == archives[1]


# 归档不能位于自身导出目录中，避免递归打包或生成不完整输出。
def test_archive_inside_output_is_rejected_before_any_write(source_tree, tmp_path):
    output = tmp_path / "kit"
    result = run_export(source_tree, output, output / "kit.zip")
    assert result.returncode != 0
    assert "outside the output directory" in result.stderr
    assert not output.exists()


# 禁止归档写回原源包内部，保护原工作树文件。
def test_archive_cannot_write_into_a_source_package(source_tree, tmp_path):
    output = tmp_path / "kit"
    archive = source_tree / "src/core/shared/common/export.zip"
    result = run_export(source_tree, output, archive)
    assert result.returncode != 0
    assert "outside source package directories" in result.stderr
    assert not output.exists()
    assert not archive.exists()


# 导出后文档链接仍可解释，链接转换及原文哈希必须被记录。
def test_interface_links_are_portable_and_transformation_is_recorded(source_tree, tmp_path):
    document = source_tree / "docs/control-autopilot-interface.md"
    document.write_text(
        "# Interface\n[State](../src/core/shared/common/common/models.py)\n"
        "[PID](../src/core/control_module/controller/controller/pid_controller.py)\n"
        "[Trajectory](../src/core/decision_module/trajectory_planner/trajectory_planner/trajectory.py)\n"
        "[Scheduler](../src/sim_env/sim_env/simulation.py)\n", encoding="utf-8")
    output, archive = tmp_path / "kit", tmp_path / "kit.zip"
    result = run_export(source_tree, output, archive)
    assert result.returncode == 0, result.stderr
    exported = output / "docs/control-autopilot-interface.md"
    text = exported.read_text(encoding="utf-8")
    assert "[State](../src/common/common/models.py)" in text
    assert "[PID](../src/controller/controller/pid_controller.py)" in text
    assert "[Trajectory](../src/trajectory_planner/trajectory_planner/trajectory.py)" in text
    assert "../src/sim_env" not in text
    assert "`src/sim_env/sim_env/simulation.py`" in text
    manifest = json.loads((output / "provenance.json").read_text(encoding="utf-8"))
    entry = next(value for value in manifest["files"] if value["relative_path"] == "docs/control-autopilot-interface.md")
    assert entry["source_sha256"] == hashlib.sha256(document.read_bytes()).hexdigest()
    assert entry["sha256"] == hashlib.sha256(exported.read_bytes()).hexdigest()
    assert entry["source_sha256"] != entry["sha256"]
    assert entry["transformation"]


# 独立包只跳过明确依赖未导出 world 的夹具，其余地理转换测试仍必须运行。
def test_standalone_collection_skips_only_known_world_fixture_cases(source_tree, tmp_path):
    common_test = source_tree / "src/core/shared/common/test/test_geodesy.py"
    common_test.write_text('''import pytest
def test_world_reference_matches_current_world_and_example_point():
    raise AssertionError("sim_env world is not exported")
@pytest.mark.parametrize("change", ["absent", "NED", "heading", "surface", "latitude", "elevation"])
def test_unsupported_or_incomplete_world_rejected(change):
    raise AssertionError("sim_env world is not exported")
def test_unrelated_geodesy_failure_is_not_hidden():
    assert False
''', encoding="utf-8")
    other_test = source_tree / "src/core/shared/common/test/test_other.py"
    other_test.write_text('''def test_world_reference_matches_current_world_and_example_point():
    assert False
''', encoding="utf-8")
    output, archive = tmp_path / "kit", tmp_path / "kit.zip"
    result = run_export(source_tree, output, archive)
    assert result.returncode == 0, result.stderr
    check = subprocess.run(
        [sys.executable, "-I", "-B", "-m", "pytest", "--confcutdir=.", "-q", "-p", "no:cacheprovider"],
        cwd=output, capture_output=True, encoding="utf-8", errors="replace",
    )
    assert check.returncode == 1, check.stdout + check.stderr
    assert "2 failed, 1 passed, 7 skipped" in check.stdout
