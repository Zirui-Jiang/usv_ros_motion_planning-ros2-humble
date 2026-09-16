"""Archive this reorganization's source hashes and completed build/test evidence."""
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import xml.etree.ElementTree as ET


def main():
    output = Path(__file__).resolve().parent
    repo = output.parents[2]
    workspace = Path('/home/usv/module-layout-20260914')
    for name in ('source-before-manifest.json', 'source-before-summary.json'):
        destination = output / name
        if destination.exists():
            raise FileExistsError(destination)
        shutil.copy2(workspace / name, destination)
    build_logs = output / 'build'
    build_logs.mkdir()
    for name in ('logger_all.log', 'events.log'):
        shutil.copy2(workspace / 'log/latest_build' / name, build_logs / name)
    files = [path for directory in ('src', 'scripts') for path in (repo / directory).rglob('*')
             if path.is_file() and not any(part in ('__pycache__', '.pytest_cache')
                                          or part.endswith('.egg-info') for part in path.parts)
             and path.suffix not in ('.pyc', '.pyo')]
    files += [repo / name for name in ('README.md', 'conftest.py', 'pytest.ini')]
    manifest = {str(path.relative_to(repo)): hashlib.sha256(path.read_bytes()).hexdigest()
                for path in sorted(files)}
    (output / 'source-after-manifest.json').write_text(
        json.dumps(manifest, indent=2, ensure_ascii=False) + '\n', encoding='utf-8')
    suites = ET.parse(output / 'source-tests-final/tests.xml').getroot()
    totals = {key: sum(int(suite.get(key, '0')) for suite in suites.iter('testsuite'))
              for key in ('tests', 'failures', 'errors', 'skipped')}
    install = json.loads((output / 'install-audit.json').read_text())
    preservation = json.loads((output / 'preservation.json').read_text())
    smoke = json.loads((output / 'process-smoke/summary.json').read_text())
    yaml_probe = json.loads((output / 'yaml/summary.json').read_text())
    gazebo = json.loads((output / 'gazebo/summary.json').read_text())
    packages = {ET.parse(path).getroot().findtext('name'): str(path.parent.relative_to(repo))
                for path in sorted((repo / 'src').rglob('package.xml'))}
    report = dict(
        branch=subprocess.check_output(['git', 'branch', '--show-current'], cwd=repo, text=True).strip(),
        head=subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=repo, text=True).strip(),
        source_state='Uncommitted worktree; HEAD alone does not identify these tested sources.',
        workspace=str(workspace), source_before=str(workspace / 'source-before'),
        source_after_file_count=len(manifest), packages=packages, package_count=len(packages),
        source_tests=totals,
        source_tests_report='source-tests-final/tests.xml',
        initial_source_tests_report='source-tests/tests.xml',
        initial_source_tests_result='925 passed, 1 legacy process-worker timeout; isolated rerun passed. '
                                    'The exact transient cause was not established; no assertion or algorithm was changed.',
        install_audit=dict(passed=install['all_passed'], checks=len(install['checks'])),
        preservation=dict(passed=preservation['passed'], checks=len(preservation['checks'])),
        process_smoke=smoke, yaml_probe=yaml_probe,
        navigation=dict(passed=gazebo['all_passed'], checks=len(gazebo['results']),
                        run_finished=gazebo['run_finished']),
    )
    (output / 'validation-summary.json').write_text(
        json.dumps(report, indent=2, ensure_ascii=False) + '\n', encoding='utf-8')
    print(json.dumps({key: value for key, value in report.items()
                      if key not in ('packages', 'process_smoke', 'yaml_probe')}, indent=2))


if __name__ == '__main__':
    main()
