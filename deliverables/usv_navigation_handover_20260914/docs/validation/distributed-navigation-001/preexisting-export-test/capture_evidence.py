"""Capture targeted diagnostic evidence; no repository implementation edits."""
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import shlex
import shutil
import subprocess
import time
import xml.etree.ElementTree as ET


OUTPUT = Path(__file__).resolve().parent
CURRENT = OUTPUT.parents[3]
BASELINE = Path('/home/usv/distributed-navigation-baseline-20260913')
CURRENT_INSTALL = Path('/home/usv/distributed-navigation-build-20260913/install/setup.bash')
TARGET = 'scripts/control/test_export_control_workspace.py::test_standalone_collection_skips_only_known_world_fixture_cases'
COMMAND = ['python3', '-m', 'pytest', '-q', TARGET, '-p', 'no:cacheprovider']


def capture(name, cwd, install, pytest_addopts=None):
    prefix = ['source /opt/ros/humble/setup.bash', 'source ' + shlex.quote(str(install)),
              'export ROS_DOMAIN_ID=166', 'export ROS_LOCALHOST_ONLY=1']
    if pytest_addopts is None:
        prefix.append('unset PYTEST_ADDOPTS')
    else:
        prefix.append('export PYTEST_ADDOPTS=' + shlex.quote(pytest_addopts))
    command = '; '.join(prefix + [shlex.join(COMMAND)])
    version_command = '; '.join(prefix + ['python3 -m pytest --version'])
    started = datetime.now(timezone.utc).isoformat()
    before = time.monotonic()
    result = subprocess.run(['bash', '-lc', command], cwd=cwd, capture_output=True,
                            text=True, encoding='utf-8', errors='replace')
    elapsed = time.monotonic() - before
    version = subprocess.run(['bash', '-lc', version_command], cwd=cwd, capture_output=True,
                             text=True, encoding='utf-8', errors='replace')
    (OUTPUT / (name + '.command.txt')).write_text('cwd: ' + str(cwd) + '\n' + command + '\n', encoding='utf-8')
    (OUTPUT / (name + '.stdout.log')).write_text(result.stdout, encoding='utf-8')
    (OUTPUT / (name + '.stderr.log')).write_text(result.stderr, encoding='utf-8')
    record = dict(command=command, cwd=str(cwd), started_utc=started,
                  exit_code=result.returncode, elapsed_monotonic_seconds=elapsed,
                  pytest_addopts=pytest_addopts, pytest_version_command=version_command,
                  pytest_version_stdout=version.stdout, pytest_version_stderr=version.stderr,
                  pytest_version_exit_code=version.returncode)
    (OUTPUT / (name + '.json')).write_text(json.dumps(record, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
    return dict(name=name, exit_code=result.returncode,
                pytest_version=(version.stdout + version.stderr).strip())


def main():
    summaries = []
    for name, root, install in [('current', CURRENT, CURRENT_INSTALL),
                                ('baseline', BASELINE, BASELINE / 'install/setup.bash')]:
        summaries.append(capture(name + '-without-testpath-override', root, install))
        summaries.append(capture(name + '-with-baseline-testpath-override', root, install,
                                 '-o testpaths=src/common/test'))
    hashes = {}
    for relative in ('scripts/control/export_control_workspace.py',
                     'scripts/control/test_export_control_workspace.py'):
        hashes[relative] = {label: hashlib.sha256((root / relative).read_bytes()).hexdigest()
                            for label, root in [('current', CURRENT), ('baseline', BASELINE)]}
    (OUTPUT / 'source-hashes.json').write_text(json.dumps(hashes, indent=2) + '\n', encoding='utf-8')
    original = BASELINE / 'validation'
    copies = ('full-source-test-command.txt', 'full-source-tests.log', 'full-source-tests.xml',
              'source-test-command.txt', 'source-tests.log')
    for filename in copies:
        shutil.copyfile(original / filename, OUTPUT / ('original-baseline-' + filename))
    shutil.copyfile(BASELINE / 'pytest.ini', OUTPUT / 'baseline-pytest.ini.txt')
    shutil.copyfile(BASELINE / 'src/core/usv_mission/validation/run_source_tests.bash',
                    OUTPUT / 'baseline-run_source_tests.bash.txt')
    xml = ET.parse(original / 'full-source-tests.xml')
    cases = xml.findall('.//testcase')
    target_cases = [item for item in cases if item.attrib.get('name') == TARGET.split('::')[-1]]
    scope = dict(original_command=(original / 'full-source-test-command.txt').read_text(),
                 testcase_count=len(cases),
                 target_cases=[dict(attributes=item.attrib, child_tags=[child.tag for child in item])
                               for item in target_cases],
                 explanation='The outer invocation explicitly selects src scripts. PYTEST_ADDOPTS overrides '
                             'testpaths only for the nested pytest invocation, which has no positional test path.')
    (OUTPUT / 'baseline-scope.json').write_text(json.dumps(scope, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
    (OUTPUT / 'runs-summary.json').write_text(json.dumps(summaries, indent=2) + '\n', encoding='utf-8')
    print(json.dumps(dict(runs=summaries, baseline_testcases=len(cases),
                         target_case_count=len(target_cases)), ensure_ascii=False))


if __name__ == '__main__':
    main()
