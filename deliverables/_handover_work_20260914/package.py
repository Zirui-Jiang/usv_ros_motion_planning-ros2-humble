"""Create and verify a source-only handover snapshot without changing the project."""
from pathlib import Path
from datetime import datetime, timezone
import hashlib
import json
import shutil
import subprocess
import sys
import zipfile
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[2]
KIT = ROOT / 'deliverables/usv_navigation_handover_20260914'
ARCHIVE = KIT.with_suffix('.zip')
WORK = Path(__file__).resolve().parent
CACHE = {'.git', '__pycache__', '.pytest_cache', '.mypy_cache', '.ruff_cache',
         '.venv', 'venv', 'build', 'install', '.superpowers', '.idea', '.vscode'}


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def excluded(path):
    parts = path.relative_to(ROOT).parts
    if set(parts) & CACHE or any(p.endswith('.egg-info') for p in parts):
        return True
    if path.suffix in {'.pyc', '.pyo'}:
        return True
    if parts[0] == 'docs':
        if {'raw', 'superpowers'} & set(parts):
            return True
        if path.name.endswith(('.tar.gz', '.zip')):
            return True
        if path.suffix == '.jsonl' and parts[:3] != ('docs', 'validation', 'module-layout-001'):
            return True
    return False


def write_json(path, value):
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')


def stage():
    if KIT.exists() or ARCHIVE.exists():
        raise SystemExit('Refusing to replace an existing handover.')
    files = [ROOT / p for p in ['README.md', 'CONTEXT.md', 'conftest.py', 'pytest.ini',
                                '.gitignore', '.clang-format', 'deliverables/README.md']]
    for directory in ['src', 'scripts', 'docs', 'docker']:
        files.extend(p for p in (ROOT / directory).rglob('*') if p.is_file() and not excluded(p))
    rows = []
    for src in sorted(files):
        if src.is_symlink():
            raise SystemExit(f'Unexpected symbolic link: {src}')
        rel = src.relative_to(ROOT)
        dst = KIT / rel
        dst.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(src, dst)
        if digest(src) != digest(dst):
            raise SystemExit(f'Copy mismatch: {rel}')
        rows.append({'path': rel.as_posix(), 'sha256': digest(dst), 'bytes': dst.stat().st_size})
    packages = {ET.parse(p).getroot().findtext('name'): p.parent.relative_to(KIT).as_posix()
                for p in (KIT / 'src').rglob('package.xml')}
    assert len(packages) == 20
    git = lambda *args: subprocess.check_output(['git', *args], cwd=ROOT).decode('utf-8').strip()
    provenance = {
        'created_at': datetime.now(timezone.utc).isoformat(),
        'source_type': 'Current uncommitted working tree, not git archive HEAD',
        'git_head': git('rev-parse', 'HEAD'), 'git_branch': git('branch', '--show-current'),
        'git_status_for_included_paths': git('-c', 'core.quotePath=false', 'status', '--short', '--',
            'src', 'scripts', 'docs', 'docker', 'README.md', 'CONTEXT.md', 'conftest.py', 'pytest.ini'),
        'source_files': rows, 'ros_packages': packages,
        'excluded': ['Git history, build/install/runtime logs/caches/editor settings',
                     'Older deliverables except the explanatory README', 'Root adrc and USV_control reference folders',
                     'docs/superpowers, docs/**/raw, old archives, historical JSONL except module-layout-001'],
        'source_files_byte_preserved': True,
    }
    (KIT / 'handover').mkdir(exist_ok=True)
    write_json(KIT / 'handover/SOURCE_PROVENANCE.json', provenance)
    old = json.loads((ROOT / 'docs/validation/module-layout-001/source-after-manifest.json').read_text())
    changes = [p for p, sha in old.items() if not (KIT / p).is_file() or digest(KIT / p) != sha]
    write_json(KIT / 'handover/HISTORICAL_COMPARISON.json', {
        'reference': 'docs/validation/module-layout-001/source-after-manifest.json',
        'reference_file_count': len(old), 'changed_or_missing': changes,
        'matching': len(old) - len(changes),
        'note': 'This hash comparison does not replace a fresh build or runtime validation.'})
    print(json.dumps({'files_copied': len(rows), 'packages': len(packages),
                      'historical_changed_or_missing': changes, 'kit': str(KIT)}, ensure_ascii=False))


def finalize():
    if ARCHIVE.exists():
        raise SystemExit('Refusing to overwrite archive.')
    provenance = json.loads((KIT / 'handover/SOURCE_PROVENANCE.json').read_text(encoding='utf-8'))
    for row in provenance['source_files']:
        assert digest(KIT / row['path']) == row['sha256'], row['path']
        assert digest(ROOT / row['path']) == row['sha256'], 'Source changed during packaging: ' + row['path']
    files = sorted(p for p in KIT.rglob('*') if p.is_file()
                   and not (set(p.relative_to(KIT).parts) & CACHE) and p.suffix not in {'.pyc', '.pyo'})
    manifest = {'format': 1, 'note': 'SHA-256 covers every archived file except this manifest itself.',
                'files': [{'path': p.relative_to(KIT).as_posix(), 'bytes': p.stat().st_size,
                           'sha256': digest(p)} for p in files]}
    write_json(KIT / 'PACKAGE_MANIFEST.json', manifest)
    files.append(KIT / 'PACKAGE_MANIFEST.json')
    with zipfile.ZipFile(ARCHIVE, 'x', compression=zipfile.ZIP_DEFLATED, compresslevel=6) as z:
        for p in sorted(files):
            data = p.read_bytes()
            info = zipfile.ZipInfo(KIT.name + '/' + p.relative_to(KIT).as_posix(), (2026, 9, 14, 0, 0, 0))
            info.create_system = 3
            executable = p.suffix in {'.sh', '.bash'} or data.startswith(b'#!')
            info.external_attr = (0o100755 if executable else 0o100644) << 16
            info.compress_type = zipfile.ZIP_DEFLATED
            z.writestr(info, data)
    with zipfile.ZipFile(ARCHIVE) as z:
        assert z.testzip() is None
        assert len(z.namelist()) == len(files)
        for row in manifest['files']:
            data = z.read(KIT.name + '/' + row['path'])
            assert hashlib.sha256(data).hexdigest() == row['sha256'], row['path']
    checksum = digest(ARCHIVE)
    ARCHIVE.with_suffix('.zip.sha256').write_text(checksum + '  ' + ARCHIVE.name + '\n', encoding='ascii')
    summary = {'archive': ARCHIVE.name, 'bytes': ARCHIVE.stat().st_size,
               'files': len(files), 'ros_packages': len(provenance['ros_packages']),
               'sha256': checksum, 'zip_crc_and_all_file_hashes': 'passed'}
    write_json(WORK / 'archive-check.json', summary)
    print(json.dumps(summary, ensure_ascii=False))


if __name__ == '__main__':
    {'stage': stage, 'finalize': finalize}[sys.argv[1]]()
