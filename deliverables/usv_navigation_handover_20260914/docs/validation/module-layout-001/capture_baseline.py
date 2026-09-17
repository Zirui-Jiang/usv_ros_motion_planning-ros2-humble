"""Capture the authorized dirty source before reorganizing package ownership."""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import xml.etree.ElementTree as ET


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    source, output = args.source.resolve(), args.output.resolve()
    if output.exists():
        parser.error('snapshot output already exists')
    output.mkdir(parents=True)
    files = []
    for folder in ('src', 'scripts'):
        files.extend(path for path in (source / folder).rglob('*')
                     if path.is_file() and not any(
                         part in ('__pycache__', '.pytest_cache', '.git') for part in path.parts))
    files.extend(source / name for name in ('README.md', 'conftest.py', 'pytest.ini'))
    manifest = []
    for path in sorted(files):
        relative = path.relative_to(source)
        destination = output / relative
        destination.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(path, destination)
        before = digest(path)
        assert digest(destination) == before, relative
        manifest.append(dict(path=relative.as_posix(), sha256=before))
    packages = {ET.parse(path).getroot().findtext('name'): path.parent.relative_to(source).as_posix()
                for path in sorted((source / 'src').rglob('package.xml'))}
    result = dict(source=str(source), snapshot=str(output), files=len(manifest),
                  verified=True, head=subprocess.check_output(
                      ['git', '-C', str(source), 'rev-parse', 'HEAD'], text=True).strip(),
                  packages=packages)
    output.with_name(output.name + '-manifest.json').write_text(
        json.dumps(manifest, ensure_ascii=False, indent=2), encoding='utf-8')
    output.with_name(output.name + '-summary.json').write_text(
        json.dumps(result, ensure_ascii=False, indent=2), encoding='utf-8')
    print(json.dumps(result, ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
