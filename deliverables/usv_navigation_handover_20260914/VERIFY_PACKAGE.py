"""Read-only SHA-256 verification; run before modifying the handover snapshot."""
import hashlib
import json
from pathlib import Path, PurePosixPath
import sys


def main():
    root = Path(__file__).resolve().parent
    manifest = json.loads((root / 'PACKAGE_MANIFEST.json').read_text(encoding='utf-8'))
    failed = []
    for row in manifest['files']:
        relative = PurePosixPath(row['path'])
        if relative.is_absolute() or '..' in relative.parts:
            failed.append(row['path'] + ': invalid path')
            continue
        path = root.joinpath(*relative.parts)
        if not path.resolve().is_relative_to(root) or not path.is_file():
            failed.append(row['path'] + ': missing or outside package')
            continue
        data = path.read_bytes()
        if len(data) != row['bytes'] or hashlib.sha256(data).hexdigest() != row['sha256']:
            failed.append(row['path'] + ': content mismatch')
    for message in failed:
        print(message)
    print(f"Verified {len(manifest['files'])} files; failures: {len(failed)}")
    return 1 if failed else 0


if __name__ == '__main__':
    sys.exit(main())
