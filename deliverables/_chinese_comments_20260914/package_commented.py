"""Preserve the simple source ZIP's members, replacing only audited comments."""
from pathlib import Path
import hashlib
import json
import zipfile

ROOT=Path(__file__).resolve().parents[2]
WORK=Path(__file__).resolve().parent
source=ROOT/'deliverables/usv_code_20260914.zip'
target=ROOT/'deliverables/usv_code_zh_20260914.zip'
if target.exists():raise SystemExit('Refusing existing output ZIP.')
audit=json.loads((WORK/'audit.json').read_text(encoding='utf-8'))
assert audit['passed']==audit['files'] and not audit['failed']
entries={row['path']:row for row in audit['details']}
with zipfile.ZipFile(source) as old, zipfile.ZipFile(target,'x',compression=zipfile.ZIP_DEFLATED,compresslevel=6) as new:
    expected={}
    for info in old.infolist():
        relative=info.filename.removeprefix('usv_code/')
        before=old.read(info.filename)
        if relative in entries:
            record=entries[relative]
            assert hashlib.sha256(before).hexdigest()==record['before_sha256'],relative
            data=(ROOT/relative).read_bytes()
            assert hashlib.sha256(data).hexdigest()==record['after_sha256'],relative
        else:data=before
        new.writestr(info,data)
        expected[info.filename]=hashlib.sha256(data).hexdigest()
with zipfile.ZipFile(target) as result:
    assert result.testzip() is None
    assert set(result.namelist())==set(expected)
    assert len(result.namelist())==len(expected)
    for name,sha in expected.items():assert hashlib.sha256(result.read(name)).hexdigest()==sha,name
report={'archive':target.name,'bytes':target.stat().st_size,
        'sha256':hashlib.sha256(target.read_bytes()).hexdigest(),
        'files':len(expected),'annotated_files':len(entries),
        'chinese_comment_lines_added':audit['added_comment_lines'],
        'original_non_target_files_preserved':True,'all_zip_hashes_and_crc':'passed'}
(WORK/'package-check.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
print(json.dumps(report,ensure_ascii=False,indent=2))
