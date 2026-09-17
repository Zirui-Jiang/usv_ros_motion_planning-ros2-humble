import ast
import configparser
import difflib
import io
import json
from pathlib import Path
import re
import tokenize
import xml.etree.ElementTree as ET
import yaml

root = Path.cwd()
base = root / 'deliverables/_chinese_comments_20260914'
entries = json.loads((base/'inventory.json').read_text(encoding='utf-8'))
counts = {'files': 0, 'added_comment_lines': 0, 'python_ast_and_tokens': 0, 'xml': 0, 'yaml_rviz': 0, 'cfg': 0}
for item in entries:
    name = item['path']
    if not name.startswith(('src/core/mission_module/', 'src/core/perception_module/')):
        continue
    before, after = (base/'before'/name).read_bytes(), (root/name).read_bytes()
    original, changed = before.splitlines(keepends=True), after.splitlines(keepends=True)
    additions = []
    rebuilt = []
    for op, a, b, c, d in difflib.SequenceMatcher(a=original, b=changed, autojunk=False).get_opcodes():
        assert op in ('equal', 'insert'), (name, op)
        if op == 'equal':
            rebuilt.extend(changed[c:d])
        else:
            assert all(line.lstrip().startswith((b'#', b'//', b'<!--')) for line in changed[c:d]), name
            assert all(re.search('[\u4e00-\u9fff]', line.decode('utf-8')) for line in changed[c:d]), name
            additions.extend(changed[c:d])
    assert b''.join(rebuilt) == before
    assert additions, name
    counts['files'] += 1
    counts['added_comment_lines'] += len(additions)
    if name.endswith('.py') or '/scripts/perception_' in name:
        assert ast.dump(ast.parse(before)) == ast.dump(ast.parse(after)), name
        def tokens(raw):
            return [(t.type, t.string) for t in tokenize.tokenize(io.BytesIO(raw).readline)
                    if t.type not in (tokenize.COMMENT, tokenize.NL, tokenize.ENCODING)]
        assert tokens(before) == tokens(after), name
        counts['python_ast_and_tokens'] += 1
    elif name.endswith('.xml'):
        assert ET.tostring(ET.fromstring(before)) == ET.tostring(ET.fromstring(after)), name
        counts['xml'] += 1
    elif name.endswith(('.yaml', '.rviz')):
        assert yaml.safe_load(before) == yaml.safe_load(after), name
        counts['yaml_rviz'] += 1
    elif name.endswith('.cfg'):
        left, right = configparser.ConfigParser(), configparser.ConfigParser()
        left.read_string(before.decode('utf-8'))
        right.read_string(after.decode('utf-8'))
        assert {key: dict(value) for key, value in left.items()} == {key: dict(value) for key, value in right.items()}, name
        counts['cfg'] += 1
print(json.dumps(counts, ensure_ascii=False))
