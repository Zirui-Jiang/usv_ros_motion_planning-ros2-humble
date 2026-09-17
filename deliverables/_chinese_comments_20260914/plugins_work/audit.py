from pathlib import Path
import ast
import difflib
import io
import json
import re
import tokenize
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[3]
BASE = ROOT / 'deliverables/_chinese_comments_20260914/before'
manifest = json.loads((BASE.parent / 'inventory.json').read_text(encoding='utf-8'))
cpp = re.compile(r'R"([^ ()\\\t\r\n]{0,16})\([\s\S]*?\)\1"|"(?:\\[\s\S]|[^"\\])*"|\'(?:\\[\s\S]|[^\'\\])*\'|//[^\r\n]*|/\*[\s\S]*?\*/|[A-Za-z_][A-Za-z_0-9]*|\d+(?:\.\d*)?|[^\s]', re.M)
counts = {}
added_total = 0
for entry in manifest:
    p = entry['path']
    if not p.startswith('src/plugins/'):
        continue
    before = (BASE / p).read_bytes()
    after = (ROOT / p).read_bytes()
    a = before.decode('utf-8').splitlines(keepends=True)
    b = after.decode('utf-8').splitlines(keepends=True)
    added = []
    for tag, i1, i2, j1, j2 in difflib.SequenceMatcher(None, a, b, autojunk=False).get_opcodes():
        assert tag in ('equal', 'insert'), (p, tag)
        if tag == 'insert':
            added.extend(range(j1, j2))
    assert added, p
    assert b''.join(s.encode('utf-8') for i, s in enumerate(b) if i not in added) == before, p
    assert all(re.search('[\u4e00-\u9fff]', b[i]) for i in added), p
    suffix = Path(p).suffix
    kind = 'cpp' if suffix in ('.cc', '.hh', '.cpp', '.h') else 'python' if suffix == '.py' else 'xml' if suffix in ('.xml', '.xacro', '.world') else 'cmake'
    counts[kind] = counts.get(kind, 0) + 1
    if kind == 'python':
        ts = list(tokenize.tokenize(io.BytesIO(after).readline))
        comment_lines = {t.start[0] - 1 for t in ts if t.type == tokenize.COMMENT}
        assert set(added) <= comment_lines, p
        assert ast.dump(ast.parse(before)) == ast.dump(ast.parse(after)), p
        def lexical(data):
            return [(t.type, t.string) for t in tokenize.tokenize(io.BytesIO(data).readline) if t.type not in (tokenize.COMMENT, tokenize.NL, tokenize.ENCODING)]
        assert lexical(before) == lexical(after), p
    elif kind == 'cpp':
        source = after.decode('utf-8')
        matches = list(cpp.finditer(source))
        comment_offsets = {m.start() for m in matches if m.group().startswith('//')}
        offset = 0
        for i, line in enumerate(b):
            if i in added:
                assert offset + len(line) - len(line.lstrip()) in comment_offsets, (p, i)
                assert not b[i - 1].rstrip().endswith('\\'), (p, i)
            offset += len(line)
        def lexical(data):
            return [m.group() for m in cpp.finditer(data.decode('utf-8')) if not m.group().startswith(('//', '/*'))]
        assert lexical(before) == lexical(after), p
    elif kind == 'xml':
        assert all(re.fullmatch(r'\s*<!--[^\r\n]*-->\s*', b[i]) for i in added), p
        def canonical(data):
            def visit(e):
                return e.tag, e.attrib, (e.text or '').strip(), (e.tail or '').strip(), [visit(c) for c in e]
            return visit(ET.fromstring(data))
        assert canonical(before) == canonical(after), p
    else:
        assert all(b[i].lstrip().startswith('#') for i in added), p
    added_total += len(added)
print(json.dumps({'files': sum(counts.values()), 'kinds': counts, 'independent_comment_lines': added_total, 'original_bytes_recovered': True, 'lexical_ast_xml_checks': 'pass'}, ensure_ascii=False))
