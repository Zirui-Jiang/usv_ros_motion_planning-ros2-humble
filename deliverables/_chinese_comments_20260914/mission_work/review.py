import ast
import json
from pathlib import Path
root = Path.cwd()
items = json.loads((root/'deliverables/_chinese_comments_20260914/inventory.json').read_text(encoding='utf-8'))
for item in items:
    name = item['path']
    if not name.startswith(('src/core/mission_module/', 'src/core/perception_module/')) or not Path(name).name.startswith('test_'):
        continue
    source = (root/name).read_text(encoding='utf-8')
    print('\nFILE', name)
    tree = ast.parse(source)
    for node in tree.body:
        if not isinstance(node, (ast.FunctionDef, ast.ClassDef)):
            continue
        print(node.name)
        if isinstance(node, ast.FunctionDef):
            parts = []
            for child in ast.walk(node):
                if isinstance(child, (ast.Assert, ast.Raise)):
                    parts.append(ast.unparse(child))
            print(' | '.join(parts)[:1800])
