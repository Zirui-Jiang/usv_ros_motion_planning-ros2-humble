"""Verify insertion-only comments against the saved pre-edit worktree."""
import ast
from collections import Counter
import configparser
import difflib
import hashlib
import io
import json
from pathlib import Path
import re
import sys
import tokenize
import xml.etree.ElementTree as ET

import pygments
from pygments.lexers import CppLexer, BashLexer, CMakeLexer
from pygments.token import Comment
import yaml

ROOT=Path(__file__).resolve().parents[2]
WORK=Path(__file__).resolve().parent
CPP={'.cc','.cpp','.c','.hh','.h','.hpp','.hxx','.cxx','.vert','.frag','.material'}
XML={'.xml','.xacro','.urdf','.sdf','.world','.config','.erb'}


def tree_signature(data):
    def signature(node):
        return (node.tag,sorted(node.attrib.items()),(node.text or '').strip(),
                (node.tail or '').strip(),[signature(child) for child in node])
    return signature(ET.fromstring(data))


def token_signature(data,lexer):
    tokens=[]
    for kind,value in pygments.lex(data.decode('utf-8'),lexer):
        if kind in Comment and kind not in Comment.Preproc:
            continue
        if value.isspace():continue
        tokens.append((str(kind),value))
    return tokens


def python_tokens(data):
    ignored={tokenize.COMMENT,tokenize.NL,tokenize.ENCODING,tokenize.ENDMARKER}
    return [(t.type,t.string) for t in tokenize.tokenize(io.BytesIO(data).readline) if t.type not in ignored]


def verify(row):
    path=ROOT/row['path']
    before=(WORK/'before'/row['path']).read_bytes()
    after=path.read_bytes()
    assert hashlib.sha256(before).hexdigest()==row['sha256'],'Baseline was altered'
    old=before.splitlines(keepends=True);new=after.splitlines(keepends=True)
    matcher=difflib.SequenceMatcher(a=old,b=new,autojunk=False)
    additions=[]
    for op,a0,a1,b0,b1 in matcher.get_opcodes():
        if op=='equal':continue
        assert op=='insert',f'Original lines changed: {op} at original line {a0+1}'
        for line in new[b0:b1]:
            stripped=line.decode('utf-8').strip()
            assert any('\u4e00'<=c<='\u9fff' for c in stripped),'Inserted line has no Chinese explanation'
            if path.suffix in XML:
                assert stripped.startswith('<!--') and stripped.endswith('-->'),'Not a standalone XML comment'
            elif path.suffix in CPP:
                assert stripped.startswith('//'),'Not a standalone C++ comment'
                assert not (a0 and old[a0-1].rstrip(b'\r\n').endswith(b'\\')),'Comment inserted into continued C++ line'
            else:
                assert stripped.startswith('#'),'Not a standalone hash comment'
            additions.append(stripped)
    assert additions,'Target has no newly added Chinese comments'
    kind='line_restore'
    if path.suffix=='.py' or (not path.suffix and before.startswith(b'#!/usr/bin/env python')):
        assert ast.dump(ast.parse(before),include_attributes=False)==ast.dump(ast.parse(after),include_attributes=False),'Python AST changed'
        assert python_tokens(before)==python_tokens(after),'Python noncomment tokens changed'
        kind='python_ast_and_tokens'
    elif path.suffix in CPP:
        assert token_signature(before,CppLexer())==token_signature(after,CppLexer()),'C++ tokens changed'
        kind='cpp_tokens'
    elif path.suffix=='.erb':
        before_text=before.decode('utf-8');after_text=after.decode('utf-8')
        assert re.findall(r'<%.*?%>',before_text,re.S)==re.findall(r'<%.*?%>',after_text,re.S),'Ruby template code changed'
        without_erb=lambda text: re.sub(r'<%.*?%>','',text,flags=re.S).encode('utf-8')
        assert tree_signature(without_erb(before_text))==tree_signature(without_erb(after_text)),'Template XML changed'
        kind='erb_code_and_xml'
    elif path.suffix in XML:
        assert tree_signature(before)==tree_signature(after),'XML structure changed'
        kind='xml_tree'
    elif path.suffix in {'.yaml','.yml','.rviz'}:
        assert list(yaml.safe_load_all(before))==list(yaml.safe_load_all(after)),'YAML values changed'
        kind='yaml_values'
    elif path.suffix in {'.cfg','.ini'}:
        def values(data):
            parser=configparser.ConfigParser(interpolation=None)
            parser.read_string(data.decode('utf-8'))
            return {section:dict(parser[section]) for section in parser.sections()}
        assert values(before)==values(after),'INI values changed'
        kind='ini_values'
    elif path.name=='CMakeLists.txt' or path.suffix=='.cmake':
        assert token_signature(before,CMakeLexer())==token_signature(after,CMakeLexer()),'CMake tokens changed'
        kind='cmake_tokens'
    elif path.suffix in {'.bash','.sh'} or path.name=='gazebo.sh.in':
        assert token_signature(before,BashLexer())==token_signature(after,BashLexer()),'Shell tokens changed'
        kind='bash_tokens'
    return {'path':row['path'],'added_comment_lines':len(additions),'check':kind,
            'before_sha256':row['sha256'],'after_sha256':hashlib.sha256(after).hexdigest()}


def main():
    rows=json.loads((WORK/'inventory.json').read_text(encoding='utf-8'))
    passed=[];failed=[]
    for row in rows:
        try:passed.append(verify(row))
        except Exception as exc:failed.append({'path':row['path'],'error':str(exc)})
    report={'files':len(rows),'passed':len(passed),'failed':failed,
            'added_comment_lines':sum(r['added_comment_lines'] for r in passed),
            'checks':dict(Counter(r['check'] for r in passed)), 'details':passed}
    (WORK/'audit.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    print(json.dumps({k:v for k,v in report.items() if k!='details'},ensure_ascii=False,indent=2))
    return bool(failed)


if __name__=='__main__':sys.exit(main())
