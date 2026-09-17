#!/usr/bin/env python3
"""Compare algorithms and explicitly relocated modules with a pre-move snapshot."""
import argparse
import ast
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET

ALGORITHMS = ('common', 'controller', 'path_planner', 'path_processor',
              'trajectory_planner', 'perception', 'usv_mission')
CONFIGS = (
    ('usv_navigation', 'robots/otter/dynamics_otter.yaml'),
    ('usv_navigation', 'robots/otter/controller_params_otter.yaml'),
    ('usv_navigation', 'trajectory_planner/trajectory_planner_params.yaml'),
    ('usv_navigation', 'path_planner/common_planner_params.yaml'),
    ('usv_perception', 'costmap/mapping.yaml'),
    ('usv_perception', 'robots/otter/global_costmap_params_otter.yaml'),
)
MIGRATED_MODULES = (
    ('usv_navigation', 'runtime.py', 'usv_decision', 'runtime.py'),
    ('usv_navigation', 'decision.py', 'usv_decision', 'decision.py'),
    ('usv_navigation', 'control.py', 'usv_control', 'control.py'),
    ('usv_navigation', 'perception_monitor.py', 'usv_perception', 'perception_monitor.py'),
    ('usv_navigation', 'state_rules.py', 'usv_navigation', 'state_rules.py'),
    ('perception', 'mapping_node.py', 'usv_perception', 'mapping_node.py'),
)
SPLIT_DEFINITIONS = {
    ('protocol.py', 'PeerSession'): ('heartbeat.py', 'PeerSession'),
    ('protocol.py', 'HEARTBEAT_PERIOD'): ('heartbeat.py', 'HEARTBEAT_PERIOD'),
    ('protocol.py', 'PEER_TIMEOUT'): ('heartbeat.py', 'PEER_TIMEOUT'),
    ('ros_support.py', 'run_node'): ('node_runner.py', 'run_node'),
}


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def packages(root):
    """Discover historical and grouped layouts, rejecting duplicate names."""
    result = {}
    for manifest in sorted((Path(root) / 'src').rglob('package.xml')):
        name = ET.parse(manifest).getroot().findtext('name')
        if not name or name in result:
            raise ValueError(f'Missing or duplicate package name: {manifest}: {name}')
        result[name] = manifest.parent
    return result


def definitions(path):
    """Retain complete definitions/constants, decorators, defaults and docstrings."""
    result = {}
    for node in ast.parse(path.read_text(encoding='utf-8-sig')).body:
        if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef, ast.ClassDef)):
            result[node.name] = ast.dump(node, include_attributes=False)
        elif isinstance(node, (ast.Assign, ast.AnnAssign)):
            targets = node.targets if isinstance(node, ast.Assign) else [node.target]
            for target in targets:
                if isinstance(target, ast.Name):
                    result[target.id] = ast.dump(node, include_attributes=False)
    return result


def configuration_ast(path):
    """No DEFAULTS or other configuration statements are exempted."""
    return ast.dump(ast.parse(path.read_text(encoding='utf-8-sig')), include_attributes=False)


def audit(baseline, source):
    before_packages, after_packages = packages(baseline), packages(source)
    checks = []

    def byte_check(kind, before, after):
        checks.append(dict(check=kind, baseline_file=str(before), source_file=str(after),
                           passed=before.is_file() and after.is_file() and sha(before) == sha(after),
                           baseline_sha256=sha(before) if before.is_file() else None,
                           source_sha256=sha(after) if after.is_file() else None))

    for name in ALGORITHMS:
        before, after = before_packages[name] / name, after_packages[name] / name
        # Explicitly checked as migrated mapping-node AST / mission launch tests.
        excluded = {'mapping_node.py'} if name == 'perception' else (
            {'launch_support.py'} if name == 'usv_mission' else set())
        old_files = {p.relative_to(before) for p in before.rglob('*.py')
                     if p.name not in excluded and '__pycache__' not in p.parts}
        new_files = {p.relative_to(after) for p in after.rglob('*.py')
                     if p.name not in excluded and '__pycache__' not in p.parts}
        checks.append(dict(check='algorithm_inventory', package=name,
                           passed=bool(old_files) and old_files == new_files,
                           missing=sorted(map(str, old_files - new_files)),
                           added=sorted(map(str, new_files - old_files))))
        for relative in sorted(old_files):
            byte_check('algorithm_source', before / relative, after / relative)

    for owner, relative in CONFIGS:
        before = before_packages.get(owner, Path()) / 'config' / relative
        if not before.is_file():
            before = before_packages['sim_env'] / 'config' / relative
        byte_check('configuration_yaml', before, after_packages[owner] / 'config' / relative)

    before_nav = before_packages['usv_navigation'] / 'usv_navigation'
    after_nav = after_packages['usv_navigation'] / 'usv_navigation'
    before_config, after_config = before_nav / 'configuration.py', after_nav / 'configuration.py'
    checks.append(dict(check='configuration_complete_ast',
                       passed=configuration_ast(before_config) == configuration_ast(after_config),
                       baseline_file=str(before_config), source_file=str(after_config),
                       baseline_sha256=sha(before_config), source_sha256=sha(after_config)))

    for old_package, old_module, new_package, new_module in MIGRATED_MODULES:
        before = before_packages[old_package] / old_package / old_module
        after = after_packages[new_package] / new_package / new_module
        old, new = definitions(before), definitions(after)
        checks.append(dict(check='migrated_definition_inventory', baseline_file=str(before),
                           source_file=str(after), passed=bool(old) and old.keys() == new.keys()))
        for name, expected in old.items():
            checks.append(dict(check='preserved_definition', name=name,
                               baseline_file=str(before), source_file=str(after),
                               passed=new.get(name) == expected))

    for module in ('protocol.py', 'ros_support.py'):
        before = before_nav / module
        own_names = set()
        for name, expected in definitions(before).items():
            destination, destination_name = SPLIT_DEFINITIONS.get((module, name), (module, name))
            after = after_nav / destination
            if destination == module:
                own_names.add(destination_name)
            checks.append(dict(check='preserved_definition', name=name,
                               baseline_file=str(before), source_file=str(after),
                               passed=definitions(after).get(destination_name) == expected))
        checks.append(dict(check='split_definition_inventory', source_file=str(after_nav / module),
                           passed=set(definitions(after_nav / module)) == own_names))
    for destination in ('heartbeat.py', 'node_runner.py'):
        expected = {new_name for target, new_name in SPLIT_DEFINITIONS.values() if target == destination}
        checks.append(dict(check='split_definition_inventory', source_file=str(after_nav / destination),
                           passed=set(definitions(after_nav / destination)) == expected))

    byte_check('moved_cpp_source', before_packages['sim_env'] / 'src/costmap_support_node.cpp',
               after_packages['usv_perception'] / 'src/costmap_support_node.cpp')
    byte_check('moved_tf_helper', before_packages['perception'] / 'scripts/tf_process/usv_tf_expander.py',
               after_packages['usv_perception'] / 'scripts/usv_tf_expander.py')
    return dict(passed=bool(checks) and all(item['passed'] for item in checks), checks=checks,
                scope='Byte-identical numerical algorithms and six YAMLs; complete configuration AST; '
                      'explicitly mapped node/worker/heartbeat/lifecycle definitions and constants. '
                      'Module-level import relocation, launch and metadata require separate tests; '
                      'this report alone does not prove ROS/Gazebo behavior.')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--baseline', type=Path, required=True)
    parser.add_argument('--source', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        parser.error('output already exists')
    try:
        result = audit(args.baseline, args.source)
    except (OSError, ValueError, KeyError, SyntaxError) as error:
        result = dict(passed=False, checks=[], failure=f'{type(error).__name__}: {error}')
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open('x', encoding='utf-8') as stream:
        json.dump(result, stream, indent=2, ensure_ascii=False)
    print(json.dumps(dict(passed=result['passed'], checks=len(result['checks']),
                          failed=[item for item in result['checks'] if not item['passed']],
                          failure=result.get('failure')), indent=2, ensure_ascii=False))
    return 0 if result['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
