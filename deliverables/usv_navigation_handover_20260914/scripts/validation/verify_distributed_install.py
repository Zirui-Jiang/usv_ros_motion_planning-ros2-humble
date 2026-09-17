#!/usr/bin/env python3
"""Audit a sourced isolated install without importing from the source tree.

Source /opt/ros/humble/setup.bash and the selected install/setup.bash in a fresh
shell, then run this script with absolute --install and --output paths. Imports,
message serialization, and launch parameter construction are real checks; no
ROS nodes, planning workers, launch processes, or Gazebo instances are started.
The report is installation evidence, not evidence of a working ROS graph.
"""
import argparse
import ast
from datetime import datetime, timezone
import importlib
import importlib.metadata
import json
import os
from pathlib import Path
import pkgutil
import sys
import xml.etree.ElementTree as ET


NAVIGATION_MODULES = (
    "usv_navigation", "usv_navigation.configuration", "usv_navigation.heartbeat",
    "usv_navigation.node_runner", "usv_navigation.launch_support",
    "usv_navigation.protocol", "usv_navigation.ros_support", "usv_navigation.state_rules",
    "usv_decision", "usv_decision.decision", "usv_decision.runtime",
    "usv_control", "usv_control.control",
    "usv_perception", "usv_perception.perception_monitor", "usv_perception.mapping_node",
)
COMPATIBILITY_MODULES = (
    "sim_env.configuration", "sim_env.runtime",
    "sim_env.simulation", "sim_env.launch_support",
)
PROJECT_PACKAGES = (
    "usv_navigation", "usv_navigation_interfaces", "sim_env", "common",
    "controller", "path_planner", "path_processor", "trajectory_planner", "perception",
    "usv_decision", "usv_control", "usv_perception",
)
REMOVED_PACKAGES = ("mmwave_perception", "usv_radar_costmap", "usv_radar_interfaces")
ENTRYPOINTS = {
    "decision_node": ("usv_decision", "usv_decision.decision:main"),
    "control_node": ("usv_control", "usv_control.control:main"),
    "perception_monitor": ("usv_perception", "usv_perception.perception_monitor:main"),
    "perception_mapping_node": ("usv_perception", "usv_perception.mapping_node:main"),
}
MESSAGES = ("TrajectoryPoint", "NavigationReference", "ModuleHeartbeat", "ControlReport")
LAUNCHES = ("main", "simulation_support", "perception", "decision", "control")
CONFIGS = {
    "dynamics_config": "robots/otter/dynamics_otter.yaml",
    "controller_config": "robots/otter/controller_params_otter.yaml",
    "trajectory_config": "trajectory_planner/trajectory_planner_params.yaml",
    "planner_config": "path_planner/common_planner_params.yaml",
}
PERCEPTION_CONFIGS = {
    "mapping_config": "costmap/mapping.yaml",
    "costmap_config": "robots/otter/global_costmap_params_otter.yaml",
}
BUSINESS_PACKAGES = ("usv_decision", "usv_control", "usv_perception")


def within(path, parent):
    try:
        Path(path).resolve().relative_to(Path(parent).resolve())
        return True
    except ValueError:
        return False


def removed_package_discovery(package):
    """Reject removed packages discoverable through any sourced ament overlay."""
    from ament_index_python.packages import PackageNotFoundError, get_package_prefix
    try:
        prefix = get_package_prefix(package)
    except PackageNotFoundError:
        return True, {"package": package, "discoverable": False}
    return False, {"package": package, "discoverable": True,
                   "prefix": str(Path(prefix).resolve())}


def installed_dependencies(install, package):
    """Follow installed manifests; business/shared packages cannot reach sim_env."""
    pending, seen, graph = [package], set(), {}
    while pending:
        current = pending.pop()
        if current in seen:
            continue
        seen.add(current)
        manifest = Path(install) / current / "share" / current / "package.xml"
        if manifest.is_file():
            document = ET.parse(manifest).getroot()
            dependencies = {element.text for element in document
                            if element.tag.endswith("depend") and element.text}
            graph[current] = sorted(dependencies)
            pending.extend(dependencies)
    forbidden = {"sim_env", *BUSINESS_PACKAGES} - {package}
    return not forbidden & seen, {"package": package, "reachable": sorted(seen),
                                   "forbidden_reachable": sorted(forbidden & seen), "graph": graph}


def wrapper_calls_entrypoint(text, target):
    """The CMake Python wrapper must import and invoke its declared callable."""
    module, symbol = target.split(":")
    expected = f"from {module} import {symbol}\nif __name__ == '__main__':\n    {symbol}()\n"
    return ast.dump(ast.parse(text), include_attributes=False) == ast.dump(
        ast.parse(expected), include_attributes=False)


def launch_nodes(path):
    """Evaluate only declarations and pure setup; never execute a Node action."""
    from launch import LaunchContext
    from launch.actions import DeclareLaunchArgument, OpaqueFunction
    from launch.launch_description_sources import PythonLaunchDescriptionSource
    from launch.utilities import normalize_to_list_of_substitutions, perform_substitutions
    from launch_ros.actions import Node

    context = LaunchContext()
    description = PythonLaunchDescriptionSource(str(path)).get_launch_description(context)
    nodes = []

    def visit(entities):
        for action in entities:
            if isinstance(action, DeclareLaunchArgument):
                action.execute(context)
            elif isinstance(action, OpaqueFunction):
                visit(action.execute(context) or [])
            elif isinstance(action, Node):
                nodes.append(action)

    visit(description.entities)
    return context, nodes


def launch_parameters(path):
    from launch.utilities import normalize_to_list_of_substitutions, perform_substitutions
    from launch_ros.utilities import evaluate_parameters
    from rclpy.parameter import Parameter, parameter_value_to_python

    context, nodes = launch_nodes(path)
    if len(nodes) != 1:
        raise ValueError(f"Expected one standalone navigation node, found {len(nodes)}")
    node = nodes[0]

    def value(raw):
        return perform_substitutions(context, normalize_to_list_of_substitutions(raw))

    parameters = {}
    for mapping in evaluate_parameters(context, node._Node__parameters):
        if not isinstance(mapping, dict):
            raise ValueError("Expected resolved parameter dictionaries")
        # launch represents arrays as tuples. Pass through ROS ParameterValue
        # so the snapshot has the same Python types a receiving node declares.
        parameters.update({name: parameter_value_to_python(
            Parameter(name, value=value).get_parameter_value()) for name, value in mapping.items()})
    return {
        "package": value(node.node_package), "executable": value(node.node_executable),
        "namespace": value(node._Node__node_namespace), "parameters": parameters,
        "config_paths": {key: context.launch_configurations[key] for key in CONFIGS},
    }


def audit(install):
    install = Path(install).resolve()
    rows = []

    def record(name, passed, **evidence):
        rows.append({"check": name, "passed": bool(passed), **evidence})

    def check(name, action):
        try:
            passed, evidence = action()
            record(name, passed, **evidence)
        except Exception as error:
            record(name, False, error=f"{type(error).__name__}: {error}")

    record("target_install", install.is_dir() and (install / "setup.bash").is_file(),
           path=str(install), setup=str(install / "setup.bash"))
    for package in PROJECT_PACKAGES:
        def discovery(package=package):
            from ament_index_python.packages import get_package_prefix
            prefix = Path(get_package_prefix(package)).resolve()
            expected = install / package
            return prefix == expected, {"prefix": str(prefix), "expected_prefix": str(expected)}
        check(package + ":discovery", discovery)
    for package in REMOVED_PACKAGES:
        check(package + ":absent", lambda package=package: removed_package_discovery(package))
    for package in (*BUSINESS_PACKAGES, "usv_navigation"):
        check(package + ":independent_dependencies",
              lambda package=package: installed_dependencies(install, package))

    names = set(NAVIGATION_MODULES + COMPATIBILITY_MODULES)
    names.update(("usv_navigation_interfaces", "usv_navigation_interfaces.msg"))
    try:
        package = importlib.import_module("usv_navigation")
        names.update(item.name for item in pkgutil.walk_packages(package.__path__, package.__name__ + "."))
    except Exception:
        pass  # Every required import below records its own failure evidence.
    for name in sorted(names):
        def imported(name=name):
            module = importlib.import_module(name)
            origin = Path(module.__file__).resolve()
            return within(origin, install), {"origin": str(origin)}
        check("import:" + name, imported)

    for name, (package, target) in ENTRYPOINTS.items():
        executable = install / package / "lib" / package / name
        record("entrypoint:" + name,
               executable.is_file() and within(executable, install) and os.access(executable, os.X_OK),
               path=str(executable), executable=os.access(executable, os.X_OK))

        def registered(name=name, target=target, package=package, executable=executable):
            manifest = install / package / "share" / package / "package.xml"
            if ET.parse(manifest).getroot().findtext("export/build_type") == "ament_cmake":
                # ament_cmake_python installs explicit script wrappers, not a
                # setuptools console_scripts distribution. Check the wrapper
                # import and the callable at its real installed origin.
                module_name, symbol = target.split(":")
                expected_import = f"from {module_name} import {symbol}"
                text = executable.read_text(encoding="utf-8")
                module = importlib.import_module(module_name)
                valid = (wrapper_calls_entrypoint(text, target) and callable(getattr(module, symbol))
                         and within(module.__file__, install))
                return valid, {"wrapper": str(executable), "expected_import": expected_import,
                               "module_origin": str(Path(module.__file__).resolve())}
            distribution = importlib.metadata.distribution(package)
            origin = Path(distribution.locate_file("")).resolve()
            entries = [entry for entry in distribution.entry_points
                       if entry.group == "console_scripts" and entry.name == name]
            values = [entry.value for entry in entries]
            valid = within(origin, install) and values == [target]
            if valid:
                valid = callable(entries[0].load())
            return valid, {"distribution_origin": str(origin), "targets": values, "expected": target}
        check("entrypoint_registration:" + name, registered)

    old_entries = [install / "usv_navigation/lib/usv_navigation" / name for name in ENTRYPOINTS]
    record("shared:no_business_entrypoints", not any(path.exists() for path in old_entries),
           unexpected=[str(path) for path in old_entries if path.exists()])
    for name in ("costmap_support_node", "usv_tf_expander.py"):
        path = install / "usv_perception/lib/usv_perception" / name
        record("entrypoint:" + name, path.is_file() and os.access(path, os.X_OK), path=str(path))

    share = install / "sim_env" / "share" / "sim_env"
    launch_paths = {name: share / "launch" / (name + ".launch.py") for name in LAUNCHES}
    launch_paths.update({package + ":" + name: install / package / "share" / package / "launch" / (name + ".launch.py")
                         for package, name in (("usv_decision", "decision"), ("usv_control", "control"),
                                               ("usv_perception", "perception"))})
    for name, path in launch_paths.items():

        def launch_file(path=path):
            from launch import LaunchContext
            from launch.launch_description_sources import PythonLaunchDescriptionSource
            if not path.is_file() or not within(path, install):
                return False, {"path": str(path), "exists": path.is_file()}
            description = PythonLaunchDescriptionSource(str(path)).get_launch_description(LaunchContext())
            return bool(description.entities), {"path": str(path), "actions": len(description.entities)}
        check("launch:" + name, launch_file)

    navigation_share = install / "usv_navigation" / "share" / "usv_navigation"
    perception_share = install / "usv_perception" / "share" / "usv_perception"
    paths = {name: navigation_share / "config" / relative for name, relative in CONFIGS.items()}
    perception_paths = {name: perception_share / "config" / relative for name, relative in PERCEPTION_CONFIGS.items()}
    for name, path in {**paths, **perception_paths}.items():
        def project_yaml(path=path):
            from usv_navigation.configuration import load_yaml
            values = load_yaml(path)
            return within(path, install) and bool(values), {"path": str(path), "root_keys": sorted(values)}
        check("yaml:" + name, project_yaml)
    duplicate_configs = [install / package / "share" / package / "config" / relative
                         for package in ("sim_env", *BUSINESS_PACKAGES)
                         for relative in CONFIGS.values()]
    duplicate_configs.extend(share / "config" / relative for relative in PERCEPTION_CONFIGS.values())
    record("yaml:single_owner", not any(path.exists() for path in duplicate_configs),
           unexpected=[str(path) for path in duplicate_configs if path.exists()])

    for name in MESSAGES:
        def message_support(name=name):
            from rclpy.serialization import deserialize_message, serialize_message
            from rclpy.type_support import check_for_type_support
            messages = importlib.import_module("usv_navigation_interfaces.msg")
            message_type = getattr(messages, name)
            check_for_type_support(message_type)
            message = message_type()
            serialized = serialize_message(message)
            restored = deserialize_message(serialized, message_type)
            origin = Path(importlib.import_module(message_type.__module__).__file__).resolve()
            loaded = {
                key: str(Path(module.__file__).resolve()) for key, module in sys.modules.copy().items()
                if key.startswith("usv_navigation_interfaces.") and "rosidl_typesupport" in key
                and getattr(module, "__file__", None)
            }
            valid = (within(origin, install) and bool(loaded)
                     and all(within(path, install) for path in loaded.values())
                     and message_type.__class__._TYPE_SUPPORT is not None and restored == message)
            return valid, {"origin": str(origin), "type_support_libraries": loaded,
                           "serialized_bytes": len(serialized), "round_trip_equal": restored == message}
        check("type_support:" + name, message_support)

    def configuration():
        from common.models import USVModel
        from controller.controller_factory import create_controller
        from trajectory_planner.trajectory_planner_factory import create_trajectory_planner
        from sim_env import configuration as compatibility
        from usv_navigation import configuration as core
        from usv_navigation import ros_support
        from usv_control import control

        expected = core.load_project_parameters(*paths.values())
        if compatibility.load_project_parameters(*paths.values()) != expected:
            raise ValueError("Compatibility loader differs from the navigation loader")
        if ros_support.resolve_parameters is not core.resolve_parameters or control.resolve_parameters is not core.resolve_parameters:
            raise ValueError("Runtime nodes do not use the audited shared resolver")
        decision = launch_parameters(launch_paths["usv_decision:decision"])
        control_values = launch_parameters(launch_paths["usv_control:control"])
        if decision["package"] != "usv_decision" or decision["executable"] != "decision_node":
            raise ValueError("Decision launch has an unexpected node")
        if control_values["package"] != "usv_control" or control_values["executable"] != "control_node":
            raise ValueError("Control launch has an unexpected node")
        if decision["parameters"] != control_values["parameters"] or decision["namespace"] != control_values["namespace"]:
            raise ValueError("Decision/control launch values differ")
        for node in (decision, control_values):
            for name, path in paths.items():
                if Path(node["config_paths"][name]).resolve() != path:
                    raise ValueError(f"Launch default {name} does not use the target installed YAML")
        initial = dict(decision["parameters"])
        if initial.pop("use_sim_time", None) is not True:
            raise ValueError("Default launch must set use_sim_time=true")
        snapshot = core.resolve_parameters({**expected, "dynamics.ns": "otter"})
        mismatches = {key: {"launch": repr(initial.get(key)), "loader": repr(snapshot.get(key))}
                      for key in initial.keys() | snapshot.keys() if initial.get(key) != snapshot.get(key)}
        if mismatches:
            raise ValueError(f"Launch/loader values differ: {mismatches}")
        if core.resolve_parameters(initial) != initial:
            raise ValueError("Launch values are not canonical")
        model = USVModel.from_mapping(core.section(initial, "dynamics."))
        controller_name = initial[core.C + "controller_name"]
        controller_instance = create_controller(controller_name, model,
            core.section(initial, core.C + controller_name + "_controller."))
        guidance = create_trajectory_planner(initial[core.T + "planner_name"], model,
                                            core.section(initial, core.T + "los."))
        variants = {"lidar": {
            "effective_parameters": initial, "namespace": decision["namespace"],
            "controller_class": type(controller_instance).__name__,
            "guidance_class": type(guidance).__name__,
        }}
        return True, {"mode": "pure loader and installed launch action evaluation; no node startup",
                      "variants": variants}
    check("configuration:decision_control", configuration)

    def perception_launch():
        from launch.utilities import normalize_to_list_of_substitutions, perform_substitutions
        context, nodes = launch_nodes(launch_paths["usv_perception:perception"])
        def value(raw):
            return perform_substitutions(context, normalize_to_list_of_substitutions(raw))
        actual = [(value(node.node_package), value(node.node_executable)) for node in nodes]
        expected = [("usv_perception", name) for name in
                    ("perception_mapping_node", "costmap_support_node", "perception_monitor")]
        configured = {name: context.launch_configurations.get(name) for name in perception_paths}
        correct_paths = all(configured[name] and Path(configured[name]).resolve() == path
                            for name, path in perception_paths.items())
        return sorted(actual) == sorted(expected) and correct_paths, {
            "nodes": actual, "expected_nodes": expected, "config_paths": configured}
    check("configuration:perception", perception_launch)

    # Check all project dependencies actually loaded by the audit, not just the
    # named entry modules. Resolving symlinks deliberately rejects source imports.
    loaded = {}
    for name, module in sys.modules.copy().items():
        if name.split(".")[0] in PROJECT_PACKAGES and getattr(module, "__file__", None):
            loaded[name] = str(Path(module.__file__).resolve())
    contaminated = {name: path for name, path in loaded.items() if not within(path, install)}
    record("loaded_project_modules", bool(loaded) and not contaminated,
           origins=loaded, outside_target_install=contaminated)
    python_paths = [str(Path(path or os.getcwd()).resolve())
                    for path in os.environ.get("PYTHONPATH", "").split(os.pathsep) if path]
    source_paths = [path for path in python_paths if not within(path, install)
                    and any((Path(path) / name / "__init__.py").is_file() for name in PROJECT_PACKAGES)]
    record("no_source_pythonpath", not source_paths, entries=python_paths, project_paths_outside_install=source_paths)
    return {
        "classification": "distributed_navigation_installation_audit",
        "collected_utc": datetime.now(timezone.utc).isoformat(),
        "install": str(install), "python_executable": sys.executable,
        "python_version": sys.version, "environment": {name: os.environ.get(name, "") for name in
            ("AMENT_PREFIX_PATH", "PYTHONPATH", "LD_LIBRARY_PATH")},
        "checks": rows, "all_passed": bool(rows) and all(row["passed"] for row in rows),
    }


def absolute_path(value):
    path = Path(value).expanduser()
    if not path.is_absolute():
        raise argparse.ArgumentTypeError("Path must be absolute")
    return path.resolve()


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--install", required=True, type=absolute_path)
    parser.add_argument("--output", required=True, type=absolute_path)
    args = parser.parse_args(argv)
    if args.output.exists():
        parser.error(f"Refusing to overwrite an existing report: {args.output}")
    report = audit(args.install)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open("x", encoding="utf-8") as stream:
        json.dump(report, stream, ensure_ascii=False, indent=2)
        stream.write("\n")
    failures = [row["check"] for row in report["checks"] if not row["passed"]]
    print(json.dumps({"output": str(args.output), "all_passed": report["all_passed"],
                      "checks": len(report["checks"]), "failed_checks": failures}))
    return 0 if report["all_passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
