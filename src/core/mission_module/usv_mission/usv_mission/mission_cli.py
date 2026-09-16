# 任务命令行客户端：本地校验 YAML 后执行 start、status 或 cancel，并输出机器可读结果。
"""ROS mission command line client; parsing and YAML validation work without ROS."""
import argparse
import json
import math
from pathlib import Path
import sys
import time
import yaml

from usv_mission.configuration import load_yaml


MISSION_DEFAULTS = {
    "heading_deg": 0.0, "speed": 0.5, "position_tolerance": 0.5,
    "watch_radius": 10.0, "orbit_radius": 10.0, "clockwise": True,
    "laps": 0, "duration": 0.0,
}
MISSION_REQUIRED = frozenset(("mode", "latitude", "longitude"))


# 检查必填键、类型和基本范围；与实际船舶模型有关的速度/转弯限制仍由服务端校验。
def mission_request_values(mapping):
    """Validate a task YAML before opening a ROS client.

    The server additionally checks model-specific speed, turning feasibility,
    localization and geographic availability. A rejected replacement is atomic.
    """
    if not isinstance(mapping, dict):
        raise ValueError("Mission must be a YAML mapping")
    unknown = set(mapping) - (set(MISSION_DEFAULTS) | MISSION_REQUIRED)
    missing = MISSION_REQUIRED - set(mapping)
    if unknown:
        raise ValueError(f"Unknown mission fields: {sorted(unknown, key=str)}")
    if missing:
        raise ValueError(f"Missing mission fields: {sorted(missing)}")
    result = dict(MISSION_DEFAULTS, **mapping)
    if result["mode"] not in ("standby", "orbit", "station_keep"):
        raise ValueError("mode must be standby, orbit or station_keep")
    for name in ("latitude", "longitude", "heading_deg", "speed", "position_tolerance",
                 "watch_radius", "orbit_radius", "duration"):
        value = result[name]
        if type(value) not in (float, int) or not math.isfinite(value):
            raise ValueError(f"{name} must be a finite number")
        result[name] = float(value)
    if not -90 <= result["latitude"] <= 90 or not -180 <= result["longitude"] <= 180:
        raise ValueError("latitude/longitude outside WGS84 degree ranges")
    if any(result[name] <= 0 for name in ("speed", "position_tolerance", "watch_radius", "orbit_radius")):
        raise ValueError("speed, position_tolerance and radii must be positive")
    if result["duration"] < 0:
        raise ValueError("duration must be nonnegative seconds")
    if type(result["clockwise"]) is not bool:
        raise ValueError("clockwise must be a boolean")
    if type(result["laps"]) is not int or not 0 <= result["laps"] <= 4294967295:
        raise ValueError("laps must be a uint32 integer")
    if result["mode"] == "standby" and result["position_tolerance"] >= result["watch_radius"]:
        raise ValueError("standby position_tolerance must be smaller than watch_radius")
    return result


# 读取拒绝重复键的任务 YAML，再补充接口默认值。
def load_mission(path):
    return mission_request_values(load_yaml(path))


# 命令行命名空间与响应超时放在子命令前；start 接收任务 YAML 路径。
def make_parser():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--namespace", default="otter", help="vessel ROS namespace (default: otter)")
    parser.add_argument("--timeout", type=float, default=5.0, help="wall-time response limit in seconds")
    commands = parser.add_subparsers(dest="command", required=True)
    commands.add_parser("start", help="start or atomically replace a mission").add_argument("yaml", type=Path)
    commands.add_parser("status", help="print the next navigation status as JSON")
    commands.add_parser("cancel", help="cancel navigation or mission and request zero thrust")
    return parser


# 先完成无需 ROS 的参数检查，再按统一墙钟截止时刻等待服务或状态消息。
def main(argv=None):
    parser = make_parser()
    args = parser.parse_args(argv)
    try:
        if not math.isfinite(args.timeout) or args.timeout <= 0:
            raise ValueError("timeout must be finite and positive")
        if not args.namespace.strip("/"):
            raise ValueError("namespace must be nonempty")
        values = load_mission(args.yaml) if args.command == "start" else None
    except (ValueError, OSError, yaml.YAMLError) as error:
        parser.error(str(error))

    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String
    from std_srvs.srv import Trigger
    from usv_mission_interfaces.srv import SetMission

    rclpy.init(args=[])
    node = None
    try:
        node = Node("mission_cli", namespace="/" + args.namespace.strip("/"))
        if args.command == "status":
            received = []
            node.create_subscription(String, "navigation_status", lambda message: received.append(message.data), 10)
            deadline = time.monotonic() + args.timeout
            while rclpy.ok() and not received and time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=min(0.1, max(0.0, deadline - time.monotonic())))
            if not received:
                raise TimeoutError("No navigation_status received before timeout")
            print(json.dumps(json.loads(received[0]), ensure_ascii=False, indent=2))
            return 0

        service_type = SetMission if args.command == "start" else Trigger
        service_name = "set_mission" if args.command == "start" else "cancel_navigation"
        client = node.create_client(service_type, service_name)
        deadline = time.monotonic() + args.timeout
        if not client.wait_for_service(timeout_sec=args.timeout):
            raise TimeoutError(f"Service {service_name} unavailable")
        request = service_type.Request()
        for name, value in (values or {}).items():
            setattr(request, name, value)
        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future, timeout_sec=max(0.0, deadline - time.monotonic()))
        if not future.done():
            future.cancel()
            raise TimeoutError(f"Service {service_name} response timed out; inspect status before retrying")
        response = future.result()
        if args.command == "start":
            output = {"accepted": response.accepted, "mission_id": response.mission_id, "reason": response.reason}
            success = response.accepted
        else:
            output = {"success": response.success, "message": response.message}
            success = response.success
        print(json.dumps(output, ensure_ascii=False))
        return 0 if success else 2
    except (ValueError, RuntimeError, TimeoutError) as error:
        print(json.dumps({"error": str(error)}, ensure_ascii=False), file=sys.stderr)
        return 2
    except KeyboardInterrupt:
        return 130
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
