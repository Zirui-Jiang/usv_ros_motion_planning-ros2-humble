"""Read-only map/process observation in the acceptance run's ROS domain."""
import json
from pathlib import Path
import time

import rclpy
from nav_msgs.msg import OccupancyGrid


destination = Path(__file__).with_name("live-map-and-processes.json")
rclpy.init()
node = rclpy.create_node("startup_fix_map_observer")
maps = []
subscription = node.create_subscription(
    OccupancyGrid, "/otter/map", lambda msg: maps.append(dict(
        stamp=msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
        frame=msg.header.frame_id, width=msg.info.width, height=msg.info.height)), 1)
deadline = time.monotonic() + 4
while time.monotonic() < deadline:
    rclpy.spin_once(node, timeout_sec=0.1)
processes = []
for path in Path("/proc").glob("[0-9]*/cmdline"):
    try:
        command = path.read_bytes().decode().split("\0")[:-1]
        if any("/module-layout-20260914/install/" in part for part in command) and any(
                part.endswith(("/perception_mapping_node", "/perception_monitor", "/decision_node", "/control_node"))
                for part in command):
            processes.append(dict(pid=int(path.parent.name), command=command))
    except (OSError, UnicodeError):
        continue
result = dict(maps=maps, processes=processes, nodes=node.get_node_names_and_namespaces(),
              map_publishers=[dict(name=info.node_name, namespace=info.node_namespace)
                              for info in node.get_publishers_info_by_topic("/otter/map")])
node.destroy_node()
rclpy.shutdown()
with destination.open("x", encoding="utf-8") as stream:
    json.dump(result, stream, indent=2)
print(json.dumps(dict(map_samples=len(maps), processes=processes)))
raise SystemExit(0 if len({item["stamp"] for item in maps}) >= 3 else 1)
