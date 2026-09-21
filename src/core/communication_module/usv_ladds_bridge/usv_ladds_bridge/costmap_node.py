"""ROS 2 node that forwards simulation costmap snapshots to the 0141 bridge topic."""

from __future__ import annotations

import rclpy
from nav2_msgs.msg import Costmap
from rclpy.node import Node
from std_msgs.msg import String

from .costmap_0141 import (
    DEFAULT_COST_THRESHOLD,
    DEFAULT_MAX_CELLS,
    costmap_to_0141_json,
)


# 可选：把 costmap 编成测试用 0141 JSON，发给 /ladds/obstacle_in。
class CostmapAdapterNode(Node):
    """Convert ``nav2_msgs/Costmap`` messages into test 0141 JSON messages."""

    def __init__(self) -> None:
        super().__init__("usv_ladds_costmap")
        self.declare_parameter("input_topic", "/otter/costmap_raw")
        self.declare_parameter("output_topic", "/ladds/obstacle_in")
        self.declare_parameter("cost_threshold", DEFAULT_COST_THRESHOLD)
        self.declare_parameter("max_cells", DEFAULT_MAX_CELLS)
        self.declare_parameter("include_unknown", False)
        self.declare_parameter("queue_depth", 10)
        self.declare_parameter("use_sim_time", False)

        input_topic = str(self.get_parameter("input_topic").value)
        output_topic = str(self.get_parameter("output_topic").value)
        cost_threshold = int(self.get_parameter("cost_threshold").value)
        max_cells = int(self.get_parameter("max_cells").value)
        include_unknown = bool(self.get_parameter("include_unknown").value)
        queue_depth = int(self.get_parameter("queue_depth").value)
        if not input_topic.strip() or not output_topic.strip():
            raise ValueError("input_topic and output_topic must not be empty")
        if queue_depth <= 0:
            raise ValueError("queue_depth must be greater than zero")
        if not 0 <= cost_threshold <= 255:
            raise ValueError("cost_threshold must be between 0 and 255")
        if max_cells <= 0:
            raise ValueError("max_cells must be greater than zero")

        self._cost_threshold = cost_threshold
        self._max_cells = max_cells
        self._include_unknown = include_unknown
        self._sequence = 1
        self._publisher = self.create_publisher(String, output_topic, queue_depth)
        self._subscription = self.create_subscription(
            Costmap,
            input_topic,
            self._on_costmap,
            queue_depth,
        )
        self.get_logger().info(
            f"costmap 0141 adapter: {input_topic} -> {output_topic} "
            f"(threshold={cost_threshold}, max_cells={max_cells})"
        )

    def _on_costmap(self, message: Costmap) -> None:
        try:
            text = costmap_to_0141_json(
                message,
                sequence=self._sequence,
                cost_threshold=self._cost_threshold,
                max_cells=self._max_cells,
                include_unknown=self._include_unknown,
            )
        except (TypeError, ValueError, OverflowError) as exc:
            self.get_logger().error(f"failed to serialize costmap for 0141: {exc}")
            return

        output = String()
        output.data = text
        self._publisher.publish(output)
        self._sequence = (self._sequence + 1) % 65535


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = CostmapAdapterNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
