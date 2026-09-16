"""Real ROS parameter declaration accepts the new planner without live sensors."""
import pytest

rclpy = pytest.importorskip('rclpy')
pytest.importorskip('usv_navigation_interfaces')

from usv_decision.decision import DecisionNode


def test_decision_node_initializes_with_hybrid_and_declares_its_parameters():
    rclpy.init(args=['--ros-args', '-r', '__ns:=/hybrid_parameter_test',
                    '-p', 'planner_name:=hybrid_astar',
                    '-p', 'path_planner.minimum_turning_radius:=6.0',
                    '-p', 'path_planner.angle_bins:=96'])
    node = None
    try:
        node = DecisionNode()
        assert node.config['planner_name'] == 'hybrid_astar'
        assert node.config['path_planner.minimum_turning_radius'] == 6.
        assert node.config['path_planner.angle_bins'] == 96
        assert node.get_parameter('path_planner.collision_step').value == .2
        assert node.get_parameter('navigation.footprint_length').value == 2.2
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()
