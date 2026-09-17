"""A restarted perception publisher changes the module identity, a gap does not."""
import importlib
import importlib.util
import time
import pytest
rclpy = pytest.importorskip('rclpy')
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from std_msgs.msg import String


def test_perception_publisher_restart_rotates_identity():
    assert importlib.util.find_spec('usv_perception.perception_monitor') is not None, 'Perception monitor is missing'
    rclpy.init(args=['--ros-args', '-r', '__ns:=/perception_epoch',
                     '-p', "watched_topics:=['observations']"])
    monitor = importlib.import_module('usv_perception.perception_monitor').PerceptionMonitor()
    driver = Node('observations_driver')
    executor = SingleThreadedExecutor()
    executor.add_node(monitor)
    executor.add_node(driver)
    publisher = driver.create_publisher(String, 'observations', 1)
    def until(predicate):
        end = time.monotonic() + 4.
        while time.monotonic() < end and not predicate():
            executor.spin_once(timeout_sec=.05)
        assert predicate()
    try:
        until(lambda: monitor.ready)
        original = monitor.session
        # No messages are needed for process identity; observation freshness
        # remains the input consumer's original responsibility.
        for _ in range(5):
            executor.spin_once(timeout_sec=.05)
        assert monitor.session == original
        driver.destroy_publisher(publisher)
        until(lambda: not monitor.ready)
        assert monitor.session == original
        publisher = driver.create_publisher(String, 'observations', 1)
        until(lambda: monitor.session != original)
        assert monitor.ready
    finally:
        executor.shutdown()
        driver.destroy_node()
        monitor.destroy_node()
        rclpy.shutdown()
