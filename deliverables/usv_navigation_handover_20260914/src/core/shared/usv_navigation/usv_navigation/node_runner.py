"""ROS node lifecycle shared by independently owned business modules."""
import signal

import rclpy
from rclpy.signals import SignalHandlerOptions


def run_node(factory, args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    def terminate(signum, frame):
        raise KeyboardInterrupt
    old_term = signal.signal(signal.SIGTERM, terminate)
    node = None
    try:
        node = factory()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        signal.signal(signal.SIGTERM, old_term)
