# 统一 ROS 节点初始化、spin 与退出清理；将 SIGTERM 转为正常退出路径，保证业务 destroy_node 得到调用。
"""ROS node lifecycle shared by independently owned business modules."""
import signal

import rclpy
from rclpy.signals import SignalHandlerOptions


# 关闭 rclpy 默认信号处理，自行保存并恢复 SIGTERM 处理器，finally 中清理节点和上下文。
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
