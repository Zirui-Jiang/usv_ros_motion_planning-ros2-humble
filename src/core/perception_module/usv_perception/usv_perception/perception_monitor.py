# 感知模块进程身份监控：观察输出发布端点，发布模块心跳以通知下游感知链是否重启。
"""Identify restarts in the existing multi-process perception module."""
import uuid
from rclpy.node import Node
from rclpy.clock import Clock, ClockType
from usv_navigation_interfaces.msg import ModuleHeartbeat
from usv_navigation.heartbeat import HEARTBEAT_PERIOD
from usv_navigation.node_runner import run_node


# 发布者存在性决定 ready，端点身份变化切换 session；观测内容新鲜度由消费者另行检查。
class PerceptionMonitor(Node):
    def __init__(self):
        super().__init__('usv_perception')
        self.declare_parameter('watched_topics', ['odom', 'costmap_raw', 'map'])
        self.topics = [self.resolve_topic_name(topic) for topic in self.get_parameter('watched_topics').value]
        if not self.topics:
            raise ValueError('The perception module must monitor its output publishers')
        self.session = uuid.uuid4().hex
        self.sequence = 0
        self.ready = False
        self.publisher_identities = {}
        self.retired = {topic: set() for topic in self.topics}
        self.heartbeat_pub = self.create_publisher(ModuleHeartbeat, 'navigation/heartbeat', 10)
        self.steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.create_timer(HEARTBEAT_PERIOD, self._poll, clock=self.steady_clock)

    # 短暂消失只撤销就绪状态；出现新的端点身份才生成新会话并重置序列号。
    def _poll(self):
        ready, restarted = True, False
        for topic in self.topics:
            current = {bytes(info.endpoint_gid) for info in self.get_publishers_info_by_topic(topic)}
            current -= self.retired[topic]
            ready = ready and bool(current)
            if not current:
                # A temporary disappearance pauses consumers but does not
                # erase the identity needed to recognize a returning process.
                continue
            previous = self.publisher_identities.get(topic)
            if previous is not None and current - previous:
                restarted = True
                self.retired[topic].update(previous - current)
            self.publisher_identities[topic] = current
        if restarted:
            self.session = uuid.uuid4().hex
            self.sequence = 0
            self.get_logger().info('Perception publisher restarted; a new navigation goal is required')
        self.ready = ready
        self.sequence += 1
        self.heartbeat_pub.publish(ModuleHeartbeat(
            module='perception', session=self.session, sequence=self.sequence, ready=ready,
            reason='' if ready else 'Waiting for perception output publishers'))


# 交给公共节点运行器处理感知监控节点的启动和退出。
def main(args=None):
    run_node(PerceptionMonitor, args)
