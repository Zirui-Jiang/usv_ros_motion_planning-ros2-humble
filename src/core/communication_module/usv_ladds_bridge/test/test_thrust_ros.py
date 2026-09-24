"""usv_ladds_tx 转发航控推力到 0054：套 head、断流由看门狗发零、未登记主题启动即失败。"""

import json
import time
import uuid

import pytest

rclpy = pytest.importorskip("rclpy")

from rclpy.executors import SingleThreadedExecutor  # noqa: E402

from usv_ladds_bridge.config import BridgeConfig, HeaderCache  # noqa: E402
from usv_ladds_bridge.outbound_node import LaDDSOutboundNode  # noqa: E402
from usv_ladds_bridge.thrust import zero_thrust_content  # noqa: E402


class FakeClient:
    """Transport boundary fake that records topic and payload of each send."""

    def __init__(self, send_topics=("0052", "0051", "0053", "0054")):
        self.send_topics = send_topics
        self.sent = []

    def send(self, payload, *, topic=None):
        self.sent.append((topic, payload))


def thrust_packets(client):
    return [json.loads(payload) for topic, payload in list(client.sent) if topic == "0054"]


@pytest.fixture
def ros():
    rclpy.init()
    try:
        yield
    finally:
        rclpy.shutdown()


def spin(executor, seconds):
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        executor.spin_once(timeout_sec=0.01)


def test_commands_are_forwarded_and_silence_falls_back_to_zero(ros):
    from usv_navigation_interfaces.msg import ThrusterCommand

    topic = f"/thrust_{uuid.uuid4().hex[:8]}/vector_cmd"
    client = FakeClient()
    config = BridgeConfig.from_mapping({
        "send_topic": "0052", "thrust_send_topic": "0054",
        "thrust_input_topic": topic, "thrust_timeout_sec": 0.3,
    })
    node = LaDDSOutboundNode(config, client, HeaderCache())
    driver = rclpy.create_node("thrust_driver_" + uuid.uuid4().hex[:8])
    publisher = driver.create_publisher(ThrusterCommand, topic, 1)
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    executor.add_node(driver)
    try:
        # 尚未收到航控命令：看门狗发带 head 的零推力。
        spin(executor, 0.3)
        first = thrust_packets(client)
        assert first and first[0]["content"] == zero_thrust_content()
        assert set(first[0]["head"]) == {
            "packageSeq", "packageType", "recvPlatform", "sendPlatform", "time"}

        command = ThrusterCommand(mode=0, left_rpm=283.4, right_rpm=301.2)
        end = time.monotonic() + 3.0
        while (not any(p["content"]["left_rpm"] == 283.4 for p in thrust_packets(client))
               and time.monotonic() < end):
            publisher.publish(command)
            spin(executor, 0.05)
        forwarded = [p for p in thrust_packets(client) if p["content"]["left_rpm"] == 283.4]
        assert forwarded and forwarded[-1]["content"]["right_rpm"] == 301.2

        # 停止发布：超过 0.3 s 后只剩看门狗零推力，head 序号持续递增。
        spin(executor, 0.1)
        client.sent.clear()
        spin(executor, 0.8)
        packets = thrust_packets(client)
        assert len(packets) >= 3
        assert all(p["content"] == zero_thrust_content() for p in packets)
        sequence = [p["head"]["packageSeq"] for p in packets]
        assert sequence == sorted(sequence) and len(set(sequence)) == len(sequence)
    finally:
        executor.shutdown()
        node.destroy_node()
        driver.destroy_node()


# 中继没有登记 0054 时启动即失败，而不是运行中每 0.1 s 报一次发送错误。
def test_unregistered_thrust_topic_fails_at_startup(ros):
    config = BridgeConfig.from_mapping({"send_topic": "0052", "thrust_send_topic": "0054"})
    with pytest.raises(ValueError, match="ThrustSendTopic"):
        LaDDSOutboundNode(config, FakeClient(("0052", "0051", "0053")), HeaderCache())


def test_disabled_thrust_topic_sends_nothing(ros):
    client = FakeClient(("0052",))
    node = LaDDSOutboundNode(BridgeConfig.from_mapping({"send_topic": "0052"}), client, HeaderCache())
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    try:
        spin(executor, 0.3)
        assert not thrust_packets(client)
    finally:
        executor.shutdown()
        node.destroy_node()
