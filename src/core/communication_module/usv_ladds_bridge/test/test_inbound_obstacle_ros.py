"""usv_ladds_rx 收 0141：剥外壳转发业务对象，按 0411 同一原点发布整张替换的障碍栅格。"""

from dataclasses import dataclass
import json
import math
import time
import uuid

import pytest

rclpy = pytest.importorskip("rclpy")

from nav_msgs.msg import OccupancyGrid  # noqa: E402
from rclpy.executors import SingleThreadedExecutor  # noqa: E402
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy  # noqa: E402
from std_msgs.msg import String  # noqa: E402

from common.geodesy import GeoReference  # noqa: E402
from usv_ladds_bridge.inbound_node import LaDDSInboundNode  # noqa: E402


ORIGIN = GeoReference(30.5472, 114.3891, 0.0)


@dataclass(frozen=True)
class Packet:
    domain_id: int
    topic: str
    payload: bytes
    src: int = 1001
    dst: int = 1000


class FakeClient:
    """TCP 传输替身：测试把包放进 incoming，接收节点按轮询取走。"""

    version = "fake"
    dropped_messages = 0

    def __init__(self, **_settings):
        self.incoming = []

    def start(self):
        pass

    def receive(self):
        return self.incoming.pop(0) if self.incoming else None

    def close(self):
        pass


@pytest.fixture
def prefix():
    return "/rx_" + uuid.uuid4().hex[:8]


@pytest.fixture
def ros(prefix):
    # Unique topics keep this test away from any running bridge in the same ROS domain.
    rclpy.init(args=[
        "--ros-args",
        "-p", f"obstacle_grid_topic:={prefix}/grid",
        "-p", f"obstacle_output_topic:={prefix}/targets",
        "-p", f"obstacle_json_topic:={prefix}/obstacle",
        "-p", f"ownship_json_topic:={prefix}/ownship",
        "-p", f"ownship_output_topic:={prefix}/odom",
        "-p", "origin_latitude:=30.5472",
        "-p", "origin_longitude:=114.3891",
    ])
    try:
        yield
    finally:
        rclpy.shutdown()


def spin_until(executor, predicate, timeout=3.0):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        executor.spin_once(timeout_sec=0.02)
        if predicate():
            return True
    return False


def cell(grid, x, y):
    column = math.floor((x - grid.info.origin.position.x) / grid.info.resolution)
    row = math.floor((y - grid.info.origin.position.y) / grid.info.resolution)
    assert 0 <= column < grid.info.width and 0 <= row < grid.info.height
    return grid.data[row * grid.info.width + column]


# 验证：启动即有空栅格；带外壳的 0141 画成圆；count 为 0 整张清空；坏包不改动栅格。
def test_0141_packets_replace_the_obstacle_grid(ros, prefix):
    clients = []

    def factory(**settings):
        clients.append(FakeClient(**settings))
        return clients[-1]

    node = LaDDSInboundNode(tcp_client_factory=factory)
    driver = rclpy.create_node("rx_obstacle_driver_" + uuid.uuid4().hex[:8])
    grids, targets = [], []
    latched = QoSProfile(
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )
    driver.create_subscription(OccupancyGrid, prefix + "/grid", grids.append, latched)
    driver.create_subscription(String, prefix + "/targets", targets.append, 10)
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    executor.add_node(driver)
    try:
        # The static layer receives a valid map before the first 0141.
        assert spin_until(executor, lambda: grids)
        first = grids[-1]
        assert (first.info.width, first.info.height, list(first.data)) == (1, 1, [0])
        assert first.header.frame_id == "otter/map"

        latitude, longitude, _ = ORIGIN.to_geographic(40.0, 10.0)
        content = {"count": 1, "targets": [{
            "batch": 1, "latitude": latitude, "longitude": longitude, "threatingRadius": 5.0}]}
        wrapped = {"head": {"packageSeq": 1, "sendPlatform": 1001, "recvPlatform": 1000},
                   "content": content}
        clients[0].incoming.append(Packet(1, "0141", json.dumps(wrapped).encode("utf-8")))
        assert spin_until(executor, lambda: targets and len(grids) >= 2)
        assert json.loads(targets[-1].data) == content
        drawn = grids[-1]
        assert cell(drawn, 40.0, 10.0) == 100
        assert cell(drawn, 43.0, 10.0) == 100
        assert cell(drawn, 35.2, 5.2) == 0

        clients[0].incoming.append(Packet(1, "0141", b'{"count":0,"targets":[]}'))
        assert spin_until(executor, lambda: len(grids) >= 3)
        assert list(grids[-1].data) == [0]

        clients[0].incoming.append(Packet(1, "0141", b'{"count":2,"targets":[]}'))
        spin_until(executor, lambda: False, timeout=0.3)
        assert len(grids) == 3
    finally:
        executor.shutdown()
        driver.destroy_node()
        node.close()
        node.destroy_node()
