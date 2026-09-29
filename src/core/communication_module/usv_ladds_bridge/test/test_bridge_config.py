"""ROS-independent tests for bridge parameter validation and payload decoding."""

from dataclasses import dataclass
from pathlib import Path
import json

import pytest
import yaml

from usv_ladds_bridge.config import (
    BridgeConfig,
    decode_utf8,
    drain_received,
    packet_info_json,
    receive_channel,
    validate_obstacle_payload,
)


def test_library_path_falls_back_to_environment():
    config = BridgeConfig.from_mapping(
        {"backend": "native", "library_path": ""},
        environ={"LADDS_LIBRARY_PATH": "/opt/ladds/lib/libladds.so"},
    )

    assert config.library_path == "/opt/ladds/lib/libladds.so"


def test_optional_stop_subscription_has_explicit_topic_routing():
    config = BridgeConfig.from_mapping({'stop_receive_topic': '0331'})
    assert receive_channel('0331', config.receive_topic, config.stop_receive_topic) == 'stop'
    assert receive_channel('0211', config.receive_topic, config.stop_receive_topic) == 'route'
    assert receive_channel('0111', config.receive_topic, config.stop_receive_topic) is None
    assert receive_channel(
        '0411',
        config.receive_topic,
        config.stop_receive_topic,
        config.ownship_receive_topic,
        config.obstacle_receive_topic,
    ) == 'ownship'
    assert receive_channel(
        '0141',
        config.receive_topic,
        config.stop_receive_topic,
        config.ownship_receive_topic,
        config.obstacle_receive_topic,
    ) == 'obstacle'
    assert receive_channel('0331', '0211', '') == 'route'  # generic bridge compatibility
    with pytest.raises(ValueError, match='different'):
        BridgeConfig.from_mapping({'stop_receive_topic': '0211'})


def test_optional_berth_subscription_is_separate_from_route_and_stop():
    config = BridgeConfig.from_mapping({
        'stop_receive_topic': '0331',
        'berth_receive_topic': '0212',
    })
    assert receive_channel(
        '0212',
        config.receive_topic,
        config.stop_receive_topic,
        config.ownship_receive_topic,
        config.obstacle_receive_topic,
        config.berth_receive_topic,
    ) == 'berth'
    assert receive_channel(
        '0211',
        config.receive_topic,
        config.stop_receive_topic,
        config.ownship_receive_topic,
        config.obstacle_receive_topic,
        config.berth_receive_topic,
    ) == 'route'
    assert receive_channel(
        '0331',
        config.receive_topic,
        config.stop_receive_topic,
        config.ownship_receive_topic,
        config.obstacle_receive_topic,
        config.berth_receive_topic,
    ) == 'stop'
    assert receive_channel('0212', '0211', '0331') is None
    with pytest.raises(ValueError, match='different'):
        BridgeConfig.from_mapping({'berth_receive_topic': '0211'})
    with pytest.raises(ValueError, match='different'):
        BridgeConfig.from_mapping({
            'stop_receive_topic': '0331',
            'berth_receive_topic': '0331',
        })
    with pytest.raises(ValueError, match='different'):
        BridgeConfig.from_mapping({'berth_receive_topic': '0411'})


def test_missing_library_path_has_actionable_error():
    with pytest.raises(ValueError, match="library_path.*LADDS_LIBRARY_PATH"):
        BridgeConfig.from_mapping(
            {"backend": "native", "library_path": ""}, environ={}
        )


def test_tcp_backend_does_not_require_native_library():
    config = BridgeConfig.from_mapping(
        {"backend": "tcp", "library_path": ""}, environ={}
    )

    assert config.library_path == ""


@pytest.mark.parametrize(
    ("override", "message"),
    [
        ({"max_payload_bytes": 0}, "max_payload_bytes"),
        ({"queue_depth": 0}, "queue_depth"),
        ({"poll_interval_sec": 0.0}, "poll_interval_sec"),
        ({"ttl": -1}, "ttl"),
        ({"ttl": 256}, "ttl"),
        ({"send_topic": ""}, "send_topic"),
        ({"ownship_send_topic": ""}, "ownship_send_topic"),
        ({"receive_topic": ""}, "receive_topic"),
        ({"ownship_receive_topic": "0211"}, "ownship_receive_topic"),
        ({"backend": "udp"}, "backend"),
        ({"relay_port": 0}, "relay_port"),
        ({"relay_port": 65536}, "relay_port"),
        ({"relay_timeout": 0.0}, "relay_timeout"),
        ({"ownship_json_topic": "/ladds/0411"}, "must not start with a number"),
        ({"status_send_topic": ""}, "status_send_topic"),
        ({"status_send_topic": "0411"}, "status_send_topic"),
        ({"progress_send_topic": ""}, "progress_send_topic"),
        ({"progress_send_topic": "0051"}, "progress_send_topic"),
        ({"heartbeat_input_topic": ""}, "heartbeat_input_topic"),
        ({"heartbeat_input_topic": "/0052/heartbeat"}, "must not start with a number"),
        ({"obstacle_grid_topic": "/ladds/0141/grid"}, "must not start with a number"),
        ({"obstacle_grid_topic": "/grid", "obstacle_grid_frame": " "}, "obstacle_grid_frame"),
        ({"obstacle_grid_resolution": 0.0}, "obstacle_grid_resolution"),
        ({"obstacle_default_radius": -1.0}, "obstacle_default_radius"),
        ({"obstacle_radius_scale": float("nan")}, "obstacle_radius_scale"),
        ({"obstacle_grid_max_size": 0.5}, "obstacle_grid_max_size"),
        ({"obstacle_grid_max_size": 10000.0, "obstacle_grid_resolution": 1.0}, "4096"),
        ({"obstacle_timeout_sec": -0.1}, "obstacle_timeout_sec"),
    ],
)
def test_invalid_transport_limits_are_rejected(override, message):
    values = {"library_path": "/tmp/libladds.so", **override}

    with pytest.raises(ValueError, match=message):
        BridgeConfig.from_mapping(values, environ={})


def test_default_transport_parameters_match_bridge_contract():
    config = BridgeConfig.from_mapping(
        {},
        environ={},
    )

    assert config.backend == "tcp"
    assert config.library_path == ""
    assert config.relay_host == "127.0.0.1"
    assert config.relay_port == 17880
    assert config.relay_timeout == 5.0
    assert config.domain_id == 1
    assert config.bind_ip == ""
    assert config.ttl == 1
    assert config.qos == "best_effort"
    assert config.send_topic == "0211"
    assert config.ownship_send_topic == "0411"
    assert config.obstacle_send_topic == "0141"
    assert config.ownship_input_topic == ""
    assert config.obstacle_input_topic == "/ladds/obstacle_in"
    assert config.receive_topic == "0211"
    assert config.ownship_receive_topic == "0411"
    assert config.obstacle_receive_topic == "0141"
    assert config.berth_receive_topic == ""
    assert config.ownship_output_topic == "/otter/odom"
    assert config.obstacle_output_topic == "/ladds/targets"
    assert config.ownship_json_topic == "/ladds/ownship"
    assert config.obstacle_json_topic == "/ladds/obstacle"
    assert config.ownship_frame_id == "otter/odom"
    assert config.ownship_child_frame_id == "otter/base_link"
    assert config.src == 0
    assert config.dst == 0
    assert config.encryption is False
    assert config.max_payload_bytes == 1_048_576
    assert config.queue_depth == 100
    assert config.poll_interval_sec == 0.01
    assert config.status_send_topic == "0051"
    assert config.heartbeat_input_topic == "/otter/navigation/heartbeat"
    assert config.progress_send_topic == "0053"
    assert config.thrust_send_topic == ""
    assert config.thrust_input_topic == "/otter/thrusters/vector_cmd"
    assert config.thrust_timeout_sec == 0.5


# 航路配置启用 0054；推力主题不能占用其它发送/接收主题，ROS 输入话题不能以数字开头。
def test_thrust_topic_is_separate_from_other_ladds_topics():
    config = BridgeConfig.from_mapping({"send_topic": "0052", "thrust_send_topic": "0054"})
    assert config.thrust_send_topic == "0054"
    for override in ({"thrust_send_topic": "0052"}, {"thrust_send_topic": "0051"},
                     {"thrust_send_topic": "0411"}, {"thrust_send_topic": "0211"}):
        with pytest.raises(ValueError, match="thrust_send_topic"):
            BridgeConfig.from_mapping({"send_topic": "0052", **override})
    for override, message in (({"thrust_input_topic": "/0054/cmd"}, "thrust_input_topic"),
                              ({"thrust_input_topic": " "}, "thrust_input_topic"),
                              ({"thrust_timeout_sec": 0.0}, "thrust_timeout_sec"),
                              ({"thrust_send_topic": " "}, "thrust_send_topic")):
        with pytest.raises(ValueError, match=message):
            BridgeConfig.from_mapping({"send_topic": "0052", "thrust_send_topic": "0054", **override})


def test_ownship_transport_topic_and_ros_input_are_configurable():
    config = BridgeConfig.from_mapping({
        "send_topic": "0052",
        "ownship_send_topic": "0411",
        "ownship_input_topic": "/otter/odom",
    })
    assert config.send_topic == "0052"
    assert config.ownship_send_topic == "0411"
    assert config.ownship_input_topic == "/otter/odom"


def test_ownship_output_and_json_topics_are_configurable():
    config = BridgeConfig.from_mapping({
        "ownship_receive_topic": "0411",
        "ownship_output_topic": "/boat/odom",
        "ownship_json_topic": "/ladds/ownship",
        "ownship_frame_id": "boat/odom",
        "ownship_child_frame_id": "boat/base_link",
    })
    assert config.ownship_receive_topic == "0411"
    assert config.ownship_output_topic == "/boat/odom"
    assert config.ownship_json_topic == "/ladds/ownship"
    assert config.ownship_frame_id == "boat/odom"
    assert config.ownship_child_frame_id == "boat/base_link"


def test_obstacle_transport_topic_and_ros_input_are_configurable():
    config = BridgeConfig.from_mapping({
        "obstacle_send_topic": "0141",
        "obstacle_input_topic": "/navigation/obstacles",
    })
    assert config.obstacle_send_topic == "0141"
    assert config.obstacle_input_topic == "/navigation/obstacles"


# YAML 1.1 会把未加引号的 0141 读成八进制整数 97，桥参数必须保持字符串。
def test_bridge_yaml_keeps_ladds_topic_ids_as_strings():
    config_dir = Path(__file__).resolve().parents[1] / "config"
    topic_keys = (
        "send_topic",
        "ownship_send_topic",
        "obstacle_send_topic",
        "status_send_topic",
        "progress_send_topic",
        "thrust_send_topic",
        "receive_topic",
        "stop_receive_topic",
        "berth_receive_topic",
        "ownship_receive_topic",
        "obstacle_receive_topic",
    )
    route = yaml.safe_load((config_dir / "route.yaml").read_text(encoding="utf-8"))
    assert route["usv_ladds_rx"]["ros__parameters"]["thrust_send_topic"] == "0054"
    for filename in ("bridge.yaml", "route.yaml", "single_leg.yaml"):
        loaded = yaml.safe_load((config_dir / filename).read_text(encoding="utf-8"))
        assert "usv_ladds_rx" in loaded
        params = loaded["usv_ladds_rx"]["ros__parameters"]
        for key in topic_keys:
            if key not in params or params[key] in ("", None):
                continue
            assert isinstance(params[key], str), f"{filename} {key}={params[key]!r}"
        assert params["obstacle_send_topic"] == "0141"
        assert params["ownship_send_topic"] == "0411"
        assert params["status_send_topic"] == "0051"
        assert params["progress_send_topic"] == "0053"
        assert params["heartbeat_input_topic"] == "/otter/navigation/heartbeat"
        if "ownship_json_topic" in params and params["ownship_json_topic"]:
            assert params["ownship_json_topic"] == "/ladds/ownship"
        if "obstacle_json_topic" in params and params["obstacle_json_topic"]:
            assert params["obstacle_json_topic"] == "/ladds/obstacle"
        if filename in ("route.yaml", "single_leg.yaml"):
            assert "usv_ladds_route" in loaded
            assert params["obstacle_input_topic"] == "/ladds/obstacle_in"
            assert params["obstacle_output_topic"] == "/ladds/targets"
            assert params["berth_receive_topic"] == "0212"
            route_params = loaded["usv_ladds_route"]["ros__parameters"]
            assert route_params["berth_receive_topic"] == "/ladds/berth_rx"
            assert route_params["progress_topic"] == "/ladds/progress_tx"
        assert "usv_ladds_radar" in loaded
        radar = loaded["usv_ladds_radar"]["ros__parameters"]
        # route.yaml 只收 0141：仿真激光目标改到没人转发的话题，其余配置仍走上行。
        expected_radar_output = (
            "/ladds/lidar_targets" if filename == "route.yaml" else "/ladds/obstacle_in"
        )
        assert radar["output_topic"] == expected_radar_output
        assert radar["scan_topic"].endswith("/lidar/scan")


# 验证：航路配置关闭 0141 回灌并打开障碍栅格，浮点参数写成浮点，ROS 声明时类型才一致。
def test_route_profile_receives_0141_into_obstacle_grid():
    config_dir = Path(__file__).resolve().parents[1] / "config"
    route = yaml.safe_load((config_dir / "route.yaml").read_text(encoding="utf-8"))
    params = route["usv_ladds_rx"]["ros__parameters"]
    assert params["obstacle_json_topic"] == ""
    assert params["obstacle_grid_topic"] == "/otter/ladds/obstacle_grid"
    for key in (
        "obstacle_grid_resolution",
        "obstacle_grid_max_size",
        "obstacle_default_radius",
        "obstacle_radius_scale",
        "obstacle_timeout_sec",
    ):
        assert type(params[key]) is float, f"route.yaml {key}={params[key]!r}"
    config = BridgeConfig.from_mapping(params, environ={})
    assert config.obstacle_grid_frame == route["usv_ladds_route"]["ros__parameters"]["map_frame"]
    # The grid must use the same origin that decodes 0411 into odometry.
    assert (config.origin_latitude, config.origin_longitude) == (
        params["origin_latitude"], params["origin_longitude"])


@pytest.mark.parametrize(
    ("text", "message"),
    [
        ('{"count": 1, "targets": []}', "count must match"),
        ('{"count": 1, "targets": [{} ,  {}]}', "count must match"),
        ('{"count": 1, "targets": [1]}', r"targets\[0\] must be an object"),
        ('{"count": true, "targets": []}', "count must be an integer"),
        ('{"count": 256, "targets": []}', "count must be in"),
        ('{"count": 0, "targets": {}}', "targets must be a list"),
        ('{"count": 0}', "missing required field: targets"),
        ('[]', "root must be an object"),
        ('not-json', "invalid obstacle JSON"),
    ],
)
def test_obstacle_payload_validation_rejects_invalid_count_or_targets(text, message):
    with pytest.raises(ValueError, match=message):
        validate_obstacle_payload(text)


def test_obstacle_payload_validation_keeps_target_attributes_opaque():
    text = (
        '{"count":1,"targets":[{"batch":7,"longitude":114.3,'
        '"vendorField":"原样保留"}]}'
    )
    payload = validate_obstacle_payload(text)
    assert payload["count"] == 1
    assert payload["targets"][0]["batch"] == 7
    assert payload["targets"][0]["vendorField"] == "原样保留"


def test_decode_utf8_returns_text_only_for_valid_utf8():
    assert decode_utf8("避碰".encode("utf-8")) == "避碰"
    assert decode_utf8(b"\xff\xfe") is None


@dataclass(frozen=True)
class _Packet:
    domain_id: int
    topic: str
    payload: bytes
    src: int
    dst: int


class _QueuedReceiver:
    def __init__(self, packets):
        self._packets = list(packets)

    def receive(self):
        return self._packets.pop(0) if self._packets else None


def test_receive_drain_stops_at_per_tick_limit():
    packets = [_Packet(1, "targets", bytes([value]), 10, 20) for value in range(3)]
    receiver = _QueuedReceiver(packets)

    first_tick = drain_received(receiver, limit=2)
    second_tick = drain_received(receiver, limit=2)

    assert [packet.payload for packet in first_tick] == [b"\x00", b"\x01"]
    assert [packet.payload for packet in second_tick] == [b"\x02"]


def test_packet_info_json_contains_observation_fields_and_byte_length():
    packet = _Packet(7, "radar/targets", b"\x00\xff\x10", 41, 52)

    assert json.loads(packet_info_json(packet)) == {
        "domain": 7,
        "topic": "radar/targets",
        "src": 41,
        "dst": 52,
        "length": 3,
    }
