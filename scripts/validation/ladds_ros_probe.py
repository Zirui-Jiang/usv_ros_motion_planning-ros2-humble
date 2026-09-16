"""观察真实 ROS 桥接收，并可发送不含航行指令的唯一测试标记。"""
import argparse
import json
from pathlib import Path
import time
import uuid

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, UInt8MultiArray


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--duration', type=float, default=15)
    parser.add_argument('--output', required=True)
    parser.add_argument('--send-marker', action='store_true')
    parser.add_argument('--require-reply', action='store_true', help='要求独立 DDS 探针对本次唯一标记回复')
    args = parser.parse_args()
    if args.require_reply and not args.send_marker:
        parser.error('--require-reply requires --send-marker')
    rclpy.init(args=[])
    node = Node('ladds_communication_probe')
    texts, raw, metadata = [], [], []
    def keep(target, value):
        if len(target) < 100:
            target.append(value)
    subscriptions = [
        node.create_subscription(String, '/ladds/rx', lambda msg: keep(texts, msg.data), 100),
        node.create_subscription(UInt8MultiArray, '/ladds/rx_bytes', lambda msg: keep(raw, bytes(msg.data).hex()), 100),
        node.create_subscription(String, '/ladds/rx_info', lambda msg: keep(metadata, json.loads(msg.data)), 100),
    ]
    publisher = node.create_publisher(String, '/ladds/tx', 10)
    marker = {'bridge_test': 'ros2_to_ladds', 'probe_id': str(uuid.uuid4())}
    sent = 0
    start = time.monotonic()
    try:
        while time.monotonic() - start < args.duration:
            rclpy.spin_once(node, timeout_sec=.1)
            if args.send_marker and sent < 3 and time.monotonic()-start > 2+sent and publisher.get_subscription_count():
                publisher.publish(String(data=json.dumps(marker)))
                sent += 1
    finally:
        node.destroy_node()
        rclpy.shutdown()
    parsed = []
    for value in texts:
        try:
            parsed.append(json.loads(value))
        except json.JSONDecodeError:
            pass
    external = [value for value in parsed if isinstance(value, dict) and 'head' in value and 'content' in value]
    exact_raw_matches = sum(value.encode().hex() in raw for value in texts)
    replies = [value for value in parsed if isinstance(value, dict)
               and value.get('bridge_test') == 'ladds_to_ros2'
               and value.get('in_reply_to') == marker['probe_id']]
    result = {'text_count': len(texts), 'raw_count': len(raw), 'metadata_count': len(metadata),
              'external_json_count': len(external), 'exact_raw_matches': exact_raw_matches,
              'matching_peer_replies': len(replies),
              'marker': marker if args.send_marker else None, 'marker_published': sent,
              'received_text': texts, 'received_raw_hex': raw, 'received_metadata': metadata}
    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding='utf-8')
    print(json.dumps({key:value for key,value in result.items() if not key.startswith('received_')}, ensure_ascii=False))
    received_expected = bool(replies) if args.require_reply else bool(external)
    return 0 if received_expected and exact_raw_matches and (not args.send_marker or sent) else 1


if __name__ == '__main__':
    raise SystemExit(main())
