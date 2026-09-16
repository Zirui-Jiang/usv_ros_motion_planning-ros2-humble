"""独立 DDS 验收端：仅回复 bridge_test 标记，不转发或执行业务报文。"""
import argparse
import json
from pathlib import Path
import sys
import time
import uuid

sys.path.insert(0, str(Path(__file__).resolve().parents[2] /
                      'src/core/communication_module/usv_ladds_bridge'))
from usv_ladds_bridge.ladds_client import LaDDSClient


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--library', required=True)
    parser.add_argument('--duration', type=float, default=20)
    parser.add_argument('--output', required=True)
    args = parser.parse_args()
    client = LaDDSClient(args.library, domain_id=1, send_topic='0211',
                         receive_topic='0211', qos='best_effort')
    observed, replies, seen = [], [], set()
    try:
        client.start()
        print('Independent LaDDS roundtrip peer ready on domain 1 / 0211', flush=True)
        end = time.monotonic() + args.duration
        while time.monotonic() < end:
            packet = client.receive()
            if packet is None:
                time.sleep(.01)
                continue
            try:
                message = json.loads(packet.payload.decode('utf-8'))
            except (UnicodeError, json.JSONDecodeError):
                continue
            if not isinstance(message, dict) or message.get('bridge_test') != 'ros2_to_ladds':
                continue
            probe_id = message.get('probe_id')
            if not isinstance(probe_id, str):
                continue
            observed.append(message)
            if probe_id in seen:
                continue
            reply = {'bridge_test': 'ladds_to_ros2', 'in_reply_to': probe_id,
                     'reply_id': str(uuid.uuid4())}
            client.send(json.dumps(reply).encode())
            seen.add(probe_id)
            replies.append(reply)
    finally:
        client.close()
    result = {'observed_ros_markers': observed, 'sent_replies': replies}
    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding='utf-8')
    print(json.dumps(result), flush=True)
    return 0 if observed and replies else 1


if __name__ == '__main__':
    raise SystemExit(main())
