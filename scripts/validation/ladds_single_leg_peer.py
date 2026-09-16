"""Independent DDS client: send one voyage command and wait for its 0052 result."""
import argparse
import json
import os
from pathlib import Path
import sys
import time

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'src/core/communication_module/usv_ladds_bridge'))
from usv_ladds_bridge.ladds_client import LaDDSClient


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--library', required=True)
    parser.add_argument('--task-file', required=True)
    parser.add_argument('--output', required=True)
    parser.add_argument('--domain-id', type=int, default=21)
    parser.add_argument('--duration', type=float, default=240)
    parser.add_argument('--warmup', type=float, default=5)
    args = parser.parse_args()
    task = json.loads(Path(args.task_file).read_text(encoding='utf-8-sig'))
    content = task.get('content', task)
    travel_id = content['travelId']
    payload = json.dumps(task, ensure_ascii=False).encode('utf-8')
    library = str(Path(args.library).resolve())
    output = Path(args.output).resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    result = dict(domain_id=args.domain_id, travelId=travel_id, sent_count=0,
                  feedback=[], success=False)
    original_cwd = Path.cwd()
    client = LaDDSClient(library, domain_id=args.domain_id,
                         send_topic='0211', receive_topic='0052')
    begun = time.monotonic()
    try:
        os.chdir(Path(library).parent)
        client.start()
        next_send = begun + args.warmup
        print(f'DDS test peer: domain={args.domain_id}, 0211 -> 0052, travelId={travel_id}', flush=True)
        while time.monotonic() - begun < args.duration:
            now = time.monotonic()
            if result['sent_count'] == 0 and now >= next_send:
                client.send(payload)
                result['sent_count'] += 1
            packet = client.receive()
            if packet is not None:
                try:
                    reply = json.loads(packet.payload.decode('utf-8'))
                    data = reply.get('content', reply)
                except (ValueError, UnicodeError, AttributeError):
                    continue
                result['feedback'].append(dict(topic=packet.topic, payload=reply,
                                               elapsed=now - begun))
                if packet.topic == '0052' and data == {'travelId': travel_id, 'taskFinish': 1}:
                    result['success'] = True
                    break
            time.sleep(.05)
    finally:
        client.close()
        os.chdir(original_cwd)
        result['elapsed'] = time.monotonic() - begun
        output.write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding='utf-8')
    print(json.dumps(result, ensure_ascii=True), flush=True)
    return 0 if result['success'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
