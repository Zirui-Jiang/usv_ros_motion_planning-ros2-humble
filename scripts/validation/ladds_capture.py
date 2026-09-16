"""限时订阅 LaDDS，保存原始字节并判断文本/JSON；不发送数据。"""
import argparse
import base64
import ctypes
import json
import os
from pathlib import Path
import queue
import time


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--library', required=True)
    parser.add_argument('--topic', default='0211')
    parser.add_argument('--domain-id', type=int, default=1)
    parser.add_argument('--bind-ip', default='')
    parser.add_argument('--duration', type=float, default=15)
    parser.add_argument('--output', required=True)
    args = parser.parse_args()
    packets = queue.Queue(maxsize=100)
    dll_dir = os.add_dll_directory(str(Path(args.library).resolve().parent)) if os.name == 'nt' else None
    sdk = ctypes.CDLL(args.library)
    callback_type = ctypes.CFUNCTYPE(None, ctypes.c_uint64, ctypes.c_int, ctypes.c_char_p,
                                   ctypes.c_void_p, ctypes.c_int, ctypes.c_uint32, ctypes.c_uint32)
    sdk.LaDDS_Create.argtypes = [ctypes.c_int, ctypes.c_uint8, ctypes.c_char_p]
    sdk.LaDDS_Create.restype = ctypes.c_uint64
    sdk.LaDDS_Destroy.argtypes = [ctypes.c_uint64]
    sdk.LaDDS_Destroy.restype = None
    sdk.LaDDS_RegSubTopic.argtypes = [ctypes.c_uint64, ctypes.c_char_p, ctypes.c_int]
    sdk.LaDDS_RegSubTopic.restype = ctypes.c_int
    sdk.LaDDS_SetRecvCallback.argtypes = [ctypes.c_uint64, callback_type]
    sdk.LaDDS_SetRecvCallback.restype = None

    @callback_type
    def callback(handle, domain, topic, buffer, length, src, dst):
        if not buffer or not 0 < length <= 1048576:
            return
        try:
            packets.put_nowait((domain, topic.decode('utf-8', errors='replace'),
                               ctypes.string_at(buffer, length), src, dst))
        except queue.Full:
            pass

    handle = sdk.LaDDS_Create(args.domain_id, 1, args.bind_ip.encode() or None)
    if not handle:
        raise RuntimeError('LaDDS_Create returned 0')
    records = []
    try:
        sdk.LaDDS_SetRecvCallback(handle, callback)
        if not sdk.LaDDS_RegSubTopic(handle, args.topic.encode(), 2):
            raise RuntimeError('LaDDS_RegSubTopic failed')
        print('Subscribed: domain=%s topic=%s qos=best_effort' % (args.domain_id, args.topic), flush=True)
        end = time.monotonic() + args.duration
        while time.monotonic() < end and len(records) < 20:
            try:
                domain, topic, payload, src, dst = packets.get(timeout=min(.2, max(.001, end-time.monotonic())))
            except queue.Empty:
                continue
            record = {'domain_id': domain, 'topic': topic, 'src': src, 'dst': dst,
                      'length': len(payload), 'hex': payload.hex(),
                      'base64': base64.b64encode(payload).decode()}
            try:
                decoded = payload.decode('utf-8')
                record['utf8'] = decoded
                record['json'] = json.loads(decoded)
            except UnicodeDecodeError:
                record['utf8'] = None
            except json.JSONDecodeError:
                record['json'] = None
            records.append(record)
    finally:
        sdk.LaDDS_Destroy(handle)
        if dll_dir:
            dll_dir.close()
    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(records, ensure_ascii=False, indent=2), encoding='utf-8')
    print(json.dumps({'count': len(records), 'output': str(output),
                      'sample': {key: value for key, value in records[0].items() if key != 'base64'} if records else None},
                     ensure_ascii=True), flush=True)
    return 0 if records else 2


if __name__ == '__main__':
    raise SystemExit(main())
