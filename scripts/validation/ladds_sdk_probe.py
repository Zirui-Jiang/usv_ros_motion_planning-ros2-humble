"""只检查厂商 SDK 加载/建域能力；不读取或修改许可证。"""
import argparse
import ctypes
import json
import os
from pathlib import Path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--library', required=True)
    parser.add_argument('--create', action='store_true')
    parser.add_argument('--domain-id', type=int, default=231)
    parser.add_argument('--bind-ip', default='127.0.0.1')
    args = parser.parse_args()
    result = {'library': str(Path(args.library).resolve()), 'cwd': os.getcwd(),
              'domain_id': args.domain_id, 'bind_ip': args.bind_ip}
    dll_directory = None
    handle = 0
    try:
        if os.name == 'nt':
            dll_directory = os.add_dll_directory(str(Path(args.library).resolve().parent))
        sdk = ctypes.CDLL(args.library)
        sdk.LaDDS_GetVersion.argtypes = []
        sdk.LaDDS_GetVersion.restype = ctypes.c_char_p
        sdk.LaDDS_Create.argtypes = [ctypes.c_int, ctypes.c_uint8, ctypes.c_char_p]
        sdk.LaDDS_Create.restype = ctypes.c_uint64
        sdk.LaDDS_Destroy.argtypes = [ctypes.c_uint64]
        sdk.LaDDS_Destroy.restype = None
        result['version'] = sdk.LaDDS_GetVersion().decode('utf-8', errors='replace')
        result['loaded'] = True
        if args.create:
            handle = sdk.LaDDS_Create(args.domain_id, 1, args.bind_ip.encode() or None)
            result['created'] = bool(handle)
        else:
            result['created'] = None
    except (OSError, AttributeError) as exc:
        result.update(loaded=False, error=str(exc))
    finally:
        if handle:
            sdk.LaDDS_Destroy(handle)
        if dll_directory is not None:
            dll_directory.close()
    print(json.dumps(result, ensure_ascii=False), flush=True)
    return 0 if result.get('loaded') and (not args.create or result.get('created')) else 1


if __name__ == '__main__':
    raise SystemExit(main())
