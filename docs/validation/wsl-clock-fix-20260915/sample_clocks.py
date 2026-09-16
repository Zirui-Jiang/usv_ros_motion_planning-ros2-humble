"""Read-only WSL clock comparison; /dev/ptp0 is the Hyper-V time reference."""
import argparse
import ctypes
import json
import os
from pathlib import Path
import subprocess
import time


class Timeval(ctypes.Structure):
    _fields_ = [('sec', ctypes.c_long), ('usec', ctypes.c_long)]


class Timex(ctypes.Structure):
    _fields_ = [('modes', ctypes.c_uint), ('offset', ctypes.c_long),
                ('freq', ctypes.c_long), ('maxerror', ctypes.c_long),
                ('esterror', ctypes.c_long), ('status', ctypes.c_int),
                ('constant', ctypes.c_long), ('precision', ctypes.c_long),
                ('tolerance', ctypes.c_long), ('time', Timeval),
                ('tick', ctypes.c_long), ('ppsfreq', ctypes.c_long),
                ('jitter', ctypes.c_long), ('shift', ctypes.c_int),
                ('stabil', ctypes.c_long), ('jitcnt', ctypes.c_long),
                ('calcnt', ctypes.c_long), ('errcnt', ctypes.c_long),
                ('stbcnt', ctypes.c_long), ('tai', ctypes.c_int),
                ('padding', ctypes.c_int * 11)]


def snapshot():
    tx = Timex()  # modes=0: query only, never adjusts the clock.
    result = ctypes.CDLL(None, use_errno=True).adjtimex(ctypes.byref(tx))
    return dict(clocksource=Path('/sys/devices/system/clocksource/clocksource0/current_clocksource').read_text().strip(),
                adjtimex=dict(result=result, offset=tx.offset, freq_ppm=tx.freq/65536,
                              tick_us=tx.tick, status=tx.status, struct_size=ctypes.sizeof(tx)),
                ntp=subprocess.run(['timedatectl', 'timesync-status'],
                                   capture_output=True, text=True).stdout)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--duration', type=float, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        raise SystemExit('Refusing to overwrite existing evidence')
    before = snapshot()
    fd = os.open('/dev/ptp0', os.O_RDONLY)
    clock_id = ((~fd) << 3) | 3
    rows = []
    origin = time.clock_gettime_ns(clock_id)
    next_progress = 10.
    try:
        while True:
            ptp_before = time.clock_gettime_ns(clock_id)
            row = [time.time_ns(), time.monotonic_ns(),
                   time.clock_gettime_ns(time.CLOCK_MONOTONIC_RAW)]
            ptp_after = time.clock_gettime_ns(clock_id)
            rows.append(row + [(ptp_before+ptp_after)//2, ptp_after-ptp_before])
            elapsed = (ptp_after-origin)*1e-9
            if elapsed >= next_progress:
                print('PROGRESS', round(elapsed, 1), before['clocksource'], flush=True)
                next_progress += 10
            if elapsed >= args.duration:
                break
            time.sleep(.05)
    finally:
        os.close(fd)
    after = snapshot()
    first, last = rows[0], rows[-1]
    ptp_span = last[3]-first[3]
    jumps = [dict(at_ptp_s=(a[3]-first[3])*1e-9,
                  wall_jump_s=((b[0]-a[0])-(b[1]-a[1]))*1e-9)
             for a, b in zip(rows, rows[1:])
             if abs((b[0]-a[0])-(b[1]-a[1])) > 20_000_000]
    summary = dict(ptp_duration_s=ptp_span*1e-9,
                   raw_vs_ptp_ppm=((last[2]-first[2])/ptp_span-1)*1e6,
                   monotonic_vs_ptp_ppm=((last[1]-first[1])/ptp_span-1)*1e6,
                   wall_jumps=jumps, samples=len(rows),
                   max_ptp_read_span_ms=max(r[4] for r in rows)*1e-6)
    args.output.write_text(json.dumps(dict(before=before, after=after, summary=summary,
                          columns=['wall_ns', 'monotonic_ns', 'raw_ns', 'hyperv_ptp_ns', 'ptp_read_span_ns'],
                          samples=rows), indent=2), encoding='utf8')
    print(json.dumps(dict(summary=summary, before=before, after=after)), flush=True)


if __name__ == '__main__':
    main()
