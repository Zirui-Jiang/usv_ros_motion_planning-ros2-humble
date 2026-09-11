#!/usr/bin/env python3
"""Observe one acceptance harness and descendants through Linux /proc only.

Writes a new sidecar JSONL; never signals, attaches to, or changes a process.
CPU percentages use observed counter differences and may exceed 100% when a
process uses multiple cores. Processes missed between samples are not estimated.
"""

import argparse
from datetime import datetime, timezone
import hashlib
import json
import math
import os
from pathlib import Path
import sys
import time


def utc_now():
    return datetime.now(timezone.utc).isoformat()


def read_process(directory, ticks, page_size):
    observed = time.monotonic()
    raw = (directory / "stat").read_text()
    # comm may contain spaces and parentheses; fields after the final ')' are fixed.
    fields = raw.rsplit(")", 1)[1].split()
    command = [part.decode(errors="replace") for part in
               (directory / "cmdline").read_bytes().split(b"\0") if part]
    user_ticks, system_ticks = int(fields[11]), int(fields[12])
    return {"pid": int(directory.name), "ppid": int(fields[1]),
            "state": fields[0], "starttime_ticks": int(fields[19]),
            "cmd": command, "utime_ticks": user_ticks, "stime_ticks": system_ticks,
            "cpu_total_s": (user_ticks + system_ticks) / ticks,
            "rss_bytes": int(fields[21]) * page_size,
            "vm_bytes": int(fields[20]), "observed_monotonic_s": observed}


def census(ticks, page_size):
    processes, errors = {}, 0
    for directory in Path("/proc").iterdir():
        if directory.name.isdigit():
            try:
                process = read_process(directory, ticks, page_size)
                processes[process["pid"]] = process
            except (OSError, ValueError, IndexError):
                errors += 1  # Exit/race/permission failures remain explicit.
    return processes, errors


def matching_harness(command, output):
    if not any(Path(arg).name == "ros_acceptance.py" for arg in command):
        return False
    for index, argument in enumerate(command):
        if argument == "--output" and index + 1 < len(command):
            return command[index + 1] == output
        if argument.startswith("--output="):
            return argument.partition("=")[2] == output
    return False


def descendants(processes, root_pid):
    selected = {root_pid}
    while True:
        added = {pid for pid, process in processes.items()
                 if process["ppid"] in selected} - selected
        if not added:
            return selected
        selected.update(added)


def direct_role(process, root_pid):
    if process["pid"] == root_pid:
        return "harness"
    names = {Path(arg).name for arg in process["cmd"]}
    for name, role in (("sim_env_node", "navigation"),
                       ("perception_mapping_node", "mapping"),
                       ("costmap_support_node", "costmap"),
                       ("gzserver", "gazebo"), ("gzclient", "gazebo_gui"),
                       ("fake_localization_node", "localization"),
                       ("robot_state_publisher", "robot_state"),
                       ("joint_state_publisher", "joint_state"),
                       ("spawn_entity.py", "entity_spawner")):
        if name in names:
            return role
    if "ros2" in names and "launch" in process["cmd"]:
        return "ros_launch"
    return "other_descendant"


def classify(processes, selected, root_pid):
    roles = {pid: direct_role(processes[pid], root_pid) for pid in selected}
    for pid in selected:
        if roles[pid] != "other_descendant":
            continue
        ancestor = processes[pid]["ppid"]
        seen = {pid}
        while ancestor in selected and ancestor not in seen:
            if roles[ancestor] == "navigation":
                command = " ".join(processes[pid]["cmd"])
                if "multiprocessing.resource_tracker" in command:
                    roles[pid] = "navigation_resource_tracker"
                elif "multiprocessing.spawn" in command or "spawn_main" in command:
                    roles[pid] = "planning_worker"
                else:
                    roles[pid] = "navigation_child_unclassified"
                break
            seen.add(ancestor)
            ancestor = processes[ancestor]["ppid"]
    return roles


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--harness-output", required=True,
                        help="exact --output argument identifying the owned harness")
    parser.add_argument("--output", type=Path, required=True, help="new sidecar JSONL")
    parser.add_argument("--interval", type=float, default=1.0)
    parser.add_argument("--max-duration", type=float, default=1800.0)
    args = parser.parse_args(argv)
    if (not math.isfinite(args.interval) or args.interval <= 0 or
            not math.isfinite(args.max_duration) or not 0 < args.max_duration <= 1800):
        parser.error("interval must be positive; duration must be in (0, 1800] seconds")
    if args.output.exists() or args.output.is_symlink() or not args.output.parent.is_dir():
        parser.error("output must be new, with an existing parent directory")
    ticks, page_size = os.sysconf("SC_CLK_TCK"), os.sysconf("SC_PAGE_SIZE")
    begun, begun_utc = time.monotonic(), utc_now()
    processes, errors = census(ticks, page_size)
    candidates = [p for p in processes.values()
                  if matching_harness(p["cmd"], args.harness_output)]
    if len(candidates) != 1:
        parser.error(f"expected one live matching harness; found {len(candidates)}")
    root = candidates[0]
    root_pid, root_start = root["pid"], root["starttime_ticks"]
    if root["state"] == "Z":
        parser.error("matching harness has already exited")
    metadata = {"event": "sampler_started", "schema_version": 1,
                "started_utc": begun_utc, "started_monotonic_s": begun,
                "harness_output": args.harness_output, "root_pid": root_pid,
                "root_starttime_ticks": root_start, "root_cmd": root["cmd"],
                "interval_s": args.interval, "maximum_duration_s": args.max_duration,
                "clock_ticks_per_second": ticks, "page_size_bytes": page_size,
                "logical_cpus": os.cpu_count(), "sampler_pid": os.getpid(),
                "discovery_wall_s": time.monotonic() - begun,
                "script_sha256": hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
                "scope": "Observed /proc process tree only, from sampling start. No early-run coverage; no signals or process mutation. PID identity includes starttime. RSS sums double-count shared pages. CPU deltas require consecutive observations; exited/missed processes are not estimated."}
    previous, samples, overheads = {}, 0, []
    next_due = begun
    reason = "duration_limit"
    with args.output.open("x", encoding="utf-8") as stream:
        def emit(value):
            stream.write(json.dumps(value, ensure_ascii=False, allow_nan=False) + "\n")
            stream.flush()

        emit(metadata)
        print(json.dumps(metadata, ensure_ascii=False), flush=True)
        while time.monotonic() - begun < args.max_duration:
            started = time.monotonic() if samples else begun
            # Reuse the discovery census for the first sample to avoid a gap.
            if samples:
                processes, errors = census(ticks, page_size)
            current_root = processes.get(root_pid)
            if current_root is None or current_root["starttime_ticks"] != root_start:
                reason = "harness_exited_or_identity_changed"
                break
            if current_root["state"] == "Z":
                reason = "harness_zombie"
                break
            selected = descendants(processes, root_pid)
            roles = classify(processes, selected, root_pid)
            rows, current = [], {}
            for pid in sorted(selected):
                row = dict(processes[pid], role=roles[pid])
                identity = (pid, row["starttime_ticks"])
                prior = previous.get(identity)
                row.update(cpu_interval_s=None, cpu_delta_s=None, cpu_percent_one_core=None)
                if prior:
                    elapsed = row["observed_monotonic_s"] - prior["observed_monotonic_s"]
                    delta = row["cpu_total_s"] - prior["cpu_total_s"]
                    if elapsed > 0 and delta >= 0:
                        row.update(cpu_interval_s=elapsed, cpu_delta_s=delta,
                                   cpu_percent_one_core=100.0 * delta / elapsed)
                current[identity] = row
                rows.append(row)
            emit({"event": "sample", "sample_index": samples,
                  "monotonic_s": started, "utc": utc_now(),
                  "elapsed_since_sampler_start_s": started - begun,
                  "census_read_errors": errors, "processes": rows})
            overhead = time.monotonic() - started
            overheads.append(overhead)
            # This includes census/classification and sample serialization/flush.
            emit({"event": "sample_overhead", "sample_index": samples,
                  "wall_s": overhead})
            samples += 1
            previous = current
            next_due += args.interval
            time.sleep(max(0.0, min(next_due - time.monotonic(),
                                  begun + args.max_duration - time.monotonic())))
        finished = {"event": "sampler_finished", "finished_utc": utc_now(),
                    "finished_monotonic_s": time.monotonic(), "reason": reason,
                    "samples": samples, "sampler_wall_s": time.monotonic() - begun,
                    "sample_overhead_total_s": sum(overheads),
                    "sample_overhead_max_s": max(overheads, default=0.0),
                    "coverage_note": "Only recorded sample timestamps/CPU intervals are covered; sampler wall duration is not whole-run coverage."}
        emit(finished)
        print(json.dumps(finished, ensure_ascii=False), flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
