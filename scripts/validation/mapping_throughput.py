#!/usr/bin/env python3
"""Thirty-second source-node benchmark with exact TF verified before every scan.

No Gazebo or installed-tree writes. This conditions on TF availability and does
not replace a live run's unknown verdict. Driver and mapper share one process.
"""
import argparse
import hashlib
import importlib.util
import json
import math
import os
from pathlib import Path
import resource
import subprocess
import sys
import time
import uuid


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source-repo', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--domain-id', required=True, type=int)
    args = parser.parse_args()
    if not 1 <= args.domain_id <= 232:
        parser.error('Use a dedicated nonzero ROS domain in [1, 232]')
    repo = args.source_repo.resolve()
    module_path = repo / 'src/core/perception/perception/mapping_node.py'
    if not module_path.is_file():
        parser.error('source-repo does not contain the mapping node')
    args.output.mkdir(parents=True, exist_ok=False)
    sys.dont_write_bytecode = True
    os.environ.update(ROS_DOMAIN_ID=str(args.domain_id), ROS_LOCALHOST_ONLY='1')
    sys.path.insert(0, str(repo / 'src/core/perception'))
    import numpy as np
    import rclpy
    from geometry_msgs.msg import TransformStamped
    from nav_msgs.msg import OccupancyGrid
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from rclpy.time import Time
    from rosgraph_msgs.msg import Clock
    from sensor_msgs.msg import LaserScan
    from tf2_ros import TransformBroadcaster
    from perception.mapping_node import MappingNode
    from perception.scan_queue import ScanResult

    events = []
    phase = ['setup']
    def record(event, **values):
        row = dict(event=event, phase=phase[0], monotonic_s=time.monotonic(), **values)
        events.append(row)
        return row

    class ObservedMapper(MappingNode):
        # Keep the production deques and algorithms intact. Copy each produced
        # timing immediately so all 450 integrations survive the 256-entry cap.
        def _try_process(self, pending):
            result = super()._try_process(pending)
            record('scan_attempt', stamp_ns=pending.stamp_ns, result=result.value,
                   received_monotonic_s=pending.received_at,
                   processing_ms=float(self.processing_ms[-1]) if result == ScanResult.PROCESSED else None,
                   queue_ms=float(self.waiting_ms[-1]) if result == ScanResult.PROCESSED else None,
                   integrated_scans=self.integrated_scans)
            return result

        def _drain(self):
            super()._drain()
            checkpoint, size, age, oldest = self.pending._observations[-1]
            record('after_drain', checkpoint_monotonic_s=checkpoint, queue_size=size,
                   oldest_pending_age_ms=age, oldest_pending=oldest,
                   counters=vars(self.pending.counters).copy(), integrated_scans=self.integrated_scans)

        def _publish_map(self):
            super()._publish_map()
            record('map_published', serialization_ms=float(self.serialization_ms[-1]))

        def _log_metrics(self):
            super()._log_metrics()
            now = time.monotonic()
            payload = dict(queue_size=len(self.pending), integrated_scans=self.integrated_scans,
                           invalid_scans=self.invalid_scans, queue_counters=vars(self.pending.counters).copy(),
                           queue_observation_window=self.pending.observation_summary(now),
                           queue_ms=describe(list(self.waiting_ms)),
                           process=dict(pid=os.getpid(), monotonic=now))
            record('metrics', payload=payload)

    def describe(values):
        return {'count': len(values), **({k: float(v) for k, v in
                 zip(('p50', 'p95', 'p99'), np.percentile(values, [50, 95, 99]))}
                | {'max': float(max(values))} if values else {})}

    def sha(path):
        return hashlib.sha256(path.read_bytes()).hexdigest()

    sources = [module_path, module_path.with_name('mapping.py'), module_path.with_name('scan_queue.py'),
               repo / 'scripts/validation/compare_results.py', Path(__file__).resolve()]
    before = {str(p): sha(p) for p in sources}
    report = dict(schema_version=1, source_repo=str(repo), source_sha256_before=before,
                  source_commit=subprocess.check_output(['git', '-C', str(repo), 'rev-parse', 'HEAD'], text=True).strip(),
                  domain_id=args.domain_id, pid=os.getpid(),
                  protocol=dict(unique_scans=450, duration_s=30.0, beams=2000, scan_hz=15.0,
                                width=1024, resolution_m=0.5, range_min_m=0.1, range_max_m=25.0,
                                ranges='All 2000 beams are deterministic 24m hits; synthetic long-ray workload, not a recorded scene.',
                                map_publish_hz=2.0, queue_capacity=5, tf_wait_timeout_s=0.2,
                                clock='use_sim_time; continuous /clock derived from monotonic wall time, independent of WSL system-clock correction',
                                duplicate_phase='Three duplicates only after the 450 unique scans have integrated.'),
                  scope='Source MappingNode plus ROS driver in one process; CPU/RSS include driver, TF transport, instrumentation and subscriptions. Conditional normal-TF evidence only; no live verdict is rewritten.')
    namespace = 'mapping_throughput_' + uuid.uuid4().hex[:12]
    report['namespace'] = namespace
    rclpy.init(args=['--ros-args', '-r', '__ns:=/' + namespace, '-p', 'use_sim_time:=true'])
    mapper = driver = executor = None
    steady_start = steady_end = None
    try:
        mapper = ObservedMapper()
        driver = Node('driver')
        executor = SingleThreadedExecutor()
        executor.add_node(mapper)
        executor.add_node(driver)
        scans = driver.create_publisher(LaserScan, 'scan', 10)
        clocks = driver.create_publisher(Clock, '/clock', 10)
        transforms = TransformBroadcaster(driver)
        driver.create_subscription(OccupancyGrid, 'map', lambda msg: record(
            'map_received', stamp_ns=msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec), 10)

        def spin_until(predicate, deadline, description):
            while not predicate() and time.monotonic() < deadline:
                executor.spin_once(timeout_sec=0.001)
            if not predicate():
                raise RuntimeError('Timeout: ' + description)

        spin_until(lambda: scans.get_subscription_count() >= 1, time.monotonic() + 10, 'scan discovery')
        assert scans.get_subscription_count() == 1, 'Dedicated domain has another scan subscriber'
        assert mapper.get_parameter('width').value == 1024
        assert mapper.get_parameter('resolution').value == 0.5
        assert mapper.get_parameter('map_publish_frequency').value == 2.0
        assert mapper.pending.capacity == 5 and mapper.pending.tf_wait_timeout == 0.2
        spin_until(lambda: clocks.get_subscription_count() >= 1, time.monotonic()+5, 'clock discovery')
        initial_clock = Clock()
        initial_clock.clock.sec = 10
        clocks.publish(initial_clock)
        spin_until(lambda: mapper.get_clock().now().nanoseconds == 10_000_000_000,
                   time.monotonic()+5, 'initial controlled clock')
        report['backend'] = mapper.mapper.backend
        report['node_module'] = str(Path(sys.modules[MappingNode.__module__].__file__).resolve())
        frame = namespace + '/lidar'
        clock_anchor = time.monotonic()
        next_clock = clock_anchor
        t0 = clock_anchor + 0.5
        stamp0 = 10_500_000_000
        prepared, published = {}, []
        next_index = 0
        next_cpu = t0
        cpu_start = time.process_time()
        usage_start = cpu_started = None
        period = 1.0 / 15.0
        while next_index < 450 or time.monotonic() < t0 + 30.0:
            now = time.monotonic()
            if now >= next_clock:
                tick = Clock()
                clock_ns = 10_000_000_000 + round((now-clock_anchor)*1e9)
                tick.clock.sec, tick.clock.nanosec = divmod(clock_ns, 10**9)
                clocks.publish(tick)
                record('clock_published', clock_ns=clock_ns)
                next_clock = now + 0.02
            phase[0] = 'steady' if now >= t0 else 'setup'
            if steady_start is None and now >= t0:
                steady_start = t0
                usage_start = resource.getrusage(resource.RUSAGE_SELF)
                cpu_started = time.monotonic()
                record('steady_started', scheduled_monotonic_s=t0)
            if now >= next_cpu:
                usage = resource.getrusage(resource.RUSAGE_SELF)
                fields = Path('/proc/self/stat').read_text().rsplit(')', 1)[1].split()
                record('process', cpu_seconds=usage.ru_utime + usage.ru_stime,
                       rss_bytes=int(fields[21]) * os.sysconf('SC_PAGE_SIZE'),
                       vm_bytes=int(fields[20]), peak_rss_kib=usage.ru_maxrss)
                next_cpu += 1.0
            if next_index < 450:
                due = t0 + next_index * period
                stamp_ns = stamp0 + round(next_index * 10**9 / 15)
                if now >= due - 0.04 and next_index not in prepared:
                    transform = TransformStamped()
                    transform.header.frame_id, transform.child_frame_id = mapper.map_frame, frame
                    transform.header.stamp.sec, transform.header.stamp.nanosec = divmod(stamp_ns, 10**9)
                    transform.transform.rotation.w = 1.0
                    transforms.sendTransform(transform)
                    prepared[next_index] = time.monotonic()
                    record('tf_published', index=next_index, stamp_ns=stamp_ns)
                if now >= due:
                    query = Time(nanoseconds=stamp_ns)
                    # Check the actual mapper buffer immediately before publishing.
                    if mapper.tf_buffer.can_transform(mapper.map_frame, frame, query):
                        tf_ready = time.monotonic()
                        value = LaserScan()
                        value.header.frame_id = frame
                        value.header.stamp.sec, value.header.stamp.nanosec = divmod(stamp_ns, 10**9)
                        value.angle_min, value.angle_max = -math.pi, math.pi
                        value.angle_increment = 2 * math.pi / 1999
                        value.range_min, value.range_max = 0.1, 25.0
                        value.scan_time, value.time_increment = period, period / 2000
                        value.ranges = [24.0] * 2000
                        scans.publish(value)
                        record('scan_published', index=next_index, stamp_ns=stamp_ns,
                               scheduled_monotonic_s=due, tf_preverified_monotonic_s=tf_ready,
                               tf_exact_available=True, schedule_lateness_ms=(time.monotonic()-due)*1000)
                        published.append(value)
                        next_index += 1
                    elif now > due + 0.2:
                        raise RuntimeError(f'Exact TF unavailable before scan {next_index}; workload invalid')
            executor.spin_once(timeout_sec=0.001)
            if time.monotonic() > t0 + 31.0:
                raise RuntimeError('Could not deliver the fixed 450 scans within the measurement schedule')
        steady_end = t0 + 30.0
        usage_end = resource.getrusage(resource.RUSAGE_SELF)
        cpu_ended = time.monotonic()
        phase[0] = 'settle'
        spin_until(lambda: mapper.integrated_scans == 450 and len(mapper.pending) == 0,
                   time.monotonic() + 0.3, 'all 450 unique scans integrated')
        record('steady_finished', scheduled_monotonic_s=steady_end,
               integrated_scans=mapper.integrated_scans, counters=vars(mapper.pending.counters).copy())
        mapper._log_metrics()
        report['steady_cpu_seconds_including_driver'] = ((usage_end.ru_utime + usage_end.ru_stime)
                                                       - (usage_start.ru_utime + usage_start.ru_stime))
        report['steady_cpu_observation_duration_s'] = cpu_ended - cpu_started
        report['steady_cpu_mean_one_core_percent_including_driver'] = (report['steady_cpu_seconds_including_driver']
                                                                     / (cpu_ended-cpu_started) * 100)
        report['process_cpu_elapsed_s_including_setup'] = time.process_time() - cpu_start
        phase[0] = 'duplicate_check'
        before_map = hashlib.sha256(mapper.mapper.data.tobytes()).hexdigest()
        duplicates_before = mapper.pending.counters.duplicate
        for index in (0, 224, 449):
            value = published[index]
            value.ranges = [0.5] * 2000  # Would change the map if accidentally integrated.
            scans.publish(value)
            record('duplicate_published', original_index=index)
            expected = duplicates_before + (1 if index == 0 else 2 if index == 224 else 3)
            spin_until(lambda: mapper.pending.counters.duplicate == expected,
                       time.monotonic() + 1.0, 'duplicate rejection')
        assert mapper.integrated_scans == 450 and len(mapper.pending) == 0
        assert hashlib.sha256(mapper.mapper.data.tobytes()).hexdigest() == before_map
        report['duplicate_check'] = dict(passed=True, duplicate_delta=3, map_sha256_unchanged=before_map)
        report['final_counters'] = vars(mapper.pending.counters).copy()
        report['final_integrated_scans'] = mapper.integrated_scans
        report['final_queue_size'] = len(mapper.pending)
        integrations = [e for e in events if e['event'] == 'scan_attempt' and e['result'] == 'processed']
        unique_stamps = {e['stamp_ns'] for e in integrations}
        timings = [e['processing_ms'] for e in integrations]
        report['processing_ms_all_unique_scans'] = describe(timings)
        report['queue_ms_all_unique_scans'] = describe([e['queue_ms'] for e in integrations])
        pubs = [e for e in events if e['event'] == 'scan_published']
        report['publisher_lateness_ms'] = describe([e['schedule_lateness_ms'] for e in pubs])
        report['publisher_interval_ms'] = describe([(b['monotonic_s']-a['monotonic_s'])*1000 for a,b in zip(pubs[:-1],pubs[1:])])
        report['map_publications_in_steady_window'] = sum(e['event']=='map_published' and t0<=e['monotonic_s']<=steady_end for e in events)
        drains = [e for e in events if e['event']=='after_drain' and t0<=e['checkpoint_monotonic_s']<=steady_end]
        report['max_post_drain_queue_size'] = max(e['queue_size'] for e in drains)
        ages = [e['oldest_pending_age_ms'] for e in drains if e['oldest_pending_age_ms'] is not None]
        report['max_observed_pending_age_ms'] = max(ages, default=None)
        spec = importlib.util.spec_from_file_location('throughput_compare', repo/'scripts/validation/compare_results.py')
        comparison = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(comparison)
        metric_records = [dict(payload=e['payload'],wall_monotonic_s=e['monotonic_s'],source=f'events[{i}]')
                          for i,e in enumerate(events) if e['event']=='metrics' and e['phase'] in ('steady','settle')]
        report['frozen_backlog_check'] = comparison.backlog_performance(metric_records, dict(start=t0,end=steady_end,complete=True))
        counters = report['final_counters']
        report['checks'] = dict(all_450_unique_integrated_once=len(integrations)==len(unique_stamps)==450,
                               all_tf_ready_before_publish=len(pubs)==450 and all(e['tf_preverified_monotonic_s']<=e['monotonic_s'] for e in pubs),
                               no_tf_wait_attempts=all(e['result']=='processed' for e in events if e['event']=='scan_attempt'),
                               no_expiry_overflow_invalid=all(counters[k]==0 for k in ('expired','overflow','invalid','invalid_consumed','resets')),
                               processing_p95_le_66_7_ms=report['processing_ms_all_unique_scans']['p95']<=66.7,
                               fixed_input_schedule=max(e['schedule_lateness_ms'] for e in pubs)<=1000/15,
                               final_queue_empty=report['final_queue_size']==0,
                               frozen_no_backlog_rule=report['frozen_backlog_check']['status']=='pass',
                               map_2hz_publication_count=59<=report['map_publications_in_steady_window']<=61,
                               duplicate_map_unchanged=report['duplicate_check']['passed'])
        report['passed'] = all(report['checks'].values())
    except Exception as error:
        report.update(passed=False,error=f'{type(error).__name__}: {error}')
    finally:
        if mapper is not None:
            report.setdefault('final_counters', vars(mapper.pending.counters).copy())
            report.setdefault('final_integrated_scans', mapper.integrated_scans)
            report.setdefault('final_queue_size', len(mapper.pending))
        all_integrations = [e for e in events if e['event']=='scan_attempt' and e['result']=='processed']
        report.setdefault('processing_ms_all_unique_scans', describe([e['processing_ms'] for e in all_integrations]))
        report.setdefault('queue_ms_all_unique_scans', describe([e['queue_ms'] for e in all_integrations]))
        if executor is not None:
            executor.shutdown()
        if mapper is not None:
            mapper.destroy_node()
        if driver is not None:
            driver.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        report['source_sha256_after'] = {str(p):sha(p) for p in sources}
        report['source_unchanged'] = before == report['source_sha256_after']
        report['passed'] = report.get('passed',False) and report['source_unchanged']
        event_path = args.output/'events.jsonl'
        with event_path.open('x') as output:
            for event in events:
                output.write(json.dumps(event,allow_nan=False)+'\n')
        report['events_sha256'] = sha(event_path)
        report['event_count'] = len(events)
        report['steady_interval_monotonic_s'] = [steady_start,steady_end]
        with (args.output/'summary.json').open('x') as output:
            json.dump(report,output,indent=2,allow_nan=False)
            output.write('\n')
        print(json.dumps(report,indent=2),flush=True)
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
