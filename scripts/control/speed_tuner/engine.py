"""Deterministic experiments using the real ADRC adapter and shared RK4 model.

No ROS, no actuator publisher, no duplicate PI implementation. Context captures
the loaded YAML bytes so each run/export has a reproducible source identity.
"""
from dataclasses import asdict
from datetime import datetime, timezone
import copy
import csv
import hashlib
import json
import math
from pathlib import Path
import sys
import time
import uuid

import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[3]
for relative in ('src/core/shared/common', 'src/core/control_module/controller',
                 'src/core/decision_module/trajectory_planner', 'src/core/shared/usv_navigation'):
    sys.path.insert(0, str(ROOT / relative))

from common.dynamics import step_dynamics
from common.models import AlgorithmStatus, ControlCommand, USVModel, VesselState
from controller import ADRCController
from trajectory_planner.trajectory import Trajectory
from usv_navigation.configuration import C, flatten, merge_explicit, resolve_parameters, section

CONFIG = ROOT / 'src/core/shared/usv_navigation/config/robots/otter/adrc_controller_params_otter.yaml'
DYNAMICS = CONFIG.with_name('dynamics_otter.yaml')
RUN_KEYS = {'kp', 'ki', 'target', 'duration', 'scenario', 'source_id'}
# Capture implementation provenance once at import, matching the loaded process.
_SOURCE_PATHS = [Path(__file__), *(Path(sys.modules[name].__file__) for name in (
    'common.dynamics', 'controller.adrc_controller', 'controller.adrc_core.pi_controller',
    'controller.adrc_core.adrc_controller', 'controller.adrc_core.heading_linear_regression_filter'))]
SOURCE_HASHES = {str(path.relative_to(ROOT)): hashlib.sha256(path.read_bytes()).hexdigest()
                 for path in _SOURCE_PATHS}


def number(value, name, low, high):
    if type(value) not in (int, float) or not math.isfinite(value) or not low <= value <= high:
        raise ValueError(f'{name} 必须是 {low:g}～{high:g} 之间的有限数值')
    return float(value)


class Context:
    def __init__(self, controller_path=CONFIG, dynamics_path=DYNAMICS):
        self.controller_path = Path(controller_path).resolve()
        self.dynamics_path = Path(dynamics_path).resolve()
        config_bytes = self.controller_path.read_bytes()
        dynamics_bytes = self.dynamics_path.read_bytes()
        self.document = yaml.safe_load(config_bytes)
        self.dynamics_document = yaml.safe_load(dynamics_bytes)
        values = flatten(self.dynamics_document)
        merge_explicit(values, flatten(self.document, 'usv_simulation'))
        self.resolved = resolve_parameters(values)
        if self.resolved[C + 'controller_name'] != 'adrc':
            raise ValueError('请选择 controller_name 为 adrc 的完整项目 YAML')
        self.gains = section(self.resolved, C + 'adrc_controller.')
        self.model = USVModel.from_mapping(section(self.resolved, 'dynamics.'))
        self.source_id = hashlib.sha256(config_bytes + b'\x00' + dynamics_bytes).hexdigest()
        self.config_sha = hashlib.sha256(config_bytes).hexdigest()
        self.dynamics_sha = hashlib.sha256(dynamics_bytes).hexdigest()
        self.dt = self.gains['sample_time']

    @classmethod
    def default(cls):
        return cls()

    def metadata(self):
        return dict(source_id=self.source_id, config_path=str(self.controller_path),
                    dynamics_path=str(self.dynamics_path), config_sha256=self.config_sha,
                    dynamics_sha256=self.dynamics_sha, implementation_sha256=SOURCE_HASHES,
                    gains=self.gains, model=asdict(self.model),
                    dt=self.dt, model_kind='无风浪的平面三自由度数值模型',
                    default=dict(kp=self.gains['k_p_speed'], ki=self.gains['k_i_speed'],
                                 target=min(1., self.model.velocity_max), duration=120., scenario='step'))


def settings(context, body):
    if not isinstance(body, dict) or set(body) - RUN_KEYS:
        raise ValueError('请求只能包含 Kp、Ki、参考速度、时长、场景和来源标识')
    if body.get('source_id', context.source_id) != context.source_id:
        raise ValueError('源配置已改变，请点击“重新读取配置”后再计算')
    result = dict(
        kp=number(body.get('kp', context.gains['k_p_speed']), 'Kp', 0, 300),
        ki=number(body.get('ki', context.gains['k_i_speed']), 'Ki', 0, 100),
        target=number(body.get('target', min(1., context.model.velocity_max)),
                      '参考速度', .1, min(3., context.model.velocity_max)),
        duration=number(body.get('duration', 120), '试验时长', 20, 180),
        scenario=body.get('scenario', 'step'), source_id=context.source_id)
    if result['scenario'] not in ('step', 'down', 'coast'):
        raise ValueError('未知数值试验场景')
    if result['duration'] / context.dt > 6000:
        raise ValueError('单次最多 6000 个控制样本，请缩短时长')
    steps = round(result['duration'] / context.dt)
    if not math.isclose(steps * context.dt, result['duration'], abs_tol=1e-7):
        raise ValueError('试验时长必须是控制采样周期的整数倍')
    return result


def phase_metrics(times, speeds, target, initial, start):
    """t90: first 90% progress crossing; settling: stays in band >=1 s to end.

    Time is relative to the evaluated reference change. Missing crossings stay
    None. Band is max(0.02 m/s, 5% of actual initial-to-target speed change).
    """
    t = np.asarray(times, dtype=float)
    u = np.asarray(speeds, dtype=float)
    delta = target - initial
    band = max(.02, .05 * abs(delta))
    selected = t >= start - 1e-8
    t, u = t[selected], u[selected]
    if not len(t):
        raise ValueError('指标窗口为空')
    progress = (u - initial) / delta if abs(delta) > 1e-9 else np.ones_like(u)
    crossing = np.flatnonzero(progress >= .9 - 1e-10)
    outside = np.flatnonzero(np.abs(u - target) > band + 1e-10)
    settle_index = int(outside[-1]) + 1 if len(outside) else 0
    settled = settle_index < len(t) and t[-1] - t[settle_index] >= 1 - 1e-8
    tail = u[t > max(start, t[-1] - 10) + 1e-8]
    return dict(t90_s=float(t[crossing[0]] - start) if len(crossing) else None,
                settling_s=float(t[settle_index] - start) if settled else None,
                overshoot_pct=float(max(0., np.max(progress) - 1) * 100),
                tail_mae_mps=float(np.mean(np.abs(tail - target))),
                band_mps=band, phase_start_s=start, phase_duration_s=float(t[-1] - start),
                phase_initial_mps=initial, phase_target_mps=target)


def _reference(speed, frame):
    return Trajectory([[0., 0., 0., 0., 0., 0., speed, 0.],
                       [400., 400*speed, 400*speed, 0., 0., 0., speed, 0.]],
                      stamp=0., frame_id=frame, goal=None)


def simulate(context, body):
    begun = time.perf_counter()
    cfg = settings(context, body)
    gains = {**context.gains, 'k_p_speed': cfg['kp'], 'k_i_speed': cfg['ki']}
    controller = ADRCController(context.model, gains)
    frame = context.model.ns + '/map'
    state = VesselState(frame_id=frame)
    target, dt = cfg['target'], context.dt
    count = round(cfg['duration'] / dt)
    change_index = count // 2
    final_target = target if cfg['scenario'] == 'step' else (target * .3 if cfg['scenario'] == 'down' else 0.)
    high, low = _reference(target, frame), _reference(final_target, frame)
    series = {k: [] for k in ('time', 'speed', 'reference', 'force', 'integral', 'proportional', 'left', 'right')}
    saturated = 0
    for i in range(count):
        changed = cfg['scenario'] != 'step' and i >= change_index
        reference_speed = final_target if changed else target
        if changed and cfg['scenario'] == 'coast':
            # Production SETTLING publishes zero and notifies on_control_hold.
            controller.on_control_hold()
            command = ControlCommand()
        else:
            command = controller.compute(state, low if changed else high, dt)
        if command.status != AlgorithmStatus.SUCCESS or not command.is_valid():
            raise ValueError('该组参数产生无效控制输出：' + command.message)
        force = (command.left + command.right) * context.model.max_force
        if (abs(force-gains['force_max']) < 1e-7 or abs(force-gains['force_min']) < 1e-7
                or max(abs(command.left), abs(command.right)) >= gains['command_limit']-1e-7):
            saturated += 1
        sample = (i*dt, state.u, reference_speed, force, controller.speed_loop.integral_speed,
                  controller.speed_loop.non_integral_output, command.left, command.right)
        for key, value in zip(series, sample):
            series[key].append(float(value))
        state = step_dynamics(state, command, context.model, dt)
    for key in series:
        value = count*dt if key == 'time' else state.u if key == 'speed' else series[key][-1]
        series[key].append(float(value))
    index = 0 if cfg['scenario'] == 'step' else change_index
    metrics = phase_metrics(series['time'], series['speed'], final_target,
                            series['speed'][index], index*dt)
    error = np.asarray(series['reference'][:-1]) - np.asarray(series['speed'][:-1])
    metrics.update(iae_m=float(np.sum(np.abs(error))*dt),
                   peak_force_n=max(abs(f) for f in series['force']),
                   saturation_pct=100*saturated/count,
                   peak_acceleration_mps2=float(np.max(np.abs(np.diff(series['speed'])/dt))),
                   final_speed_mps=state.u)
    return dict(settings=cfg, metrics=metrics, series=series, runtime_ms=(time.perf_counter()-begun)*1000,
                gazebo_verified=False, source=context.metadata())


def sweep(context, body):
    sweep_keys = {'kp_min', 'kp_max', 'ki_min', 'ki_max'}
    if not isinstance(body, dict) or set(body) - RUN_KEYS - sweep_keys:
        raise ValueError('筛选请求包含未知字段')
    run = settings(context, {k: v for k, v in body.items() if k in RUN_KEYS})
    bounds = [number(body.get(k, default), k, 0, ceiling) for k, default, ceiling in
              [('kp_min', 8, 300), ('kp_max', 120, 300), ('ki_min', .8, 100), ('ki_max', 30, 100)]]
    if bounds[0] >= bounds[1] or bounds[2] >= bounds[3]:
        raise ValueError('筛选范围的下限必须小于上限')
    begun = time.perf_counter()
    candidates = []
    for kp in np.linspace(bounds[0], bounds[1], 5):
        for ki in np.linspace(bounds[2], bounds[3], 5):
            result = simulate(context, {**run, 'kp': float(kp), 'ki': float(ki)})
            m = result['metrics']
            duration = m['phase_duration_s']
            t90 = m['t90_s'] if m['t90_s'] is not None else duration
            score = (m['iae_m'] / (run['duration']*run['target']) + .5*m['overshoot_pct']/100
                     + .25*t90/duration + .1*m['saturation_pct']/100)
            candidates.append(dict(kp=float(kp), ki=float(ki), score=score, metrics=m))
    return dict(candidates=sorted(candidates, key=lambda row: row['score']),
                runtime_ms=(time.perf_counter()-begun)*1000, settings=run,
                bounds=dict(zip(('kp_min', 'kp_max', 'ki_min', 'ki_max'), bounds)),
                method='5×5 等距网格；只比较当前模型和当前场景，分数越低越靠前',
                score_formula='IAE/(时长×目标速度) + 0.5×超调比例 + 0.25×T90/阶段时长 + 0.1×饱和比例；未达到90%时T90取阶段时长')


def wsl_path(path):
    path = Path(path).resolve()
    if path.drive:
        return '/mnt/' + path.drive[0].lower() + path.as_posix()[2:]
    return str(path)


def export_candidate(context, body, directory):
    result = simulate(context, body)
    document = copy.deepcopy(context.document)
    cfg = result['settings']
    document['controller']['adrc_controller'].update(k_p_speed=cfg['kp'], k_i_speed=cfg['ki'])
    # Validate the complete exported project configuration before writing anything.
    values = flatten(context.dynamics_document)
    merge_explicit(values, flatten(document, 'usv_simulation'))
    resolve_parameters(values)
    directory = Path(directory).resolve()
    directory.mkdir(parents=True, exist_ok=True)
    name = 'adrc_kp{:g}_ki{:g}_{}_{}'.format(cfg['kp'], cfg['ki'],
                datetime.now().strftime('%Y%m%d_%H%M%S'), uuid.uuid4().hex[:6])
    path = directory / (name + '.yaml')
    text = '# 数值调参候选；尚未通过 Gazebo/实艇验证。\n' + yaml.safe_dump(document, allow_unicode=True, sort_keys=False)
    result['exported_config_sha256'] = hashlib.sha256(text.encode('utf-8')).hexdigest()
    result['created_utc'] = datetime.now(timezone.utc).isoformat()
    with path.open('x', encoding='utf-8', newline='\n') as stream:
        stream.write(text)
    report_path, csv_path = path.with_suffix('.json'), path.with_suffix('.csv')
    report_path.write_text(json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False), encoding='utf-8')
    with csv_path.open('x', encoding='utf-8-sig', newline='') as stream:
        writer = csv.writer(stream)
        writer.writerow(result['series'])
        writer.writerows(zip(*result['series'].values()))
    return dict(path=str(path), report_path=str(report_path), csv_path=str(csv_path),
                wsl_path=wsl_path(path), yaml=text, metrics=result['metrics'],
                run_command='ros2 launch sim_env main.launch.py world:=ocean controller_name:=adrc controller_config:="'+wsl_path(path)+'"')
