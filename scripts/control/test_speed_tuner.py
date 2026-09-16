"""Behavioral checks for numerical tuning, metrics, and safe candidate exports."""
import json
from pathlib import Path
import sys

import pytest
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parent))
from speed_tuner.engine import Context, phase_metrics, simulate, sweep, export_candidate


@pytest.fixture
def context():
    return Context.default()


def test_first_step_uses_production_pi_force_and_actual_model(context):
    result = simulate(context, {"kp": 8, "ki": .8, "duration": 20})
    assert result["series"]["force"][0] == pytest.approx(8.08)
    assert result["series"]["speed"][0] == 0
    assert result["series"]["time"][-1] == pytest.approx(20)
    assert context.model.mass + context.model.x_dot_u == pytest.approx(34.84)


def test_original_numeric_acceptance_is_reproduced(context):
    result = simulate(context, {"kp": 8, "ki": .8, "duration": 180})
    pairs = zip(result["series"]["time"], result["series"]["speed"])
    tail = [abs(1-u) for t, u in pairs if t > 160 + 1e-7]
    assert sum(tail) / len(tail) == pytest.approx(.004780707154923185, abs=1e-10)
    assert result["metrics"]["t90_s"] == pytest.approx(67.1)


def test_missing_crossing_and_settling_are_not_zero():
    result = phase_metrics([0, 1, 2, 3], [0, .1, .2, .3], 1, 0, 0)
    assert result["t90_s"] is None
    assert result["settling_s"] is None


def test_down_step_uses_direction_and_late_excursion_prevents_settling():
    result = phase_metrics([0, 1, 2, 3, 4], [1, .4, .1, .01, .2], 0, 1, 0)
    assert result["t90_s"] == pytest.approx(2)
    assert result["settling_s"] is None
    assert result["overshoot_pct"] == 0


@pytest.mark.parametrize("body", [{"kp": -1}, {"ki": float('nan')},
    {"duration": 999}, {"scenario": "unknown"}, {"target": 0}, {"kp": True},
    {"force_max": 1000}, {"target": 99}])
def test_invalid_or_out_of_scope_inputs_rejected(context, body):
    with pytest.raises(ValueError):
        simulate(context, body)


def test_actual_settling_policy_cuts_thrust_and_clears_integral(context):
    result = simulate(context, {"kp": 60, "ki": 8, "duration": 20, "scenario": "coast"})
    for t, force, integral in zip(result['series']['time'][:-1],
                                 result['series']['force'], result['series']['integral']):
        if t >= 10 - 1e-8:
            assert force == 0
            assert integral == 0
    assert result['series']['speed'][-1] < result['series']['speed'][100]


def test_sweep_respects_ranges_and_does_not_mutate_base(context):
    original = dict(context.gains)
    result = sweep(context, {"duration": 20, "kp_min": 8, "kp_max": 12,
                             "ki_min": .8, "ki_max": 1.2})
    assert len(result['candidates']) == 25
    assert all(8 <= r['kp'] <= 12 and .8 <= r['ki'] <= 1.2 for r in result['candidates'])
    assert result['candidates'] == sorted(result['candidates'], key=lambda r: r['score'])
    assert context.gains == original


def test_export_preserves_other_algorithms_and_includes_evidence(context, tmp_path):
    original = context.controller_path.read_bytes()
    result = export_candidate(context, {"kp": 40, "ki": 4, "duration": 20}, tmp_path)
    exported = yaml.safe_load(Path(result['path']).read_text(encoding='utf-8'))
    before = yaml.safe_load(original)
    for algorithm in ('pid_controller', 'lqr_controller'):
        assert exported['controller'][algorithm] == before['controller'][algorithm]
    assert exported['controller']['adrc_controller']['k_p_speed'] == 40
    assert context.controller_path.read_bytes() == original
    assert Path(result['report_path']).is_file()
    assert Path(result['csv_path']).is_file()
    assert json.loads(Path(result['report_path']).read_text(encoding='utf-8'))['gazebo_verified'] is False


def test_http_api_rejects_cross_origin_and_changed_sources(tmp_path):
    pytest.importorskip('flask', reason='Flask is an optional dependency of the local tuning UI')
    from speed_tuner.web import create_app
    app = create_app(export_dir=tmp_path)
    client = app.test_client()
    assert client.get('/api/config').status_code == 200
    assert client.post('/api/analyze', json={'source_id': 'old'}).status_code == 400
    assert client.post('/api/export', json={}, headers={'Origin': 'https://example.com'}).status_code == 403
    assert list(tmp_path.iterdir()) == []
    result = client.post('/api/analyze', json={'duration': 20, 'kp': 40, 'ki': 4})
    assert result.status_code == 200
    assert result.json['candidate']['settings']['kp'] == 40
    assert result.json['baseline']['settings']['kp'] == 8
    assert client.post('/api/analyze', json={'kp': -1}).status_code == 400
