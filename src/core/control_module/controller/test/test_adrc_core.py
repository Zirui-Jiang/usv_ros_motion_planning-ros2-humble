"""Numerical contracts independent of ROS and of the outer adapter."""
import copy
import importlib.util
import math
from pathlib import Path
import sys

import pytest

# Load the core as a standalone package: no ROS or trajectory imports required.
CORE = Path(__file__).parents[1] / "controller" / "adrc_core"
spec = importlib.util.spec_from_file_location("tested_adrc_core", CORE / "__init__.py", submodule_search_locations=[str(CORE)])
core = importlib.util.module_from_spec(spec)
sys.modules[spec.name] = core
spec.loader.exec_module(core)
PI = core.PIController
Heading = core.ADRCController
Filter = sys.modules["tested_adrc_core.heading_linear_regression_filter"].HeadingLinearRegressionFilter


@pytest.mark.parametrize("direction", [-1, 1])
def test_multiple_turns(direction):
    f = Filter(window_length=3)
    for i in range(150):
        angle = direction * i * .1
        f.update(math.atan2(math.sin(angle), math.cos(angle)))
        assert f.last_unwrapped == pytest.approx(angle)


def test_irregular_time_and_duplicate():
    f = Filter(window_length=5)
    for t in (0., .08, .21, .31, .49):
        f.update(.3 + .2 * t, stamp=t)
    assert f.output()[1] == pytest.approx(.2)
    before = copy.deepcopy(f.__dict__)
    f.update(1., stamp=.49)
    assert f.__dict__ == before
    with pytest.raises(ValueError):
        f.update(.5, stamp=.4)
    assert f.__dict__ == before


def test_actual_dt_and_legacy_fit():
    f = Filter(sample_time=.1, window_length=3)
    for angle in (0., .2, .4):
        f.update(angle, dt=.2)
    assert f.output()[1] == pytest.approx(1.)
    assert f._fit([0, 1, 4]) == pytest.approx((11 / 3, 2))


@pytest.mark.parametrize("kwargs", [{"window_length":2.5}, {"history_length":2}, {"window_length":1}, {"sample_time":0}])
def test_filter_configuration_rejects_invalid(kwargs):
    with pytest.raises(ValueError):
        Filter(**kwargs)


@pytest.mark.parametrize("cls,bad", [(PI,{"force_min":1}), (PI,{"integral_min":8}), (Heading,{"filter_window_length":2.5}), (Heading,{"filter_history_length":2}), (Heading,{"acceleration_gain":-1}), (Heading,{"error_reset_deg":181}), (Heading,{"integral_min":1}), (Heading,{"moment_max":-1})])
def test_config_atomic(cls, bad):
    c = cls()
    before = dict(c.config)
    with pytest.raises(ValueError):
        c.update_config(bad)
    assert c.config == before


def test_structure_reset_and_gain_preservation():
    c = Heading()
    c.compute(0, .1, 0, 0)
    old = c.heading_filter
    c.update_config({"k_p_heading":2})
    assert c.heading_filter is old and c.previous_error is not None
    c.update_config({"sample_time":.2,"filter_window_length":4})
    assert c.heading_filter is not old
    assert c.heading_filter.window_length == 4
    assert c.previous_error is None
    assert c.integral_heading == 0


def test_first_steps_baseline_and_actual_integral_dt():
    p, h = PI(), Heading()
    assert p.compute(0, 0, .5, .2, .1) == pytest.approx(2.424)
    assert h.compute(.1, 0, 0, 0, .1) == pytest.approx(.1503)
    assert p.compute(0, 0, .5, .2, .2) == pytest.approx(2.472)


def test_final_integral_matches_output_when_candidate_saturates():
    p = PI({"k_p_speed":0,"k_i_speed":1,"force_min":-1,"force_max":1})
    p.integral_speed = .9
    assert p.compute(0, 0, 1, 0, .2) == pytest.approx(.9)
    h = Heading({"k_p_heading":0,"k_d_heading":0,"k_i_heading":1,"moment_min":-.1,"moment_max":.1,"error_reset_deg":180})
    h.integral_heading = .09
    assert h.compute(0, .1, 0, 0, .2) == pytest.approx(-.09)


def test_large_error_resets_before_output_and_refresh_does_not_advance():
    h = Heading({"k_d_heading":0,"moment_min":-10,"moment_max":10})
    h.integral_heading = .2
    assert h.compute(0, .5, 0, 0) == pytest.approx(-.75)
    assert h.integral_reset and h.integral_heading == 0
    before = copy.deepcopy(h.heading_filter.__dict__)
    assert h.refresh_output() == pytest.approx(-.75)
    assert h.heading_filter.__dict__ == before


@pytest.mark.parametrize("cls", [PI, Heading])
@pytest.mark.parametrize("index", range(4))
def test_all_scalar_inputs_finite(cls,index):
    c = cls()
    args = [0.,0.,0.,0.]
    args[index] = float("nan")
    with pytest.raises(ValueError):
        c.compute(*args)


def test_filter_nonfinite_atomic():
    f = Filter()
    f.update(.1)
    before = copy.deepcopy(f.__dict__)
    with pytest.raises(ValueError):
        f.update(float("nan"))
    assert f.__dict__ == before


def test_overflow_is_not_hidden_by_saturation():
    p = PI({"k_p_speed":1e308})
    with pytest.raises(ValueError):
        p.compute(0,0,1e308,0)
    h = Heading({"k_p_heading":1e308})
    with pytest.raises(ValueError):
        h.compute(0,3.,0,0)


def test_original_multistep_baseline():
    # Frozen from unchanged 0914controller before any core numerical fixes.
    p, h = PI(), Heading()
    expected = [(2.424, .1503), (2.448, .08557),
                (2.472, .0437141666666667), (2.496, .06383483333333335)]
    for measured, (surge, yaw) in zip((0., .01, .025, .03), expected):
        assert p.compute(.1, measured, .5, .2, .1) == pytest.approx(surge)
        assert h.compute(.1, measured, .5, .2, .1) == pytest.approx(yaw)


def test_heading_stamp_overrides_dt_for_filter_only():
    h = Heading({"filter_window_length":3})
    for stamp in (0., .2, .4):
        h.compute(0, stamp, 0, 0, .1, measurement_stamp=stamp)
    assert h.heading_filter.rate == pytest.approx(1.)


def test_single_sample_and_pi_refresh():
    f = Filter()
    f.update(3.1, stamp=1_700_000_000.)
    assert f.output() == pytest.approx((3.1, 0, 0))
    p = PI()
    p.compute(0,0,.2,0)
    p.integral_speed = 0
    assert p.refresh_output() == pytest.approx(1.6)
    p.reset()
    assert p.non_integral_output == p.refresh_output() == 0
