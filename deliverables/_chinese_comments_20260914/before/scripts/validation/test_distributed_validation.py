"""Guard against accepting absent or nonfinite output as stopped control."""
import importlib.util
from pathlib import Path
from types import SimpleNamespace

import pytest


@pytest.fixture
def harness(monkeypatch):
    directory = Path(__file__).parent
    monkeypatch.syspath_prepend(str(directory))
    spec = importlib.util.spec_from_file_location("distributed_validation_harness", directory / "ros_acceptance.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.fixture
def zero_window(harness):
    return harness.zero_output_window


def records():
    return {axis: [(index / 10. + .05, 0.) for index in range(10)]
            for axis in ("left", "right", "lateral")}


def test_regular_finite_zero_stream_covers_entire_window(zero_window):
    report = zero_window(records(), 0., 1.)
    assert report["passed"]
    assert all(axis["samples"] == 10 for axis in report["axes"].values())


@pytest.mark.parametrize("value", [float("nan"), float("inf"), -float("inf"), .01])
def test_nonfinite_or_nonzero_sample_cannot_pass(zero_window, value):
    data = records()
    data["left"][4] = (.45, value)
    assert not zero_window(data, 0., 1.)["passed"]


def test_one_cached_zero_per_axis_cannot_pass(zero_window):
    data = {axis: [(.05, 0.)] for axis in ("left", "right", "lateral")}
    assert not zero_window(data, 0., 1.)["passed"]


@pytest.mark.parametrize("kind", ["missing_axis", "stopped_stream", "interior_gap", "late_start"])
def test_missing_or_incomplete_stream_cannot_pass(zero_window, kind):
    data = records()
    if kind == "missing_axis":
        del data["lateral"]
    elif kind == "stopped_stream":
        data["left"] = data["left"][:4]
    elif kind == "interior_gap":
        data["right"] = [(t, v) for t, v in data["right"] if t < .25 or t > .75]
    else:
        data["left"] = data["left"][5:]
    assert not zero_window(data, 0., 1.)["passed"]


def test_nonmonotonic_receipts_are_rejected(zero_window):
    data = records()
    data["right"][4], data["right"][5] = data["right"][5], data["right"][4]
    assert not zero_window(data, 0., 1.)["passed"]


@pytest.mark.parametrize("control_matches", [False, True])
def test_decision_readback_cannot_substitute_for_actual_control(harness, monkeypatch, control_matches):
    name = "usv_simulation.controller.pid_controller.k_p_speed"
    events, called = [], []
    kind = SimpleNamespace(Request=SimpleNamespace)
    probe = SimpleNamespace(namespace="/boat", controller_node="usv_control",
        types={"SetParametersAtomically": kind, "GetParameters": kind,
               "Parameter": lambda *args, **kwargs: SimpleNamespace(to_parameter_msg=lambda: object())},
        log=SimpleNamespace(write=lambda event, **kwargs: events.append((event, kwargs))))

    def call_service(probe, kind, names, request):
        called.extend(names)
        if names[0].endswith("set_parameters_atomically"):
            return SimpleNamespace(result=SimpleNamespace(successful=True, reason=""))
        value = 4. if "usv_control" not in names[0] or control_matches else 3.
        return SimpleNamespace(values=[SimpleNamespace(type=3, double_value=value)])

    monkeypatch.setattr(harness, "call_service", call_service)
    times = iter([0., 6.])
    monkeypatch.setattr(harness.time, "monotonic", lambda: next(times))
    if control_matches:
        harness.apply_start_parameters(probe, {name: 4.})
        assert events[-1][0] == "pre_goal_control_parameter_readback"
    else:
        with pytest.raises(harness.TrialBlocked, match="actual control parameters"):
            harness.apply_start_parameters(probe, {name: 4.})
    assert "/boat/usv_control/get_parameters" in called
