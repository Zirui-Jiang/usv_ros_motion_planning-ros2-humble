"""Export labels must identify the selected RMSE without hiding legacy values."""
import importlib.util
from pathlib import Path

_spec = importlib.util.spec_from_file_location("acceptance_plot", Path(__file__).with_name("plot_acceptance.py"))
plot = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(plot)


def test_old_report_uses_legacy_label_and_value():
    trial = {"row": {"cross_track_initial_m": {"rms": .2}}, "record": {"observations": {}}}
    assert plot.rmse_metric_name({}) == "legacy_wall"
    assert "legacy_wall" in plot.rmse_axis_label({})
    assert plot.rmse_export_values(trial, {})["selected_rmse_m"] == .2


def test_source_plot_csv_labels_and_both_values():
    report = {"method": {"rmse_metric": "source_time_trapezoid"}}
    trial = {"row": {"cross_track_initial_m": {"rms": .2}}, "record": {"observations": {"tracking": {
        "source_time_trapezoid": {"status": "valid", "rmse_m": .1, "max_source_gap_s": .1, "error_max_m": .3, "error_p95_m": .2}}}}}
    values = plot.rmse_export_values(trial, report)
    assert "source_time_trapezoid" in plot.rmse_axis_label(report)
    assert values["selected_rmse_metric"] == "source_time_trapezoid"
    assert values["selected_rmse_m"] == values["source_time_trapezoid_rmse_m"] == .1
    assert values["legacy_wall_rmse_m"] == .2
    assert values["source_max_gap_s"] == .1
    trial["record"]["observations"]["tracking"]["source_time_trapezoid"].update(status="unknown", rmse_m=None)
    assert plot.rmse_export_values(trial, report)["selected_rmse_m"] is None
