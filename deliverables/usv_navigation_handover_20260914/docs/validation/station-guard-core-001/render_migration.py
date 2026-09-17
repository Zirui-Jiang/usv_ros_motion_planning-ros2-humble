#!/usr/bin/env python3
"""Plot the first actual repeat of each completed migration matrix scenario."""
import argparse
from collections import Counter
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import sys
import tempfile


def sha256(path):
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--workspace", required=True, type=Path)
    parser.add_argument("--manifest", required=True, type=Path)
    args = parser.parse_args()
    workspace = args.workspace.resolve()
    matrix = workspace / "results/mission-matrix-001"
    summary_path = matrix / "summary.json"
    summary = json.loads(summary_path.read_text(encoding="utf-8"))
    if summary.get("all_passed") is not True or summary.get("counts", {}).get("running") != 0:
        raise ValueError("The selected matrix must be complete and passed before rendering")
    selected = [row for row in summary["results"] if row.get("matrix_repeat") == 1]
    scenarios = {row["scenario"] for row in selected}
    expected = {"station", "station_disturbance", "standby", "standby_obstacle",
                "orbit_cw", "orbit_ccw", "orbit_obstacle"}
    if len(selected) != 7 or scenarios != expected:
        raise ValueError("Expected exactly the first repeat of the seven recorded scenarios")
    if args.manifest.exists():
        raise ValueError("Render provenance manifest must be a new file")
    plotting_source = workspace / "src/core/usv_mission/validation/plot_missions.py"
    protected = {str(summary_path): sha256(summary_path), str(plotting_source): sha256(plotting_source)}
    directories = []
    for row in selected:
        directory = (Path(row["output"]) / row["measurement"]["trial"]).resolve()
        if not directory.is_relative_to(matrix.resolve()):
            raise ValueError("Trial output is outside the selected matrix")
        for filename in ("events.jsonl", "metrics.json"):
            path = directory / filename
            protected[str(path)] = sha256(path)
        directories.append((row, directory))

    # Avoid writing bytecode into frozen source or sharing a cache with ROS runs.
    sys.dont_write_bytecode = True
    os.environ["OPENBLAS_NUM_THREADS"] = "1"
    os.environ["OMP_NUM_THREADS"] = "1"
    with tempfile.TemporaryDirectory(prefix="mission-render-cache-") as cache:
        os.environ["MPLCONFIGDIR"] = cache
        spec = importlib.util.spec_from_file_location("frozen_mission_plotter", plotting_source)
        plotter = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(plotter)
        results = []
        for row, directory in directories:
            rendered = plotter.plot_trial(directory)
            files = [{"path": name, "sha256": sha256(Path(name)), "bytes": Path(name).stat().st_size}
                     for name in rendered["plots"]]
            results.append({"scenario": row["scenario"], "matrix_repeat": row["matrix_repeat"],
                            "trial_id": row["trial_id"], "trial_directory": str(directory), "files": files})
            print(json.dumps({"scenario": row["scenario"], "plots": len(files)}), flush=True)
    if any(sha256(Path(path)) != checksum for path, checksum in protected.items()):
        raise ValueError("Measured inputs or plotting source changed during rendering")
    extensions = Counter(Path(item["path"]).suffix for result in results for item in result["files"])
    provenance = {"workspace": str(workspace), "matrix_summary": str(summary_path),
                  "selection": "one real trial per scenario, matrix_repeat == 1; no rescoring",
                  "protected_input_sha256": protected, "inputs_unchanged": True,
                  "plot_counts": dict(extensions), "trials": results}
    args.manifest.parent.mkdir(parents=True, exist_ok=True)
    args.manifest.write_text(json.dumps(provenance, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(json.dumps({"trials": len(results), "plot_counts": dict(extensions),
                      "manifest": str(args.manifest), "inputs_unchanged": True}), flush=True)


if __name__ == "__main__":
    main()
