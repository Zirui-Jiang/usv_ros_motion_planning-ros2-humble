#!/usr/bin/env python3
"""Execute existing C++ shared libraries, then compare migrated Python APIs."""
import argparse
import hashlib
import json
import math
from pathlib import Path
import subprocess
import sys


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def fixtures():
    line = [[i, 0, 0] for i in range(9)]
    bend = [[0, 0, 0.2], [1, 0.2, 0.3], [2, 1, 0.6], [3, 1.2, 0.2]]
    affine = [[i, 2*i, 0.5*i, 0.2*i, 0.1*i, 0.01*i, 1+0.2*i, 0.1*i] for i in range(5)]
    nonlinear = [[i, i, i*i, 0, 0, 0, 1, 0] for i in range(5)]
    reference = [[0, 0, 0.1, 0.05, 0.05, 0, 0.5, 0], [1, 0.5, 0.1, 0.05, 0.05, 0, 0.5, 0], [2, 1, 0.1, 0.05, 0.05, 0, 0.5, 0]]
    cases = [
        dict(id="resample_straight", operation="resample", points=line, spacing=0.4),
        dict(id="resample_bend", operation="resample", points=bend, spacing=0.3),
        dict(id="rdp_bend", operation="rdp", points=bend, epsilon=0.05, max_interval=0.7),
        dict(id="rdp_straight", operation="rdp", points=line, epsilon=0.1, max_interval=1.3),
        dict(id="sg_straight_interior", operation="sg", points=line, selection="interior_xy"),
        dict(id="sg_full_pose_probe", operation="sg", points=line, semantic_probe="原SG末点未固定，航向置零；Python修复端点并重算内部航向"),
        dict(id="trajectory_time_affine", operation="trajectory_time", points=affine, queries=[0,0.25,1,1.75,4]),
        dict(id="trajectory_distance_affine", operation="trajectory_distance", points=affine, queries=[0,0.5,2,3.5,8]),
        dict(id="trajectory_time_bracket_probe", operation="trajectory_time", points=nonlinear, queries=[0.5,1.5], semantic_probe="原lower_bound使用上界及下一点，Python修复为相邻包围区间；查询远离末段避免原越界"),
        dict(id="pid_two_ticks", operation="pid", points=reference, states=[[0,0,0,0.2,0,0],[0.01,0,0.01,0.25,0,0]], config=dict(goal_distance_tolerance=0.1, lookahead_time=1, k_p_speed=0.5,k_i_speed=0.1,k_d_speed=0.01,k_p_course=0.2,k_i_course=0.01,k_d_course=0.001)),
        dict(id="lqr_forward_output", operation="lqr", points=reference, states=[[0.08,0.04,0.04,0.45,0,0],[0.09,0.045,0.045,0.48,0,0]], config=dict(goal_distance_tolerance=0.1,lookahead_time=1,discrete_dt=0.1,solve_eps=0.1,max_iterations=1000,Q_matrix_diag=[100,100,100,10,10,0],R_matrix_diag=[1,1]))]
    return {"model": dict(mass=29.84,i_z=10.03,D=0.395,max_force=50,x_dot_u=5,y_dot_v=5,n_dot_r=1,x_u=-20,y_v=-20,n_r=-20), "cases": cases}


def python_case(case, model):
    import numpy as np
    from common.models import USVModel, VesselState
    from path_planner.path_planner import resample
    from path_processor.ramer_douglas_peucker import ramer_douglas_peucker
    from path_processor.savitzky_golay import savitzky_golay
    from trajectory_planner.trajectory import Trajectory
    from controller.pid_controller import PIDController
    from controller.lqr_controller import LQRController
    op, points = case["operation"], case["points"]
    if op == "resample": return resample(points,case["spacing"])
    if op == "rdp": return ramer_douglas_peucker(points,case["epsilon"],case["max_interval"])
    if op == "sg": return savitzky_golay(points)
    trajectory = Trajectory(points, goal=(100,0))
    if op.startswith("trajectory_"):
        query = trajectory.reference_at_time if op.endswith("time") else trajectory.reference_at_distance
        return np.array([query(value) for value in case["queries"]])
    control = (PIDController if op == "pid" else LQRController)(USVModel(**model),case["config"])
    result = []
    for s in case["states"]:
        command = control.compute(VesselState(*s),trajectory,0.1)
        if command.status.value != "success": raise RuntimeError(str(command))
        result.append([command.left,command.right,command.lateral])
    return np.array(result)


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument("--baseline",type=Path,required=True)
    p.add_argument("--python-source",type=Path,required=True)
    p.add_argument("--output",type=Path,required=True,help="new isolated WSL build/result directory")
    p.add_argument("--atol",type=float,default=1e-6)
    p.add_argument("--rtol",type=float,default=1e-5)
    args=p.parse_args()
    if not all(math.isfinite(v) and v >= 0 for v in (args.atol,args.rtol)):
        p.error("atol and rtol must be finite and nonnegative")
    baseline=args.baseline.resolve(); source=args.python_source.resolve(); output=args.output.resolve()
    if output.exists(): p.error("output must be a new directory")
    if baseline == output or baseline in output.parents: p.error("output must be outside baseline")
    output.mkdir(parents=True)
    tool=Path(__file__).resolve().parent
    fixture=fixtures(); fixture_path=output/"fixtures.json"
    fixture_path.write_text(json.dumps(fixture,indent=2,ensure_ascii=False))
    def run(command,name):
        with (output/name).open("w") as log:
            subprocess.run(command,stdout=log,stderr=subprocess.STDOUT,check=True,timeout=100)
    run(["cmake","-S",str(tool),"-B",str(output/"build"),"-DCMAKE_BUILD_TYPE=Release"],"configure.log")
    run(["cmake","--build",str(output/"build"),"-j2"],"build.log")
    binary=output/"build/cpp_golden"
    run([str(binary),str(fixture_path),str(output/"cpp_golden.tsv")],"cpp_execution.log")
    for relative in ("src/core/common","src/core/path_planner/path_planner","src/core/path_planner/path_processor","src/core/trajectory_planner","src/core/controller/controller"):
        sys.path.insert(0,str(source/relative))
    import numpy as np
    cpp={}
    for line in (output/"cpp_golden.tsv").read_text().splitlines():
        columns=line.split("\t"); cpp[columns[0]]=np.array([float(v) for v in columns[1:]])
    results=[]
    for case in fixture["cases"]:
        actual=python_case(case,fixture["model"]); expected=cpp[case["id"]]
        if case.get("selection")=="interior_xy":
            actual=actual[1:-1,:2]; expected=expected.reshape(-1,3)[1:-1,:2]
        actual=actual.ravel(); expected=expected.ravel()
        passed=actual.shape==expected.shape and bool(np.allclose(actual,expected,atol=args.atol,rtol=args.rtol))
        results.append(dict(id=case["id"],operation=case["operation"],selection=case.get("selection","all_output_values"),semantic_probe=case.get("semantic_probe"),passed=passed,max_abs_error=float(np.max(np.abs(actual-expected))) if actual.shape==expected.shape else None,cpp=expected.tolist(),python=actual.tolist()))
    libraries=list((baseline/"install").glob("*/lib/lib*.so"))
    required={"libcommon.so","libpath_planner.so","libpath_processor.so","libtrajectory_planner.so","libcontroller.so"}
    provenance=dict(baseline=str(baseline),baseline_commit=subprocess.check_output(["git","-C",str(baseline),"rev-parse","HEAD"],text=True).strip(),python_source=str(source),python_commit=subprocess.check_output(["git","-C",str(source),"rev-parse","HEAD"],text=True).strip(),python_dirty=subprocess.check_output(["git","-C",str(source),"status","--porcelain"],text=True),fixture_sha256=sha(fixture_path),golden_executable_sha256=sha(binary),libraries={str(lib):sha(lib) for lib in libraries if lib.name in required})
    baseline_files=("src/core/path_planner/path_planner/src/path_planner.cpp", "src/core/path_planner/path_processor/src/path_prune/ramer_douglas_peucker.cpp", "src/core/path_planner/path_processor/src/path_smooth/savitzky_golay.cpp", "src/core/trajectory_planner/src/trajectory.cpp", "src/core/controller/controller/src/pid_controller/otter_pid_controller.cpp", "src/core/controller/controller/src/lqr_controller/otter_lqr_controller.cpp", "src/core/controller/controller/src/controller.cpp")
    python_files=("src/core/path_planner/path_planner/path_planner/path_planner.py", "src/core/path_planner/path_processor/path_processor/ramer_douglas_peucker.py", "src/core/path_planner/path_processor/path_processor/savitzky_golay.py", "src/core/trajectory_planner/trajectory_planner/trajectory.py", "src/core/controller/controller/controller/pid_controller.py", "src/core/controller/controller/controller/lqr_controller.py", "src/core/controller/controller/controller/controller.py", "src/core/common/common/models.py", "src/core/common/common/math_helper.py")
    provenance["baseline_source_sha256"]={name:sha(baseline/name) for name in baseline_files}
    provenance["python_source_sha256"]={name:sha(source/name) for name in python_files}
    provenance["tool_sha256"]={name:sha(tool/name) for name in ("golden.cpp","run.py","CMakeLists.txt")}
    run(["ldd",str(binary)],"linked_libraries.txt")
    linked=(output/"linked_libraries.txt").read_text()
    if any(str(baseline/"install"/name/"lib"/("lib"+name+".so")) not in linked for name in ("path_planner","path_processor","trajectory_planner","controller")):
        raise RuntimeError("executable did not link all expected baseline install libraries")
    report=dict(provenance=provenance,atol=args.atol,rtol=args.rtol,results=results,normal_passed=sum(r["passed"] for r in results if not r["semantic_probe"]),normal_total=sum(not r["semantic_probe"] for r in results),probe_differences=sum(not r["passed"] for r in results if r["semantic_probe"]),not_covered=["A*/Dijkstra/GBFS require Costmap2DROS lifecycle initialization; bounded tool does not instantiate them", "LQR A/B/K are private/local; only public control outputs executed, matrices not independently compared", "LOS rollouts and LiDAR mapping not included", "Trajectory nonlinear last interval is excluded because original upper bracket can access out of bounds", "PID wrap discontinuity/reverse/lateral LQR feedback semantic repairs excluded from normal cases"])
    (output/"report.json").write_text(json.dumps(report,indent=2,ensure_ascii=False,allow_nan=False))
    lines=["# 原C++ / Python 数值对照", "",f"正常用例通过 {report['normal_passed']}/{report['normal_total']}；语义探针发现 {report['probe_differences']} 处差异。", "", f"C++ 输出来自实际执行已构建基线共享库；atol={args.atol:g}、rtol={args.rtol:g}。语义探针不计正常用例通过数。", "", "|用例|结果|最大绝对差|范围/说明|", "|---|---|---|---|"]
    for r in results: lines.append(f"|{r['id']}|{'一致' if r['passed'] else '差异'}|{r['max_abs_error']}|{r['semantic_probe'] or r['selection']}|")
    lines += ["", "未覆盖：", "", *["- "+v for v in report["not_covered"]], "", "完整输入、原C++ TSV、编译/执行日志、ldd与提交/库SHA256见同目录。测试不启动Gazebo，不证明闭环性能。"]
    (output/"report.md").write_text("\n".join(lines)+"\n")
    print(json.dumps({k:report[k] for k in ("normal_passed","normal_total","probe_differences")}))
    return 0 if report["normal_passed"]==report["normal_total"] else 1


if __name__=="__main__":
    sys.exit(main())
