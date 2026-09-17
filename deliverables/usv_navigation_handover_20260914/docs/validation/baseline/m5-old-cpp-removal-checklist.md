# M5 旧 C++ 精确移除清单（待执行）

状态：只列清单，没有删除任何文件。等待新 Python 闭环确认后，由主任务明确启动移除步骤。

当前清单包含 21 个目标、78 个实际文件。JSON 文件逐一记录 SHA256、字节数及 Git 跟踪状态；后续执行前必须核对内容和路径仍符合此快照。

保留全部 plugins、fake_localization_node.cpp、costmap_support_node.cpp 和 sim_env/CMakeLists.txt。保留 Python 实现、安装入口、resource、package.xml、测试、配置、世界、模型及基线标签/独立 checkout。

计划中目录仅用于分组；执行时优先依据 JSON 的逐文件路径处理，防止后来新增文件被递归删除。未列出的文件不在此次移除范围。每个实际目标必须解析到本工作区内部；对内容变化或新文件应重新核查。

## src/core/common/src

- src/core/common/src/geometry/aabox2d.cpp
- src/core/common/src/geometry/collision_checker.cpp
- src/core/common/src/geometry/curve/bezier_curve.cpp
- src/core/common/src/geometry/curve/bspline_curve.cpp
- src/core/common/src/geometry/curve/cubic_spline_curve.cpp
- src/core/common/src/geometry/curve/curve.cpp
- src/core/common/src/geometry/curve/dubins_curve.cpp
- src/core/common/src/geometry/curve/quintic_polynomial.cpp
- src/core/common/src/geometry/curve/reeds_shepp_curve.cpp
- src/core/common/src/geometry/line_segment2d.cpp
- src/core/common/src/geometry/polygon2d.cpp
- src/core/common/src/geometry/vec2d.cpp
- src/core/common/src/math/math_helper.cpp
- src/core/common/src/util/log.cpp
- src/core/common/src/util/visualizer.cpp

## src/core/common/include

- src/core/common/include/common/geometry/aabox2d.h
- src/core/common/include/common/geometry/collision_checker.h
- src/core/common/include/common/geometry/curve/bezier_curve.h
- src/core/common/include/common/geometry/curve/bspline_curve.h
- src/core/common/include/common/geometry/curve/cubic_spline_curve.h
- src/core/common/include/common/geometry/curve/curve.h
- src/core/common/include/common/geometry/curve/dubins_curve.h
- src/core/common/include/common/geometry/curve/quintic_polynomial.h
- src/core/common/include/common/geometry/curve/reeds_shepp_curve.h
- src/core/common/include/common/geometry/line_segment2d.h
- src/core/common/include/common/geometry/point.h
- src/core/common/include/common/geometry/polygon2d.h
- src/core/common/include/common/geometry/vec2d.h
- src/core/common/include/common/math/math_helper.h
- src/core/common/include/common/structure/kd_tree.h
- src/core/common/include/common/structure/node.h
- src/core/common/include/common/structure/singleton.h
- src/core/common/include/common/structure/usv_model.h
- src/core/common/include/common/util/log.h
- src/core/common/include/common/util/visualizer.h

## src/core/common/CMakeLists.txt

- src/core/common/CMakeLists.txt

## src/core/controller/controller/src

- src/core/controller/controller/src/controller.cpp
- src/core/controller/controller/src/controller_factory.cpp
- src/core/controller/controller/src/lqr_controller/otter_lqr_controller.cpp
- src/core/controller/controller/src/pid_controller/otter_pid_controller.cpp

## src/core/controller/controller/include

- src/core/controller/controller/include/controller/controller.h
- src/core/controller/controller/include/controller/controller_factory.h
- src/core/controller/controller/include/controller/lqr_controller/otter_lqr_controller.h
- src/core/controller/controller/include/controller/pid_controller/otter_pid_controller.h

## src/core/controller/controller/CMakeLists.txt

- src/core/controller/controller/CMakeLists.txt

## src/core/path_planner/path_planner/src

- src/core/path_planner/path_planner/src/graph_planner/astar_planner.cpp
- src/core/path_planner/path_planner/src/path_planner.cpp
- src/core/path_planner/path_planner/src/path_planner_core.cpp
- src/core/path_planner/path_planner/src/utils/path_planner_factory.cpp

## src/core/path_planner/path_planner/include

- src/core/path_planner/path_planner/include/path_planner/graph_planner/astar_planner.h
- src/core/path_planner/path_planner/include/path_planner/path_planner.h
- src/core/path_planner/path_planner/include/path_planner/path_planner_core.h
- src/core/path_planner/path_planner/include/path_planner/utils/motions.h
- src/core/path_planner/path_planner/include/path_planner/utils/path_planner_factory.h

## src/core/path_planner/path_planner/CMakeLists.txt

- src/core/path_planner/path_planner/CMakeLists.txt

## src/core/path_planner/path_processor/src

- src/core/path_planner/path_processor/src/path_processor/path_processor.cpp
- src/core/path_planner/path_processor/src/path_prune/ramer_douglas_peucker.cpp
- src/core/path_planner/path_processor/src/path_smooth/savitzky_golay.cpp

## src/core/path_planner/path_processor/include

- src/core/path_planner/path_processor/include/path_planner/path_processor/path_processor.h
- src/core/path_planner/path_processor/include/path_planner/path_prune/ramer_douglas_peucker.h
- src/core/path_planner/path_processor/include/path_planner/path_smooth/savitzky_golay.h

## src/core/path_planner/path_processor/CMakeLists.txt

- src/core/path_planner/path_processor/CMakeLists.txt

## src/core/trajectory_planner/src

- src/core/trajectory_planner/src/los_trajectory_planner.cpp
- src/core/trajectory_planner/src/trajectory.cpp
- src/core/trajectory_planner/src/trajectory_planner.cpp
- src/core/trajectory_planner/src/trajectory_planner_factory.cpp

## src/core/trajectory_planner/include

- src/core/trajectory_planner/include/trajectory_planner/los_trajectory_planner.h
- src/core/trajectory_planner/include/trajectory_planner/trajectory.h
- src/core/trajectory_planner/include/trajectory_planner/trajectory_planner.h
- src/core/trajectory_planner/include/trajectory_planner/trajectory_planner_factory.h

## src/core/trajectory_planner/CMakeLists.txt

- src/core/trajectory_planner/CMakeLists.txt

## src/core/perception/src

- src/core/perception/src/mapping/mapping.cpp
- src/core/perception/src/mapping/mapping_node.cpp

## src/core/perception/include

- src/core/perception/include/perception/mapping/mapping.h

## src/core/perception/CMakeLists.txt

- src/core/perception/CMakeLists.txt

## src/sim_env/src/simulation.cpp

- src/sim_env/src/simulation.cpp

## src/sim_env/src/simulation_node.cpp

- src/sim_env/src/simulation_node.cpp

## src/sim_env/include/sim_env/simulation.h

- src/sim_env/include/sim_env/simulation.h

## 执行后的验证要求

在独立干净安装区重新构建原 13 个包，核对 Python 包 discover/install 与两个人工保留 C++ 入口，再运行要求的源码/ROS 集成检查。数值对照工具继续链接独立原基线安装区，不依赖主分支删除的旧头文件。此次仅生成清单，未进行上述后续操作。
