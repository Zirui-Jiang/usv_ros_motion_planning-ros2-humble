set -eo pipefail
source /opt/ros/humble/setup.bash
source /home/usv/module-layout-20260914/install/setup.bash
/usr/bin/colcon --log-base /tmp/usv_ladds_single_leg_20260915/log build --base-paths '/mnt/d/ROS2轻量仿真系统/usv_ros_motion_planning-ros2-humble/src/core/decision_module/path_processor' '/mnt/d/ROS2轻量仿真系统/usv_ros_motion_planning-ros2-humble/src/core/decision_module/path_planner' '/mnt/d/ROS2轻量仿真系统/usv_ros_motion_planning-ros2-humble/src/core/decision_module/usv_decision' --packages-select path_processor path_planner usv_decision --build-base /tmp/usv_ladds_single_leg_20260915/build_python --install-base /tmp/usv_ladds_single_leg_20260915/install --symlink-install --executor sequential --allow-overriding path_processor path_planner usv_decision
