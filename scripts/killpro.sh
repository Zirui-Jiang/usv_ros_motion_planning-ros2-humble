#!/bin/bash
# Stop the usv_ros_motion_planning simulation (ROS 2 Humble).
#
# `ros2 launch` starts every node as its own child executable. Killing the
# `ros2` CLI alone (the old `killall -9 ros2`) ORPHANS those children, so each
# run leaked a full set of nodes (robot_state_publisher, fake_localization_node,
# perception_mapping_node, ...). The orphans keep publishing to the global /tf
# and corrupt the TF tree of the next run (e.g. TF_OLD_DATA on otter/base_link).
#
# `killall` also can't match long executable names (its /proc/<pid>/comm is
# truncated to 15 chars, so "robot_state_publisher" never matches). Match the
# full command line via `ps` and kill by PID instead, which is reliable.

# Full-command-line substrings identifying this simulation's processes.
PATTERNS=(
  'gzserver'
  'gzclient'
  'rviz2'
  'ros2 launch sim_env'
  'install/sim_env/lib'        # sim_env_node, fake_localization_node
  'install/perception/lib'     # perception_mapping_node, usv_tf_expander.py
  'robot_state_publisher'
  'robot_localization'         # ekf_node, navsat_transform_node (legacy)
  'topic_tools/relay'          # odom_relay (legacy)
  'spawn_entity'
)

# Two passes: kill children, then anything a dying launcher may have left.
for _ in 1 2; do
  for pat in "${PATTERNS[@]}"; do
    pids=$(ps -eo pid,cmd 2>/dev/null | grep -F "$pat" | grep -v grep | awk '{print $1}')
    for p in ${pids}; do kill -9 "${p}" 2>/dev/null; done
  done
  sleep 0.5
done

echo "[killpro] simulation processes stopped."
