# Gazebo Classic dependency installation

Completed on 2026-09-11 (Asia/Shanghai), in WSL distro Ubuntu-22.04.

The pre-existing apt-get update (PID 57038, parent script 56956) had stalled
for over 14 minutes. Only this task's two processes were terminated. The WSL
environment and apt configuration pointed at unavailable 127.0.0.1:10808.
Direct HTTPS tests succeeded for the Tsinghua Ubuntu/ROS2 mirrors and OSRF.

Changes:
- Original sources are backed up in apt-sources-20260910T161831Z/etc/apt/.
- Ubuntu archive/security URLs use https://mirrors.tuna.tsinghua.edu.cn/ubuntu/.
- ROS2 uses https://mirrors.tuna.tsinghua.edu.cn/ros2/ubuntu.
- OSRF keeps its original host and uses HTTPS.
- Existing signed-by keyrings and package signature checks remain enabled.
- /etc/apt/apt.conf.d/99usv-migration-mirrors selects DIRECT only for the
  Tsinghua mirror hostname, so later apt calls do not inherit the broken proxy
  for these new source URLs. The user's general proxy settings are unchanged.

Successful logs:
- apt-update-direct-20260910T161902Z.log
- apt-install-direct-20260910T161902Z.log
- installed-versions-20260910T161902Z.txt

Important environment impact: installing Gazebo Classic automatically removed
49 conflicting Garden/gz-sim7 packages, including Garden ROS bridges. This was
reported as soon as the transaction output was available. No user data was
purged, and no autoremove was run. Exact names and former versions appear in
the successful installation log and removed-garden-packages.txt. Future package
transactions must be simulated first and use --no-remove unless removal is
explicitly authorized.

Gazebo/libgazebo-dev are now 11.10.2+dfsg-1. dpkg --audit returned no findings.
Numba was not installed because the measured Python mapping kernel meets the
15 Hz budget.
