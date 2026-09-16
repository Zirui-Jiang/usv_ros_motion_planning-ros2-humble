# Costmap shutdown regression (ROS Humble)

普通导航拆分后，`test_navigation_ros.py` 使用本目录 `legacy_reference.py` 保留拆分前的 LiDAR 行为参照。该文件不安装，已按当前 LiDAR 范围移除停用传感器的分支。新生产节点的控制测试位于 `src/core/control_module/usv_control/test`，决策及跨模块闭环测试位于 `src/core/decision_module/usv_decision/test`；启动及 YAML 兼容测试为本目录 `test_launch_modules.py`。不能用旧参照测试通过代替新节点或 Gazebo 验证。

`costmap_shutdown_regression.py` runs the real `Costmap2DROS` executable in a
separate process and sends SIGINT. It requires ROS Humble and PyYAML, but does
not require Gazebo. Select a ROS domain that is not used by another experiment.

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=176
python3 src/sim_env/test/costmap_shutdown_regression.py \
  --executable install/sim_env/lib/sim_env/costmap_support_node \
  --artifacts-dir /tmp/costmap-shutdown-traffic --attempts 3
python3 src/sim_env/test/costmap_shutdown_regression.py \
  --executable install/sim_env/lib/sim_env/costmap_support_node \
  --artifacts-dir /tmp/costmap-shutdown-idle --scenario idle --attempts 3
python3 src/sim_env/test/costmap_shutdown_regression.py \
  --executable install/sim_env/lib/sim_env/costmap_support_node \
  --artifacts-dir /tmp/costmap-shutdown-startup --scenario startup_cancel --attempts 3
```

The traffic scenario uses a 400 m square costmap at 0.5 m resolution, obstacle
and inflation layers, `/clock`, static TF, and 2,000-beam scans at 15 Hz. It
queries the real lifecycle service and sends SIGINT after activation. Passing
requires the ACTIVE state, Deactivating/Cleaning up logs, and process exit 0.
The idle scenario uses only an inflation layer on a small map.

The startup-cancellation scenario deliberately withholds TF and sends SIGINT
while activation is waiting. It must return the explicit startup failure code
1, rather than abort or hang. Humble's interrupted transition can still log an
invalid-context exception; this scenario does not claim that activation or
normal lifecycle cleanup completed. The support-node destructor must release
the partially initialized resources safely.

Each run writes `summary.json` and one log per attempt. Diagnostic options
`--override PARAM:=VALUE`, `--no-scan`, and `--gdb` support narrowing the
trigger. GDB mode also inspects the log because GDB itself can exit 0 after
reporting a child SIGSEGV. The runner only signals or kills its own child
process group; it does not stop a ROS daemon or other simulations.

## Evidence for the fix

The original executable failed 3/3 traffic shutdowns with SIGSEGV after normal
lifecycle cleanup. Removing scan publication, simulation time, or reducing the
map size did not eliminate the failure. The main-thread GDB stack was
`rclcpp::CallbackGroup::~CallbackGroup` inside
`nav2_costmap_2d::Costmap2DROS::~Costmap2DROS`; spinner threads had already exited.
Keeping `liblayers.so` resident with a diagnostic-only `LD_PRELOAD` made the
unchanged binary pass 3/3, isolating the dynamic-library lifetime issue.

Humble declares `plugin_loader_` after `callback_group_`, so ordinary member
destruction can unload the obstacle-layer library before the callback group's
expired subscription control blocks are released. The support node now uses a
thin derived destructor to release those resources while the loader is alive.
It still uses the real configure/activate and Nav2 preshutdown lifecycle.

A separate startup-cancellation regression exposed SIGABRT in the base
destructor: an interrupted activation left a joinable map update thread after
preshutdown skipped the transitioning node. The derived destructor joins that
thread and also releases remaining layers and TF objects before the loader.

On the final implementation, the isolated WSL validation passed three traffic
shutdowns, one idle shutdown, and three TF-wait startup cancellations. Earlier
normal-shutdown-only validation also passed eleven runs. These are shutdown
and resource-lifetime checks, not navigation-accuracy or collision-avoidance
results. Full launch shutdown is checked separately by the simulation harness.

The reproducibility evidence from this development session is stored under
`/home/usv/usv_migration/costmap_shutdown/`: `repro-traffic/`,
`gdb-original/`, `probe-pinned-layer-library/`, `gdb-startup.log`,
`green-traffic-final/`, `green-idle-final/`, and `green-startup-tf-release/`.
Other `green-startup*` directories contain intermediate failed probes; consult
their JSON exit codes rather than interpreting the directory name as a result.
`pin_layers.bash`, source snapshots, and the standalone CMake build in that
directory are diagnostic artifacts and are not part of the application.
