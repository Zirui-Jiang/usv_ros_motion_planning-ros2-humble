# Original C++ LQR straight-run stop diagnosis

The controller was actually LQR (runtime readback and factory log). Its solver
did not emit the existing "solved failed" warning in this run.

The existing log contains exactly one "GOAL Reached!" entry. Immediately
before the first zero command, the preceding odometry speed was 1.998918 m/s
and distance to the controller path endpoint was 0.950738 m.

Observed: the last nonzero commands were full reverse (-1 left / -1 right),
then two groups of zero commands. Command and local-trajectory publications
stopped while odometry continued. The closest sample to the requested goal was
0.318683 m at simulator time 7.5 s, still moving at 1.489385 m/s. The vessel
coasted to 2.734421 m away. The supervisor interrupted only after the documented
20-second zero-command stall rule. It did not run the 180-second timeout.

Source-supported direct cause:
- src/core/controller/controller/src/lqr_controller/otter_lqr_controller.cpp:
  88-95 latches is_goal_reached_ and clears thrust using distance alone; no speed
  or dwell requirement and no subsequent re-entry recovery are evaluated here.
- src/core/controller/controller/src/controller.cpp:96-99 clears the latch only
  when the global path endpoint changes.
- src/sim_env/src/simulation.cpp:243-245 moves the FSM to PENDING on that latch;
  238-240 skips future control callbacks, and 167-169 skips future navigation
  state/trajectory updates. Thus drifting back outside the goal radius cannot
  resume control for the same goal.
- The controller's global path endpoint is (35.25,-9.75), whereas the requested
  goal is (35,-10). The harness correctly judges the requested position.

The internal latch was not exported, so the exact branch transition is a
source-supported inference; stopping publication and the timestamped speed/
position/command evidence are directly observed. Full numeric evidence is in
lqr-stop-diagnosis.json and the unchanged events.jsonl.

Other original semantic defects are separate from this direct stop:
- LQR current-state assembly at 109-115 uses velocity norm and fixes lateral
  velocity to zero, losing signed surge and sway feedback.
- Trajectory::getReferencWithTime at trajectory.cpp:251-259 uses lower_bound's
  upper sample and idx+1, rather than the enclosing lower/upper pair. In this
  run lookahead=2.0 s is a multiple of the nominal 0.5 s trajectory interval;
  the ROS Path trace does not export trajectory time or desired velocity.
  Therefore this existing defect is not claimed as the observed stop trigger.
- The ROS Path trace also cannot establish the exact reference-speed history
  or the cause of earlier LQR overspeed. No substitute controller output or
  fabricated internal matrix/trajectory values are used.

No original C++ numerical algorithm was modified. Other LQR scenarios remain
unobserved in this matrix; they cannot be labelled failures or unavailable
merely because the runner paused the remaining schedule.
