# 0914 controller numerical port

Source: repository-root `0914controller/`; the four delivered files are preserved byte-for-byte.

| Original file | SHA-256 |
| --- | --- |
| `__init__.py` | `5c3d0a2dcf2536eb8a1e771f457383f6f887489f56dff0df31abbbf4b863d2dd` |
| `adrc_controller.py` | `896db001ef99261d8997f03ec1e87e58736b65dce9b9c63d31f912524f4796aa` |
| `heading_linear_regression_filter.py` | `ba4ae6e8994c8a0a3162b8a5d62e4952d6fe5e2dd37c708f62f0f0c92db0192e` |
| `pi_controller.py` | `91ab3f51b176a9d36af6998e362dcef081ee180330e0b54481751e47e5d78956` |

## Original unsaturated baseline

Defaults, desired heading 0.1 rad, desired speed 0.5 m/s, measured speed 0.2 m/s, dt 0.1 s. Entries are `[measured_heading, surge_force, yaw_moment]`:

```json
[
  [
    0.0,
    2.424,
    0.1503
  ],
  [
    0.01,
    2.448,
    0.08557000000000003
  ],
  [
    0.025,
    2.472,
    0.0437141666666667
  ],
  [
    0.03,
    2.496,
    0.06383483333333335
  ]
]
```

## Necessary deviations

- Unwrap accumulates the shortest adjacent raw-angle difference. Physical intersample angle increments must remain below pi.
- Filter accepts keyword stamp/dt and regresses position and rate against actual relative sample times; a singleton has zero derivative. Duplicate timestamps do not mutate histories; backwards time is rejected (outer adapter owns resets/gap policy).
- Finite scalar/configuration validation, strict integer window/history validation, ordered integral and output bounds, output bounds containing zero, nonnegative gains, and error-reset angle in (0, 180] are checked before configuration commit.
- Changes to heading sample_time/window/history rebuild and reset filter/controller history. Gain-only updates retain history.
- PI and heading output are recomputed from the final stored integral after conditional saturation freeze. Large heading error clears the integral before output. This deliberately changes the original saturated/reset first-step semantics.
- Both cores cache non_integral_output and expose refresh_output() for outer allocation anti-windup without additional integration/filtering. Heading integral_reset marks a mandatory reset that the adapter must preserve.
- Nonfinite arithmetic is rejected before clipping. No gain/default limit tuning is included inside these core defaults.

## Verification

TDD run before fixes: 23 failed, 7 passed. Both positive/negative multiwrap tests first failed at 9.5 rad (returned magnitude 3.2168146928204133 rad); timed update failed because stamp/dt keywords were unsupported. Dedicated tests verify final integral/output consistency and invalid configuration atomicity.

Read-only original fixed-period diagnostic: measurements `[0, 0.2, 0.4]` with actual 0.2 s intervals and configured 0.1 s period return 2.0 rad/s instead of 1.0 rad/s. Corrected explicit-dt regression returns 1.0 rad/s.

Final core suite: `python -B -m pytest -q -p no:cacheprovider src/core/control_module/controller/test/test_adrc_core.py`: 33 passed. `git diff --check` for owned paths: no whitespace errors.

# 0924 thrust allocator

Source: repository-root `0924controller/`. In that delivery `adrc_controller.py`, `pi_controller.py` and `heading_linear_regression_filter.py` have the same SHA-256 as the 0914 originals above, so the ported heading/speed cores are unchanged. `lateral_controller.py` is described in the next section.

| Original file | SHA-256 | Port |
| --- | --- | --- |
| `thrust_allocator.py` | `ceee65f5719194a686365ff0b29ccb04350c3e9b1e51feb1a445d36153bd8a29` | copied unmodified; `test_thruster_output.py` checks the hash |

`controller/thruster_output.py` is the only caller. It rebuilds surge force and yaw moment from the published normalized command (`F = (l+r)·max_force`, `M = (r−l)·max_force·D`) and calls mode 0 with `propeller_spacing = 2·D`, so each propeller force equals `command × max_force` and only the force → rpm conversion is new. Saturation stays in the controllers; startup rejects rpm limits that cannot reach `max_force`, so the allocator's independent rpm clipping never binds.

For a vessel with two full-rotatable thrusters (`dynamics.type: Azimuth`) `thruster_output.py` allocates surge force, lateral force and yaw moment itself and marks the setpoint mode 1. It uses the delivered allocator only for the propeller model (`_force_to_rpm`, `_rpm_to_force`) and the rpm limits. Angles are in the ROS body frame, positive toward port, and stay within `thrust_allocator.max_deflection`; reverse thrust uses negative rpm. The lateral force is shared in proportion to the size of each thruster's longitudinal thrust, the differential longitudinal thrust also cancels the yaw moment of a lateral force at the stern (`dynamics.thruster_x`), and all requests are scaled together at saturation.

Known issues of the delivered mode 1, which is therefore not called:

- Its yaw moment sign is opposite to mode 0 and to the documented counter-clockwise convention (+5 N·m request produces −5 N·m).
- Lateral force is documented as positive to the right, while ROS body y is positive to the left; the physical meaning of a positive thruster angle is not specified.
- Near zero surge force the angle jumps between 0 and π, and rpm is not reduced while an angle is still turning.
- Each propeller's rpm is clipped independently, which can remove the requested yaw moment at saturation.
- It assumes that a lateral force produces no yaw moment. On a hull with both thrusters at the stern a lateral request then turns the vessel.
- It splits the lateral force equally. A thruster whose longitudinal thrust changes sign keeps half of the lateral force, so its thrust direction passes through abeam: the thruster must swing a quarter turn or more while still producing thrust. A headless WAM-V run with this geometry (signs and stern moment corrected) reversed and swung the stern repeatedly, with thruster angles reaching ±3 rad.
- The delivered `__init__.py` lists `LateralController` in `__all__` without importing it.

# 0924 lateral controller

Source: repository-root `0924controller/`.

| Original file | SHA-256 | Port |
| --- | --- | --- |
| `lateral_controller.py` | `e58846243d258b967564abdc192f902f418385c0427d83224d24f86da60de6e4` | copied unmodified; `test_lateral_thrust.py` checks the hash |

`controller/lateral_thrust.py` is the only caller. It turns the delivered force into a normalized lateral command; `thruster_output.py` allocates that command to two full-rotatable thrusters, or it drives a separate lateral thruster directly (`dynamics.type: Omni`).

- The delivered convention is kept inside the core: distance and force positive to starboard. The adapter passes the vessel's offset from the reference as `measured_lateral_distance` with `lateral_distance = 0`, and negates the returned force because ROS body y and the thruster command are positive to port.
- The offset is measured to the closest point of the global route. The decision module attaches the part of the route near the vessel to every reference (`NavigationReference.route_x/route_y`, carried as `Trajectory.route`), and `closest_route_point` is evaluated with the latest measured pose on every control tick. A reference without a route gives no lateral thrust. In ordered-route navigation each plan reconnects from the current vessel position, so the route follows the vessel and the offset stays near zero.
- `mode` is 1 while the loop is enabled (`lateral_controller.enabled`) and 0 otherwise. The step passed as `dt` is the difference of consecutive measurement stamps; a repeated stamp returns the previous output, and a gap above `max_sample_gap` resets the core. These two adapter keys are not part of the delivery.
- Startup rejects an enabled loop on a vessel that cannot produce lateral force, and force limits that do not contain zero or exceed `dynamics.max_lateral_force`. The force is divided by `max_lateral_force` to give the normalized command.
- `LateralAssistedController` adds only the lateral component to a successful tracking command of PID, LQR or ADRC. Goal reached, settling, failures, braking and holding keep `lateral == 0`; the stopping-tail prediction in `common/dynamics.py` is unchanged and still rejects lateral thrust.
- The delivered derivative is an unfiltered finite difference of the error. With the route offset it is the vessel's lateral speed relative to the route, except for a one-sample step when the route itself is replaced. `controller_params_wamv.yaml` uses proportional plus derivative action: sway damping of the simulated WAM-V is low, so the proportional term alone oscillates about the route. `k_i_lateral` is zero because the offset is large for a few seconds in a turn and the accumulated one-sided force is not unwound by the small errors that follow.
- A separate lateral thruster (`Omni`) has no field in `ThrusterCommand`/0054; `ThrusterOutput` converts only the two longitudinal propellers for such a vessel. With two full-rotatable thrusters the lateral force is part of the two angles and rpm values in that message.
