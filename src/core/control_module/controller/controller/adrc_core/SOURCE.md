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

Source: repository-root `0924controller/`. In that delivery `adrc_controller.py`, `pi_controller.py` and `heading_linear_regression_filter.py` have the same SHA-256 as the 0914 originals above, so the ported heading/speed cores are unchanged. `lateral_controller.py` is not ported yet.

| Original file | SHA-256 | Port |
| --- | --- | --- |
| `thrust_allocator.py` | `ceee65f5719194a686365ff0b29ccb04350c3e9b1e51feb1a445d36153bd8a29` | copied unmodified; `test_thruster_output.py` checks the hash |

`controller/thruster_output.py` is the only caller. It rebuilds surge force and yaw moment from the published normalized command (`F = (l+r)·max_force`, `M = (r−l)·max_force·D`) and calls mode 0 with `propeller_spacing = 2·D`, so each propeller force equals `command × max_force` and only the force → rpm conversion is new. Saturation stays in the controllers; startup rejects rpm limits that cannot reach `max_force`, so the allocator's independent rpm clipping never binds.

Known issues of the delivered mode 1, which is therefore not called:

- Its yaw moment sign is opposite to mode 0 and to the documented counter-clockwise convention (+5 N·m request produces −5 N·m).
- Lateral force is documented as positive to the right, while ROS body y is positive to the left; the physical meaning of a positive thruster angle is not specified.
- Near zero surge force the angle jumps between 0 and π, and rpm is not reduced while an angle is still turning.
- Each propeller's rpm is clipped independently, which can remove the requested yaw moment at saturation.
- The delivered `__init__.py` lists `LateralController` in `__all__` without importing it.
