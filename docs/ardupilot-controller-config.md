# ArduPilot controller configuration

`config/ardupilot-controller-config.json` is the adapter-owned runtime input.
It uses ArduPilot parameter names and is loaded by
`ArdupilotControllerConfigLoader`; no EEPROM-backed `AP_Param` store is used.

The mapped groups are `ATC_ANG_*` (attitude), `ATC_RAT_*` (body rate),
`PSC_POSZ_*`/`PSC_VELZ_*`/`PSC_ACCZ_*` (vertical position, velocity and
acceleration stages), `PSC_POSXY_*`/`PSC_VELXY_*` (horizontal position and
filtered velocity PID stages), and `MOT_THST_HOVER`.  The upstream kinematic
limits are configured with `WPNAV_SPEED_UP`, `WPNAV_SPEED_DN`,
`WPNAV_ACCEL_Z`, `PSC_JERK_Z`, `PSC_VELXY_MAX`, `PSC_ACCXY_MAX`,
`PSC_JERK_XY`, and `ANGLE_MAX_RAD` (SI units at this adapter boundary).

The altitude backend applies ArduPilot's angle-boost (tilt compensation) before
converting throttle to collective thrust normalized by `MOT_THST_HOVER`.
Runtime stage frequencies live in the `runtime` object. `dt_sec` supplied by
the caller remains authoritative for every state update.

## Takeoff and landing phases

The AP plugin is always built with `HAKO_ENABLE_FLIGHT_PHASE`. `takeoff_profile`
and `landing_profile` select the ArduCopter-style handling of the caller's
`Takeoff` and `Landing` phases. The profile reads `MOT_SPOOL_TIME`,
`TKOFF_SLEW_TIME`, `TKOFF_THR_MAX`, `LAND_SPEED`, `LAND_SPEED_HIGH`, and
`LAND_ALT_LOW`; time, throttle fraction, speed, and altitude use seconds,
0..1, m/s, and metres at the adapter boundary.

`ARMING_DELAY_SEC` is an adapter-only setting (default 2.0 seconds), named
after ArduCopter's compile-time constant rather than an EEPROM-backed
ArduPilot parameter. It is therefore not written by the ArduPilot parameter
exporter. The adapter has no arm timestamp, so it holds zero thrust and does
not advance the takeoff trajectory for this interval starting when `Takeoff`
is entered, before the existing spool and throttle-slew intervals. This
matches the supported mission and Commander sequence, which requests takeoff
immediately after arming; a takeoff requested after the vehicle has already
been armed for some time consequently differs from ArduCopter.

The implementation is independent code based on the state descriptions in
ArduCopter `takeoff.cpp:141-203` (spool and throttle slew),
`mode.cpp:640-664` (`land_run_vertical_control`), and
`land_detector.cpp:35-149` (0.2 s maybe / 1.0 s
complete timing). The common contract does not carry rangefinder, weight-on-
wheels, motor interlock, or attitude-error inputs, so those checks are not
reproduced. Ground height comes from the local position relative to the
takeoff point; contact uses height, vertical speed, acceleration, and the
adapter's lower-thrust limit. A completed landing is written to stderr as
`AP adapter landed phase_time_sec=...`; Commander retains its separate
service-side truth `landed` decision.

## Thrust units

The public allocator boundary uses normalized rotor thrust:

```text
u_i = T_i / T_hover_per_rotor
AP motor fraction = u_i / limit.max
u_i = AP motor fraction * limit.max
```

Thus hover collective is `body_z=-1` and each rotor outputs approximately
`1`. Actuator limits, trim, and linearization point use the same `u_i` unit.
They are not PWM duty values.

## Torque units

The public RateControl/ControlAllocation boundary uses ArduPilot's normalized
motor-axis command. `AC_PID::update_all()` plus `get_ff()` is passed directly
to the allocator, and unallocated torque is reported in the same normalized
unit. Geometry is taken from every `ControlAllocationInput` run. The allocator
derives ArduPilot's hover fraction as `1 / limit.max`; configuration-file
`MOT_THST_HOVER` is used by the altitude stage, not by allocation.

## EKF3 parameters

The current common `EkfAdapterConfig` exposes magnetic declination only; the
standalone EKF3 otherwise uses ArduPilot defaults. Loading a complete EKF3
`.parm` overlay remains unimplemented.
