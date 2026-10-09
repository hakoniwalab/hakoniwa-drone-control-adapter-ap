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
