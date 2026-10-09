# hakoniwa-drone-control-adapter-ap

ArduPilot control backend implementation for the public interfaces provided by
[`hakoniwa-drone-control-adapter`](https://github.com/hakoniwalab/hakoniwa-drone-control-adapter).

## Status

This repository implements the ArduPilot Control Link attitude, rate,
altitude, horizontal-position, allocation, allocation-feedback, and EKF3
backends using the pinned ArduCopter 4.6.3 source tree. Coupled 3D position
control is explicitly not provided.

The spike confirms that the existing `IRateControlBackend` contract can host
ArduPilot rate control without changing the public interface:

- `RateControlInput::dt_sec` supplies the controller time step.
- Roll, pitch, and yaw target/measured rates map directly to three `AC_PID`
  instances.
- The public positive/negative saturation flags are combined into ArduPilot's
  single per-axis integrator limit flag.
- `AC_PID::update_all()` and `AC_PID::get_ff()` are summed at the adapter
boundary, matching the value that ArduPilot ultimately passes to its motor
layer.

The first attitude backend maps WXYZ current/target quaternions and yaw-rate
feed-forward to an ArduPilot-compatible body-rate target. It extracts the
ordered thrust-vector and heading correction from ArduCopter 4.6.3, including
the square-root angle controller, yaw-error guard, rate limits, and large-tilt
feed-forward suppression.

`AC_AttitudeControl` normally reads the IMU time step and measured gyro through
vehicle objects. Both values are controller-generic, so the public
`AttitudeControlInput` now carries `dt_sec` and measured body `rate` directly.
No ArduPilot-specific state facade or vehicle singleton is required.

The current implementation also includes a first control-allocation backend.
It adapts the normalization and desaturation sequence from ArduPilot
`AP_MotorsMatrix` to the pure `IControlAllocationBackend` contract.

Supported in the first allocator:

- 1 to 12 fixed, coplanar rotors with downward FRD thrust axes
- geometry-derived roll, pitch, and yaw factors
- ArduPilot matrix-factor normalization
- yaw headroom and R/P/Y desaturation
- Quad and Hexa geometry supplied in Hakoniwa actuator order

Its output is normalized motor thrust before ArduPilot's `MOT_THST_EXPO`, spin
range, PWM, spool-state, battery-compensation, and lost-motor processing. Those
vehicle-layer features are not silently approximated by the allocator.

The optional EKF3 backend runs ArduPilot's actual `NavEKF3` in process through
the upstream `AP_DAL` replay boundary. It accepts one body-centred IMU, GPS,
barometer, and magnetometer through `IEkfAdapter`; it does not link the full
ArduCopter vehicle or instantiate hardware sensor drivers. The initial static
test feeds 60 seconds of 400 Hz IMU data and verifies valid attitude, local
position, global position, bounded velocity, and strict IMU timestamp
monotonicity. A separate constant-velocity test verifies that northward GPS
position and velocity produce the expected NED estimate without east-axis
leakage. The test also covers the distinct armed-on-ground and in-air states;
the backend does not infer one from the other.

The current scope intentionally excludes:

- integration into the normal `hakoniwa-drone-pro` build
- redistribution of a binary linked with proprietary components
- coupled 3D position control
- multi-lane EKF, optional aiding sensors, and `.parm`
  configuration overlays
- tilted/reversible rotors and ArduPilot failed-motor thrust boost

The attitude implementation has deterministic adapter tests. Direct
reference-vector comparison against a live ArduCopter 4.6.3 controller remains
an explicit completion gate before claiming full numerical parity.

## Source layout

- `thirdparty/hakoniwa-drone-control-adapter`: public MIT-licensed interface
- `thirdparty/ardupilot`: pinned GPL-3.0-or-later ArduPilot source
- `include/` and `src/`: GPL adapter implementations and narrow runtime shim
- `test/`: adapter-level regression tests

## Build

Initialize the pinned dependencies:

```bash
git submodule update --init --recursive
```

Generate ArduPilot's SITL build configuration, then build the adapter and run
its tests:

```bash
bash build.bash build
bash build.bash test
bash build.bash install
```

`build.bash` uses `python3` by default. Select a Python environment containing
ArduPilot's build dependencies when necessary:

```bash
ARDUPILOT_PYTHON=/path/to/venv/bin/python bash build.bash build
```

The adapter compiles only the required ArduPilot controller/filter sources and,
when invoked through `build.bash`, Waf's standalone DAL library. It does not
link the complete ArduCopter vehicle application. The runtime shim deliberately
provides no EEPROM-backed `AP_Param` persistence: adapter configuration is the
authoritative source of control gains, while EKF3 currently uses upstream
defaults. ArduPilot dynamic notch filters are excluded because they require the
full vehicle-level filter singleton.

`build.bash` enables EKF3 explicitly and temporarily applies the narrow patch
in `patches/` that replaces the standalone DAL's live IMU-position lookup with
a zero lever arm. The patch is reversed automatically when the command exits,
so the pinned ArduPilot submodule remains clean. Plain CMake configuration keeps
`HAKONIWA_AP_ENABLE_EKF3=OFF` by default; this preserves the pre-EKF controller
build when the Waf DAL archive is unavailable.

## PID tuning and SITL (Drone PRO)

The adapter is tuned with Drone PRO's `plugin` tuning profile under the conditions of ArduPilot SITL, and the
result is exported as ArduPilot parameters for ArduCopter running as its own process. The procedure is in Drone
PRO's `pro-docs/control-link/ardupilot/` (`pid-tuning-procedure.md`, `sitl.md`). This repository holds the
ArduPilot-specific parts:

| File | Purpose |
|---|---|
| `tuning/sitl-3ms-timing.json` | Timing profile: every Hakoniwa control cycle 3 ms, as ArduCopter's main loop in SITL |
| `tuning/profile-overrides.json` | EKF wait (+32 s), ArduPilot search ranges (rate P 0.05..0.5, rate I searched in hover) and hard gates (yaw phase 90 deg) |
| `tuning/export_ardupilot_params.py` | Tuning result -> ArduPilot parameters (`ATC_*`, `PSC_*`, limits, `MOT_THST_HOVER/EXPO`, frame) |
| `tools/build-sitl.bash` | Builds `arducopter` with MAVLink from the pinned ArduPilot (separate waf output) |
| `tools/setup_sitl_runtime.py` | Makes a SITL runtime: exported parameters, SITL-only parameters, the vehicle for Drone PRO's aircraft service, launch scripts |

The adapter configuration used for tuning carries ArduPilot's sensor filters (`INS_GYRO_FILTER`, `INS_ACCEL_FILTER`)
and attitude acceleration limits (`ATC_ACCEL_*_MAX`, ArduPilot defaults when absent), so the tuned gains hold in SITL.
Verified on EAMS (9 kg hexa, 2026-10-09): five tuning phases pass, and ArduPilot SITL flies takeoff, goto and land
with the exported parameters unchanged.

## Version pins

- ArduPilot: `Copter-4.6.3` / `92b0cd788ec29406f26c6f9c31d5ceedbd1cc538`
- Hakoniwa adapter interface: `1814967f7a5410b7b44dd2fe87ba7140f9b0fc3e`

These are Git submodule commits, not floating branch dependencies.

The build is intended for local evaluation. Do not publish the resulting
combined binary as a release, CI artifact, package, or container image without
performing the applicable GPL compliance review.

## License

This project is licensed under the GNU General Public License version 3 or
later (`GPL-3.0-or-later`). ArduPilot-derived components retain their upstream
copyright and license notices.
