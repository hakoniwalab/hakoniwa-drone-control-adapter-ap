# hakoniwa-drone-control-adapter-ap

ArduPilot control backend implementation for the public interfaces provided by
[`hakoniwa-drone-control-adapter`](https://github.com/hakoniwalab/hakoniwa-drone-control-adapter).

## Status

This repository is an early technical evaluation. The first supported backend
is body-rate control using ArduPilot's own `AC_PID`, `SlewLimiter`, and scalar
low-pass filter implementations from the pinned ArduCopter 4.6.3 source tree.

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

The current scope intentionally excludes:

- integration into the normal `hakoniwa-drone-pro` build
- redistribution of a binary linked with proprietary components
- position, allocation-feedback, and EKF backends
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
```

`build.bash` uses `python3` by default. Select a Python environment containing
ArduPilot's build dependencies when necessary:

```bash
ARDUPILOT_PYTHON=/path/to/venv/bin/python bash build.bash build
```

The adapter compiles only the required ArduPilot controller/filter sources. It
does not link the complete ArduCopter vehicle application. The runtime shim
deliberately provides no EEPROM-backed `AP_Param` persistence: the adapter
configuration is the authoritative source of gains. ArduPilot dynamic notch
filters are also excluded because they require the full vehicle-level filter
singleton.

## Version pins

- ArduPilot: `Copter-4.6.3` / `92b0cd788ec29406f26c6f9c31d5ceedbd1cc538`
- Hakoniwa adapter interface: `8990200763fe2a4ec492d50ad47a4bd2090efe94`

These are Git submodule commits, not floating branch dependencies.

The build is intended for local evaluation. Do not publish the resulting
combined binary as a release, CI artifact, package, or container image without
performing the applicable GPL compliance review.

## License

This project is licensed under the GNU General Public License version 3 or
later (`GPL-3.0-or-later`). ArduPilot-derived components retain their upstream
copyright and license notices.
