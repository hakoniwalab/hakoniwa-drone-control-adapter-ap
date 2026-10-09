# Task

## Purpose

This file tracks the staged implementation of ArduPilot control backends for
the public `hakoniwa-drone-control-adapter` interfaces.

The adapter remains a GPL project separate from `hakoniwa-drone-pro`. Local
evaluation may link the two temporarily, but the normal PRO CMake build and
release artifacts must not acquire an ArduPilot dependency.

## Version Baseline

- ArduPilot: Copter 4.6.3 (`92b0cd788ec29406f26c6f9c31d5ceedbd1cc538`)
- Public Hakoniwa interface: `1814967f7a5410b7b44dd2fe87ba7140f9b0fc3e`
- Build target: ArduPilot SITL configuration
- License: `GPL-3.0-or-later`

Version upgrades are separate tasks. A backend is not considered portable to
another ArduPilot release until its parameter semantics and tests are rerun.

## Current Status

- [x] Add the public adapter interface as a pinned submodule
- [x] Add ArduPilot as a pinned submodule
- [x] Establish an independent GPL CMake boundary
- [x] Compile only the required ArduPilot sources instead of the full vehicle
- [x] Implement `ApRateControlBackend` with ArduPilot `AC_PID`
- [x] Map per-direction Hakoniwa saturation to ArduPilot's per-axis limit
- [x] Match ArduPilot's feedback plus feed-forward motor-input boundary
- [x] Add Rate backend smoke tests
- [x] Add the first fixed-rotor `ApControlAllocationBackend`
- [x] Port ArduPilot matrix normalization and desaturation into a pure backend
- [x] Test Quad/Hexa collective, roll differential, yaw clipping, and guards
- [x] Document the local-evaluation and redistribution boundary
- [x] Prove a minimal in-process NavEKF3 backend with one IMU, GPS, barometer,
  and magnetometer
- [x] Implement split altitude and horizontal-position backends
- [x] Implement allocation feedback
- [x] Add AP-named JSON configuration and loader
- [x] Add Release build, install target, and CMake package export
- [ ] Implement coupled `IPositionControl3DBackend` (declared not provided)
- [ ] Load a complete EKF3 parameter file

## Implementation Policy

- Reuse ArduPilot control source directly; do not create an "ArduPilot-like"
  reimplementation when the upstream controller can be isolated.
- Keep adapter configuration authoritative. Do not introduce ArduPilot EEPROM
  persistence into a standalone backend.
- Keep shims narrow and documented. A shim may supply platform services such
  as time or parameter initialization, but it must not replace control logic.
- Preserve NED world-frame and FRD body-frame semantics at every public
  boundary. Add explicit conversion tests where ArduPilot uses another unit or
  convention.
- Treat a public-interface change as a separate reviewed decision. First prove
  whether the pinned interface is sufficient.
- Every backend requires deterministic unit tests before PRO integration.

## Milestone 1: Rate Control

- [x] Wrap three ArduPilot `AC_PID` instances
- [x] Support P, I, D, FF, D-FF, IMAX, scalar filters, and slew limiting
- [x] Reject invalid or non-positive time steps
- [x] Test axis mapping, proportional output, feed-forward, and saturation
- [ ] Add reference-vector tests captured from an ArduCopter 4.6.3 run
- [ ] Add reset and runtime reconfiguration regression tests

Completion gate: the standalone backend and reference ArduCopter controller
produce equivalent per-axis outputs for the same input sequence.

## Milestone 2: Attitude Control

- [x] Investigate the minimum reusable boundary in
  `AC_AttitudeControl` / `AC_AttitudeControl_Multi`
- [x] Record dependencies on `AP_AHRS_View`, `AP_Motors`, input shaping, and
  angle/rate limits
- [x] Decide whether a narrow state facade can replace vehicle singletons
  without copying attitude-control logic
- [x] Implement `IAttitudeControlBackend`
- [x] Map public WXYZ quaternion and yaw-rate feed-forward into ArduPilot units
- [x] Preserve ArduPilot thrust-vector correction and quaternion error order
- [x] Add quaternion sign, yaw wrap, tilt limit, and reset tests
- [ ] Add ArduCopter 4.6.3 reference-vector comparison

Decision: directly instantiating `AC_AttitudeControl_Multi` would pull in
`AP_AHRS_View`, `AP_Motors`, parameter storage, and vehicle singletons. The
backend therefore extracts the pure quaternion controller. IMU `dt` and
measured body rate are controller-generic state, so they are explicit fields
of the common `AttitudeControlInput`; `context` remains reserved for optional
extensions. This preserves the large-tilt yaw safety path without a vehicle
singleton or ArduPilot-specific state facade.

Completion gate: attitude input produces equivalent body-rate targets without
requiring a live ArduPilot vehicle process.

## Milestone 3: Position Control

ArduPilot's position controller is a coupled 3-axis cascade. Implement the 3D
public backend first; derive the split horizontal/altitude adapters only after
the native controller boundary is stable.

### 3D Position Backend

- [ ] Investigate `AC_PosControl`, `AC_WPNav`, and their state dependencies
- [ ] Define a facade for position, velocity, acceleration, attitude, and time
- [ ] Map position and velocity modes to ArduPilot controller entry points
- [ ] Implement `IPositionControl3DBackend`
- [ ] Map ArduPilot thrust vector and yaw targets to public output semantics
- [ ] Test position, velocity, feed-forward, hover, and saturation behavior
- [ ] Add ArduCopter 4.6.3 reference-vector comparison

### Split Compatibility Backends

- [ ] Decide whether `IAltitudeControlBackend` can be a faithful view of the
  native 3D controller rather than an independent approximation
- [ ] Implement altitude position and velocity modes if equivalence is proven
- [ ] Decide the same for `IHorizontalPositionControlBackend`
- [ ] Document any behavior that cannot be represented by the split contracts

Completion gate: the 3D backend preserves ArduPilot's cascaded controller
state and the split adapters do not silently change its control law.

## Milestone 4: Control Allocation and Feedback

- [x] Investigate the first `AP_MotorsMatrix` fixed-rotor boundary
- [ ] Investigate the separate `AP_MotorsMatrix_6DoF` boundary
- [x] Preserve Hakoniwa actuator input order at the pure geometry boundary
- [x] Implement `IControlAllocationBackend` for fixed coplanar rotor geometry
- [ ] Preserve actuator min/max, trim, lost-motor, and clipping semantics
- [x] Explicitly reject unsupported geometries and rotor counts
- [x] Document the Copter 4.6.3 maximum of 12 motors while retaining the
  public interface capacity of 16
- [ ] Implement `IAllocationFeedbackPolicy` from ArduPilot motor-limit flags
- [x] Test Quad/X and Hexa geometry, collective, differential output, and clipping
- [ ] Test Hexa/DJI-X ordering at the configuration-converter boundary
- [ ] Add one-motor failure and thrust-boost handling, or reject it explicitly
- [ ] Add allocation reference vectors from ArduCopter 4.6.3

Completion gate: actuator ordering and saturation feedback are proven for the
9 kg Hexa before any flight evaluation.

## Milestone 5: Configuration and Usage Harness

- [ ] Define an adapter-owned JSON schema for ArduPilot controller parameters
- [ ] Add a converter from `.parm` values into the runtime JSON
- [ ] Keep firmware migration separate from simulation compatibility overlays
- [ ] Add a C++ configuration loader and validation diagnostics
- [ ] Add one small GPL executable harness composing position, attitude, rate,
  allocation, and feedback backends
- [ ] Add deterministic frequency tests at 333 Hz, 400 Hz, and 800 Hz
- [ ] Add local-only instructions for temporary PRO linkage
- [ ] Confirm that PRO builds without this repository remain unchanged

Completion gate: a configuration-only test constructs every implemented
backend and a local integration test can run without modifying PRO CMake.

## Milestone 6: EKF3 Feasibility

This milestone is being completed independently before position control because
PID auto-tuning before SITL is a primary use case and EKF3 is the highest-risk
backend. The implementation uses ArduPilot's own DAL/replay boundary; it does
not reproduce the estimator equations or embed the complete flight controller.

- [x] Map `IEkfAdapter` inputs to ArduPilot IMU, magnetometer, barometer, and
  GPS sample contracts
- [x] Inventory all `AP_NavEKF3` singleton and scheduler dependencies
- [x] Decide whether direct library isolation is maintainable
- [ ] If isolation is not maintainable, document a process-boundary adapter as
  the preferred design instead of growing a large fake HAL
- [x] Map EKF validity, aiding-source, fusion, and innovation status outputs
- [x] Add timestamp monotonicity, static convergence, and constant-velocity
  sensor-consistency smoke tests
- [ ] Compare output against ArduCopter 4.6.3 using identical sensor vectors

Current feasibility result: direct isolation is maintainable for the first
evaluation. The backend links the Waf-produced `AP_DAL_libs` archive and feeds
real `NavEKF3` through `log_RISH/RISI`, `RMGH/RMGI`, `RBRH/RBRI`, and
`RGPH/RGPI/RGPJ`. A 60-second 400 Hz static-vector test reaches valid attitude,
local-position, and global-position outputs.

First-evaluation limits are explicit:

- one EKF instance, one body-centred IMU, one GPS, one barometer, one compass
- ArduPilot defaults plus the common magnetic-declination setting; `.parm`
  overlay support is deferred
- `armed`, `in_air`, and `at_rest` are independent common-interface inputs
- runtime reset recreates the NavEKF3 frontend and core
- no lane switching, GSF yaw, optical flow, rangefinder, visual odometry, or
  hardware sensor drivers

The only upstream adaptation is a build-time, reversible patch that makes the
standalone DAL use a zero IMU lever arm instead of calling the live
`AP_InertialSensor` singleton. `build.bash` applies it only while compiling and
restores the pinned submodule on exit.

Completion gate: the architectural feasibility gate has passed. Numerical
parity is not claimed until the same recorded DAL input sequence is replayed by
both ArduCopter 4.6.3 and this backend.

## Final Validation

- [ ] Run standalone unit and reference-vector tests on macOS
- [ ] Run the same tests on Linux
- [ ] Run AddressSanitizer and UndefinedBehaviorSanitizer where supported
- [ ] Validate Quad and 9 kg Hexa control composition
- [ ] Verify a clean `hakoniwa-drone-pro` build with the adapter absent
- [ ] Review GPL source, notice, artifact, and distribution obligations
- [ ] Tag the first evaluation release only after all applicable gates pass

## Out of Scope for the First Evaluation

- Shipping a combined proprietary/GPL binary
- Adding this adapter to the default `hakoniwa-drone-pro` dependency graph
- Supporting more than 12 ArduPilot motor outputs on Copter 4.6.3
- Claiming compatibility with newer ArduPilot versions without revalidation
- Replacing the established MAVLink/AirSim SITL path before backend parity is
  demonstrated
