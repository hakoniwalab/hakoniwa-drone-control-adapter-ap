# ArduPilot adapter implementation

The adapter targets ArduCopter 4.6.3 and the public Control Link interface at
revision `1814967`.

| Stage | Implementation | Notes |
|---|---|---|
| Altitude | Upstream `AC_P_1D` and two `AC_PID` instances with the `AC_PosControl` z orchestration extracted | Position sqrt control (speed and acceleration limits), velocity/acceleration filters and feed-forward, hover conversion, and angle boost are retained. As in `AC_PosControl`, the velocity PID output feeds the acceleration loop without a clamp or slew limit; the jerk limit belongs to ArduPilot's target input shaping, which the adapter does not do (Drone PRO supplies the target). |
| Horizontal | Upstream `AC_P_2D` and `AC_PID_2D` with the `AC_PosControl` xy orchestration extracted | Position sqrt control (speed and acceleration limits), filtered velocity PID, the lean-angle limit on the acceleration target, and NED-to-FRD lean conversion are retained. As in `AC_PosControl`, there is no slew limit in the feedback path (the jerk limit belongs to input shaping). |
| Position3D | Not provided | The split backends cover PID-tuning phases; a faithful coupled facade was not completed. |
| Attitude | Quaternion controller extracted from `AC_AttitudeControl` | Produces FRD body-rate targets. |
| Rate | Three upstream `AC_PID` objects | `landed` clears integrators; `dt_sec`, filters, D/FF, slew max and slew tau are honored. |
| Allocation | `AP_MotorsMatrix` normalization/desaturation extraction | Fixed coplanar downward rotors, at most 12; public thrust is normalized by hover. |
| Feedback | Unallocated torque sign to per-axis saturation | Caller supplies the previous allocation status, giving the required one-cycle delay. |
| EKF | Upstream `NavEKF3` through DAL | One IMU/GPS/baro/compass; reset recreates the frontend and core. |

`set_armed_status`, `set_in_air_status`, and `set_vehicle_at_rest` are stored
independently. Drone PRO currently never calls `set_armed_status`; consequently
EKF3 observes the default disarmed state even after takeoff indication, which
can keep armed-only EKF behavior disabled. Integration must explicitly forward
the vehicle arming state.

The allocation output omits PWM conversion, thrust expo, spool state, battery
compensation, and lost-motor boost. The plant/caller converts normalized rotor
thrust to its actuator command.

Full `AC_PosControl` is not instantiated because its constructor requires the
vehicle AHRS, inertial navigation, motors, attitude controller, scheduler, and
parameter store. The controller primitives are linked directly; only the
vehicle-dependent stage orchestration is extracted.

## Sensor filters (plugin)

ArduPilot filters the IMU before its controllers see it (`AP_InertialSensor`, `INS_GYRO_FILTER` 20 Hz and
`INS_ACCEL_FILTER` 10 Hz by default). Drone PRO hands the adapter the unfiltered rate and acceleration, so the
plugin (`plugin/ap_sensor_filters.hpp`) applies the same second-order Butterworth low-pass (the coefficients of
`libraries/Filter/LowPassFilter2p.cpp`) to the rate-control input, sampled at `runtime.rate_hz`, and to the vertical
acceleration of the altitude and 3D position control, sampled at `runtime.altitude_hz`. The keys are optional in the
adapter configuration; absent or 0 means no filter (the contract-test configuration).

## Attitude acceleration limits

`ATC_ACCEL_R_MAX`, `ATC_ACCEL_P_MAX` and `ATC_ACCEL_Y_MAX` (centidegrees/s^2) feed the attitude-error-to-rate sqrt
controller, which clamps the limit into 40..720 deg/s^2 as `AC_AttitudeControl` does. The configuration loader
therefore gives an absent key ArduPilot's default (110000, 110000, 27000) rather than 0, which the clamp would turn
into a sluggish 40 deg/s^2.
